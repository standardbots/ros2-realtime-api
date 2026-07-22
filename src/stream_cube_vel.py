import os
import time
import rclpy
import argparse
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from ikpy.chain import Chain

from detect_bot_id import detect_bot_id

# Cube corners as (x, y, z) offsets in units of the cube side, ordered so that
# consecutive corners differ on a single axis (a Gray-code Hamiltonian cycle).
# Every move is therefore axis-aligned, all 8 corners are visited, and the path
# returns to the starting corner. (Same path as stream_cube.py.)
CUBE_CORNERS = [
    (0, 0, 0),
    (0, 0, 1),
    (0, 1, 1),
    (0, 1, 0),
    (1, 1, 0),
    (1, 1, 1),
    (1, 0, 1),
    (1, 0, 0),
    (0, 0, 0),
]

URDF_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_urdfs")

# Abort thresholds for the precomputed joint path. A per-tick step above
# MAX_JOINT_STEP means the IK solver jumped to a different solution branch; an
# FK error above MAX_FK_ERROR means it failed to converge (usually a cube
# corner outside the reachable workspace at the fixed orientation). Streaming
# either would command a violent or wrong move, so we refuse instead.
MAX_JOINT_STEP = 0.03  # rad between consecutive setpoints
MAX_FK_ERROR = 0.005   # m between requested and solved tooltip position


def load_chain(robot: str) -> Chain:
    urdf = os.path.join(URDF_DIR, f"{robot}.urdf")
    # base_link -> joint0..joint5 -> flange; only the 6 revolute joints are
    # active (indices 1-6), the rest are fixed frames.
    return Chain.from_urdf_file(
        urdf,
        base_elements=["base_link"],
        active_links_mask=[False, True, True, True, True, True, True, False, False],
    )


def cube_cartesian_path(origin, size, speed, accel, settle, rate):
    """Tooltip positions for the cube trace, one per tick at `rate` Hz.

    Each edge follows the same trapezoidal speed profile as stream_cube.py
    (ramp at `accel`, cruise at `speed`), and each corner is held for `settle`
    seconds so the arm comes to rest there.
    """
    dt = 1.0 / rate
    corners = [np.asarray(origin) + np.asarray(c) * size for c in CUBE_CORNERS]
    points = [corners[0]]
    for start, target in zip(corners[:-1], corners[1:]):
        distance = float(np.linalg.norm(target - start))
        v, a = speed, accel
        ramp = v * v / (2 * a)  # distance to go 0 -> v (and v -> 0)

        if 2 * ramp <= distance:
            # Trapezoid: accelerate, cruise, decelerate.
            t_acc = v / a
            t_cruise = (distance - 2 * ramp) / v
            v_peak = v
        else:
            # Triangle: edge too short to reach cruise speed.
            v_peak = (a * distance) ** 0.5
            t_acc = v_peak / a
            t_cruise = 0.0
            ramp = distance / 2

        total_time = 2 * t_acc + t_cruise
        steps = max(1, int(round(total_time * rate)))

        for i in range(1, steps + 1):
            t = i * dt
            if t < t_acc:                        # accelerating
                s = 0.5 * a * t * t
            elif t < t_acc + t_cruise:           # cruising
                s = ramp + v_peak * (t - t_acc)
            else:                                # decelerating
                td = t - t_acc - t_cruise
                s = ramp + v_peak * t_cruise + v_peak * td - 0.5 * a * td * td

            f = min(s / distance, 1.0)
            points.append(start + (target - start) * f)

        # Hold the corner so the arm settles.
        points.extend([target] * max(1, int(settle * rate)))

    return np.array(points)


def solve_joint_trajectory(chain, start_joints, points, rate):
    """IK-solve the cartesian path into joint positions + velocities.

    Solves every tick seeded with the previous solution (keeps the solver on
    one branch), holds the starting tooltip orientation, then differentiates
    the solved joint path (central difference) to get per-tick joint
    velocities. Aborts if the solution jumps branches or fails to converge.
    """
    n_links = len(chain.links)
    full = np.zeros(n_links)
    full[1:7] = start_joints
    orientation = chain.forward_kinematics(full)[:3, :3]

    positions = np.zeros((len(points), 6))
    report_every = max(1, len(points) // 10)
    for k, p in enumerate(points):
        full = chain.inverse_kinematics(
            p, orientation, orientation_mode="all", initial_position=full
        )
        positions[k] = full[1:7]

        fk_err = float(np.linalg.norm(chain.forward_kinematics(full)[:3, 3] - p))
        if fk_err > MAX_FK_ERROR:
            raise RuntimeError(
                f"IK did not converge at point {k}/{len(points)} (error "
                f"{1000 * fk_err:.1f} mm) - is the cube (+X/+Y/+Z from the "
                f"current pose) fully inside the reachable workspace? Try a "
                f"different start pose, a smaller --size, or check --robot."
            )
        if k % report_every == 0:
            print(f"  IK {k}/{len(points)}...")

    steps = np.abs(np.diff(positions, axis=0))
    if steps.size and steps.max() > MAX_JOINT_STEP:
        raise RuntimeError(
            f"IK solution jumped {steps.max():.3f} rad between consecutive "
            f"setpoints (branch flip) - refusing to stream. Try a different "
            f"start pose or a smaller --size."
        )

    dt = 1.0 / rate
    velocities = np.zeros_like(positions)
    velocities[1:-1] = (positions[2:] - positions[:-2]) / (2 * dt)
    return positions, velocities


class StreamCubeVel(Node):
    """Joint streaming over the external-control bridge: trace a cube with the
    tooltip via client-side IK, streaming joint positions AND velocities."""

    def __init__(self, robot_id: str | None = None):
        super().__init__("stream_cube_vel")

        if robot_id is None:
            self.robot_id = detect_bot_id(self)
        else:
            self.robot_id = robot_id

        # The external-control bridge publishes/subscribes under /<robot_id>/external
        base = f"/{self.robot_id}/external"

        # Use BEST_EFFORT QoS to match the bridge publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher = self.create_publisher(
            JointState,
            f"{base}/joints",
            10
        )

        self.start_positions = None
        self.joint_names = []
        self.observed = []  # (monotonic time, joint positions) while streaming
        self.recording = False

        self.subscription = self.create_subscription(
            JointState,
            f"{base}/robot_joints",
            self.joint_state_callback,
            qos_profile
        )

    def joint_state_callback(self, msg):
        if self.start_positions is None and msg.position:
            self.start_positions = list(msg.position)
            self.joint_names = list(msg.name)
        if self.recording and msg.position:
            self.observed.append((time.monotonic(), list(msg.position[:6])))

    def report_tracking(self, t0, positions, rate):
        """Compare robot_joints observed during the stream against the
        commanded setpoints at the same instants."""
        if not self.observed:
            print("No robot_joints received while streaming - no tracking stats.")
            return
        errors = []
        for t, q_obs in self.observed:
            k = int(round((t - t0) * rate))
            if 0 <= k < len(positions) and len(q_obs) == 6:
                errors.append(np.abs(np.asarray(q_obs) - positions[k]).max())
        if not errors:
            return
        errors = np.array(errors)
        print(f"Tracking vs commanded ({len(errors)} samples): "
              f"worst-joint error mean {np.degrees(errors.mean()):.2f} deg, "
              f"max {np.degrees(errors.max()):.2f} deg")
        print("(error = lag + smoothing; expect it to drop sharply when "
              "velocities are streamed and trustClientStreamVelocity is on)")

    def start(self, robot, size, speed, accel, settle, rate, use_velocities, dry_run):
        print("Waiting for start joint state...")
        deadline = time.monotonic() + 10
        while self.start_positions is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.start_positions is None:
            print("No robot_joints received - is the bridge engaged?")
            return

        if len(self.start_positions) < 6:
            print(f"Expected >= 6 joints, got {len(self.start_positions)}")
            return

        start_joints = np.asarray(self.start_positions[:6])
        print("Start joints:", np.round(start_joints, 4).tolist())

        print(f"Loading {robot}.urdf and precomputing IK "
              f"({size * 100:.0f}cm cube, {speed * 100:.0f}cm/s, {rate:.0f}Hz)...")
        chain = load_chain(robot)
        n_links = len(chain.links)
        full = np.zeros(n_links)
        full[1:7] = start_joints
        origin = chain.forward_kinematics(full)[:3, 3]
        print("Tooltip (FK):", np.round(origin, 4).tolist(),
              "- cube extends +X/+Y/+Z from here")

        points = cube_cartesian_path(origin, size, speed, accel, settle, rate)
        positions, velocities = solve_joint_trajectory(chain, start_joints, points, rate)
        duration = len(positions) / rate
        print(f"Trajectory: {len(positions)} setpoints, {duration:.1f}s, "
              f"peak joint velocity {np.abs(velocities).max():.3f} rad/s, "
              f"velocities {'ON' if use_velocities else 'OFF (position-only)'}")

        if dry_run:
            print("Dry run - not streaming.")
            return

        names = self.joint_names[:6] if len(self.joint_names) >= 6 else \
            [f"joint{i}" for i in range(6)]
        dt = 1.0 / rate

        self.recording = True
        t0 = time.monotonic()
        t_next = t0
        for k in range(len(positions)):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = names
            msg.position = positions[k].tolist()
            if use_velocities:
                # The bridge forwards velocity only when it is one-per-joint;
                # the robot uses it only when trustClientStreamVelocity is on.
                msg.velocity = velocities[k].tolist()
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0)

            t_next += dt
            sleep_for = t_next - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
        self.recording = False

        print("Done")
        self.report_tracking(t0, positions, rate)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Trace a cube with the tooltip by streaming joint positions '
                    '+ velocities (client-side IK) over the external-control bridge')
    parser.add_argument('--bot-id', dest='bot_id', type=str, help='Robot ID to stream joints to')
    parser.add_argument('--robot', dest='robot', type=str, default='thor',
                        choices=['core', 'spark', 'thor'],
                        help='URDF to use for FK/IK (default: thor)')
    parser.add_argument('--size', dest='size', type=float, default=0.10,
                        help='Cube side length in meters (default: 0.10)')
    parser.add_argument('--speed', dest='speed', type=float, default=0.05,
                        help='Cruise Cartesian speed in m/s (default: 0.05)')
    parser.add_argument('--accel', dest='accel', type=float, default=0.2,
                        help='Cartesian acceleration in m/s^2 for the trapezoidal ramp (default: 0.2)')
    parser.add_argument('--settle', dest='settle', type=float, default=0.5,
                        help='Seconds to hold each corner (default: 0.5)')
    parser.add_argument('--rate', dest='rate', type=float, default=100.0,
                        help='Setpoint stream rate in Hz (default: 100)')
    parser.add_argument('--no-velocities', dest='no_velocities', action='store_true',
                        help='Stream positions only (A/B baseline: the robot smooths '
                             'the position stream and derives velocity itself)')
    parser.add_argument('--dry-run', dest='dry_run', action='store_true',
                        help='Precompute and validate the trajectory, do not stream')
    args = parser.parse_args()

    rclpy.init()

    if args.bot_id is None:
        args.bot_id = detect_bot_id()

    node = StreamCubeVel(robot_id=args.bot_id)

    print('Streaming cube (joint-space) to robot: ', node.robot_id)

    try:
        node.start(robot=args.robot, size=args.size, speed=args.speed,
                   accel=args.accel, settle=args.settle, rate=args.rate,
                   use_velocities=not args.no_velocities, dry_run=args.dry_run)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()
