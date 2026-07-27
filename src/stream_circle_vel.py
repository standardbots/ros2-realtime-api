import time
import rclpy
import argparse
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

from detect_bot_id import detect_bot_id
from stream_cube_vel import load_chain, solve_joint_trajectory

# Unit axis pairs (u, w) spanning each supported circle plane. The circle's
# center sits +radius along u from the current tooltip, so the whole trace
# extends in +u; the tooltip itself is the point of the circle nearest the
# start pose (start == end).
PLANE_AXES = {
    "xy": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    "xz": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    "yz": ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
}
PLANE_CENTER_AXIS = {"xy": "X", "xz": "X", "yz": "Y"}


def trapezoid_profile(distance, speed, accel, rate):
    """Arc length covered at each tick (`rate` Hz) for a single trapezoidal
    speed profile over `distance` meters — the same ramp math
    stream_cube_vel.cube_cartesian_path applies per cube edge, but spanning
    one continuous run (a circle has no corners to settle at)."""
    dt = 1.0 / rate
    v, a = speed, accel
    ramp = v * v / (2 * a)  # distance to go 0 -> v (and v -> 0)

    if 2 * ramp <= distance:
        # Trapezoid: accelerate, cruise, decelerate.
        t_acc = v / a
        t_cruise = (distance - 2 * ramp) / v
        v_peak = v
    else:
        # Triangle: arc too short to reach cruise speed.
        v_peak = (a * distance) ** 0.5
        t_acc = v_peak / a
        t_cruise = 0.0
        ramp = distance / 2

    total_time = 2 * t_acc + t_cruise
    steps = max(1, int(round(total_time * rate)))

    s = np.empty(steps)
    for i in range(1, steps + 1):
        t = i * dt
        if t < t_acc:                        # accelerating
            si = 0.5 * a * t * t
        elif t < t_acc + t_cruise:           # cruising
            si = ramp + v_peak * (t - t_acc)
        else:                                # decelerating
            td = t - t_acc - t_cruise
            si = ramp + v_peak * t_cruise + v_peak * td - 0.5 * a * td * td
        s[i - 1] = min(si, distance)
    # Rounding `steps` can leave the last sample a hair short of the full
    # distance; pin it so the trace ends exactly where it started.
    s[-1] = distance
    return s


def circle_cartesian_path(origin, radius, speed, accel, settle, laps, plane, rate):
    """Tooltip positions for `laps` full circles, one per tick at `rate` Hz.

    The circle lies in `plane`, passes through the current tooltip (the trace
    starts and ends there), and its center sits +radius along the plane's
    first axis. Tangential speed follows one trapezoidal profile over the
    whole arc (ramp at `accel`, cruise at `speed`), then the end point is held
    for `settle` seconds so the arm comes to rest.
    """
    u, w = (np.asarray(axis) for axis in PLANE_AXES[plane])
    origin = np.asarray(origin)
    center = origin + radius * u

    s = trapezoid_profile(2 * np.pi * radius * laps, speed, accel, rate)
    theta = np.pi + s / radius  # theta = pi is the tooltip itself
    points = center + radius * (np.outer(np.cos(theta), u) + np.outer(np.sin(theta), w))

    points = np.vstack([origin, points])
    points = np.vstack([points, np.repeat(points[-1:], max(1, int(settle * rate)), axis=0)])
    return points


class StreamCircleVel(Node):
    """Joint streaming over the external-control bridge: trace a circle with
    the tooltip via client-side IK, streaming joint positions AND velocities.

    Unlike the cube (straight edges, corner stops), the circle is a single
    smooth constant-curvature path with no rest points — a cleaner probe of
    steady-state tracking lag, since the error never gets to reset at a
    corner."""

    def __init__(self, robot_id: str | None = None):
        super().__init__("stream_circle_vel")

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
              "velocities are streamed and jointPositionVelocityMode is on)")

    def start(self, robot, radius, speed, accel, settle, laps, plane, rate,
              use_velocities, zero_velocities, dry_run):
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

        center_axis = PLANE_CENTER_AXIS[plane]
        print(f"Loading {robot}.urdf and precomputing IK "
              f"({radius * 100:.0f}cm-radius circle x{laps} in the {plane.upper()} "
              f"plane, {speed * 100:.0f}cm/s, {rate:.0f}Hz)...")
        chain = load_chain(robot)
        n_links = len(chain.links)
        full = np.zeros(n_links)
        full[1:7] = start_joints
        origin = chain.forward_kinematics(full)[:3, 3]
        print("Tooltip (FK):", np.round(origin, 4).tolist(),
              f"- circle center is +{center_axis} {radius * 100:.0f}cm from here")

        points = circle_cartesian_path(origin, radius, speed, accel, settle,
                                       laps, plane, rate)
        positions, velocities = solve_joint_trajectory(
            chain, start_joints, points, rate,
            shape_hint=f"the circle ({radius * 100:.0f}cm radius, "
                       f"+{center_axis} of the current pose in the "
                       f"{plane.upper()} plane)",
            size_flag="--radius",
        )
        duration = len(positions) / rate
        vel_mode = 'ON' if use_velocities else 'OFF (position-only)'
        if zero_velocities:
            vel_mode = 'ZEROED (degenerate: trusted mode with contradictory v=0)'
        print(f"Trajectory: {len(positions)} setpoints, {duration:.1f}s, "
              f"peak joint velocity {np.abs(velocities).max():.3f} rad/s, "
              f"velocities {vel_mode}")

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
            if zero_velocities:
                # Degenerate test mode: a FULL velocity vector (so the robot's
                # trusted-velocity path engages) whose content claims the arm
                # should be stationary at every point. Expect rough, steppy
                # motion — the smoother is bypassed AND there is no velocity
                # feedforward to bridge between points.
                msg.velocity = [0.0] * len(names)
            elif use_velocities:
                # The bridge forwards velocity only when it is one-per-joint;
                # the robot uses it only when jointPositionVelocityMode is on.
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
        description='Trace a circle with the tooltip by streaming joint positions '
                    '+ velocities (client-side IK) over the external-control bridge')
    parser.add_argument('--bot-id', dest='bot_id', type=str, help='Robot ID to stream joints to')
    parser.add_argument('--robot', dest='robot', type=str, default='thor',
                        choices=['core', 'spark', 'thor'],
                        help='URDF to use for FK/IK (default: thor)')
    parser.add_argument('--radius', dest='radius', type=float, default=0.05,
                        help='Circle radius in meters (default: 0.05)')
    parser.add_argument('--plane', dest='plane', type=str, default='xy',
                        choices=sorted(PLANE_AXES),
                        help='Plane the circle lies in (default: xy)')
    parser.add_argument('--laps', dest='laps', type=int, default=2,
                        help='Number of full circles to trace (default: 2)')
    parser.add_argument('--speed', dest='speed', type=float, default=0.05,
                        help='Cruise tangential speed in m/s (default: 0.05)')
    parser.add_argument('--accel', dest='accel', type=float, default=0.2,
                        help='Tangential acceleration in m/s^2 for the trapezoidal ramp (default: 0.2)')
    parser.add_argument('--settle', dest='settle', type=float, default=0.5,
                        help='Seconds to hold the end point (default: 0.5)')
    parser.add_argument('--rate', dest='rate', type=float, default=100.0,
                        help='Setpoint stream rate in Hz (default: 100)')
    parser.add_argument('--no-velocities', dest='no_velocities', action='store_true',
                        help='Stream positions only (A/B baseline: the robot smooths '
                             'the position stream and derives velocity itself)')
    parser.add_argument('--zero-velocities', dest='zero_velocities', action='store_true',
                        help='Stream a full velocity vector of ZEROS with moving positions '
                             '(degenerate test: engages the trusted-velocity path with '
                             'kinematically inconsistent input; expect steppy motion). '
                             'Keep --speed low.')
    parser.add_argument('--dry-run', dest='dry_run', action='store_true',
                        help='Precompute and validate the trajectory, do not stream')
    args = parser.parse_args()

    rclpy.init()

    if args.bot_id is None:
        args.bot_id = detect_bot_id()

    node = StreamCircleVel(robot_id=args.bot_id)

    print('Streaming circle (joint-space) to robot: ', node.robot_id)

    try:
        if args.zero_velocities and args.no_velocities:
            raise SystemExit("--zero-velocities and --no-velocities are mutually exclusive")
        if args.laps < 1:
            raise SystemExit("--laps must be >= 1")
        node.start(robot=args.robot, radius=args.radius, speed=args.speed,
                   accel=args.accel, settle=args.settle, laps=args.laps,
                   plane=args.plane, rate=args.rate,
                   use_velocities=not args.no_velocities,
                   zero_velocities=args.zero_velocities, dry_run=args.dry_run)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()
