import time
import rclpy
import argparse
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped

from detect_bot_id import detect_bot_id

# Cube corners as (x, y, z) offsets in units of the cube side, ordered so that
# consecutive corners differ on a single axis (a Gray-code Hamiltonian cycle).
# Every move is therefore axis-aligned, all 8 corners are visited, and the path
# returns to the starting corner.
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

class StreamCube(Node):
    """Pose streaming over the external-control bridge: trace an axis-aligned cube with the tooltip."""

    def __init__(self, robot_id: str | None = None, size: float = 0.10, speed: float = 0.05):
        super().__init__(f"stream_cube")

        if robot_id is None:
            self.robot_id = detect_bot_id(self)
        else:
            self.robot_id = robot_id

        self.size = size
        self.speed = speed

        # The external-control bridge publishes/subscribes under /<robot_id>/external
        base = f"/{self.robot_id}/external"

        # Use BEST_EFFORT QoS to match the bridge publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            f"{base}/pose",
            10
        )

        self.start_pose = None

        self.subscription = self.create_subscription(
            PoseStamped,
            f"{base}/robot_pose",
            self.pose_callback,
            qos_profile
        )

    def pose_callback(self, msg):
        if self.start_pose is None:
            self.start_pose = msg.pose

    def make_pose(self, x, y, z):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation = self.start_pose.orientation
        return pose_stamped

    def move_to(self, target, rate=30.0):
        # Linearly interpolate from the last commanded corner to target at a
        # fixed Cartesian speed, publishing setpoints at `rate` Hz.
        start = self.current
        dx = target[0] - start[0]
        dy = target[1] - start[1]
        dz = target[2] - start[2]
        distance = (dx * dx + dy * dy + dz * dz) ** 0.5

        steps = max(1, round((distance / self.speed) * rate))
        for i in range(1, steps + 1):
            f = i / steps
            self.publisher.publish(self.make_pose(
                start[0] + dx * f,
                start[1] + dy * f,
                start[2] + dz * f,
            ))
            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(1 / rate)

        self.current = target

    def start(self):
        print("Waiting for start pose...")
        deadline = time.monotonic() + 10
        while self.start_pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.start_pose is None:
            print("No robot_pose received - is the bridge engaged?")
            return

        print(self.start_pose)

        # The current pose is corner (0, 0, 0); the cube extends +X/+Y/+Z by `size`.
        origin = (
            self.start_pose.position.x,
            self.start_pose.position.y,
            self.start_pose.position.z,
        )
        corners = [
            (origin[0] + bx * self.size,
             origin[1] + by * self.size,
             origin[2] + bz * self.size)
            for (bx, by, bz) in CUBE_CORNERS
        ]

        self.current = corners[0]

        print(f"Tracing a {self.size * 100:.0f}cm axis-aligned cube (+X/+Y/+Z from current pose) at {self.speed * 100:.0f}cm/s")
        for index, corner in enumerate(corners[1:], start=1):
            print(f"  edge {index}/{len(corners) - 1} -> ({corner[0]:.3f}, {corner[1]:.3f}, {corner[2]:.3f})")
            self.move_to(corner)
            time.sleep(0.2)

        print("Done")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Trace an axis-aligned cube with the tooltip over the external-control bridge')
    parser.add_argument('--bot-id', dest='bot_id', type=str, help='Robot ID to stream poses to')
    parser.add_argument('--size', dest='size', type=float, default=0.10, help='Cube side length in meters (default: 0.10)')
    parser.add_argument('--speed', dest='speed', type=float, default=0.05, help='Cartesian speed in m/s (default: 0.05)')
    args = parser.parse_args()

    rclpy.init()

    if args.bot_id is None:
        args.bot_id = detect_bot_id()

    stream_cube_node = StreamCube(robot_id=args.bot_id, size=args.size, speed=args.speed)

    print('Streaming cube to robot: ', stream_cube_node.robot_id)

    try:
        print("Spinning...")
        stream_cube_node.start()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()
