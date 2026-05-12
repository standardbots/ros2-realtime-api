import time
import rclpy
import json
import os
import sys
import argparse

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped

from standardbots import StandardBotsRobot, models

from detect_bot_id import detect_bot_id


class ReplayPoses(Node):
    def __init__(self, robot_id: str | None = None):
        super().__init__("replay_poses")

        if robot_id is None:
            self.robot_id = detect_bot_id()
        else:
            self.robot_id = robot_id

        self.publisher = self.create_publisher(
            PoseStamped,
            f"/{self.robot_id}/ro1/hardware/pose/write",
            10
        )

    def start(self, log_paths: list[str], rate_hz: float = 200.0):
        period = 1.0 / rate_hz
        next_t = time.monotonic()

        for log_path in log_paths:
            print(f"Reading poses from {log_path}")
            with open(log_path, "r") as f:
                poses = [json.loads(line) for line in f if line.strip()]

            print(f"Loaded {len(poses)} poses from {log_path}, streaming at {rate_hz} Hz")

            for idx, p in enumerate(poses):
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "world"
                pose_stamped.header.stamp = self.get_clock().now().to_msg()

                pose_stamped.pose.position.x = p["x"]
                pose_stamped.pose.position.y = p["y"]
                pose_stamped.pose.position.z = p["z"]
                pose_stamped.pose.orientation.w = p["w"]
                pose_stamped.pose.orientation.x = p["i"]
                pose_stamped.pose.orientation.y = p["j"]
                pose_stamped.pose.orientation.z = p["k"]

                self.publisher.publish(pose_stamped)

                if idx % 50 == 0:
                    print(f"[{log_path}] Published pose {idx}/{len(poses)}")

                next_t += period
                sleep_for = next_t - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_t = time.monotonic()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Replay poses from a log file to a robot')
    parser.add_argument('log_paths', type=str, nargs='+',
                        help='Paths to pose log files (one JSON object per line); replayed sequentially')
    parser.add_argument('--bot-id', dest='bot_id', type=str, help='Robot ID to write poses to')
    parser.add_argument('--url', dest='robot_url', type=str, help='Robot URL to write poses to')
    parser.add_argument('--token', dest='robot_token', type=str, help='Robot api token to write poses to')
    parser.add_argument('--rate', dest='rate_hz', type=float, default=50.0,
                        help='Publish rate in Hz')
    args = parser.parse_args()

    if args.robot_url is None:
        args.robot_url = os.getenv('ROBOT_URL')
    if args.robot_token is None:
        args.robot_token = os.getenv('ROBOT_TOKEN')

    if args.robot_url is None or args.robot_token is None:
        print("Error: Robot URL and token are required")
        sys.exit(1)

    rclpy.init()

    if args.bot_id is None:
        args.bot_id = detect_bot_id()

    print('Connecting to robot: ', args.bot_id)

    sdk = StandardBotsRobot(
        url=args.robot_url,
        token=args.robot_token,
        robot_kind=StandardBotsRobot.RobotKind.Live,
    )

    with sdk.connection():
        print('Unbraking robot')
        sdk.movement.brakes.unbrake().ok()
        print('Setting control state to API')

        sdk.ros.control.update_ros_control_state(
            models.ROSControlUpdateRequest(
                action=models.ROSControlStateEnum.Enabled,
            )
        )

        state = sdk.ros.status.get_ros_control_state().ok()
        time.sleep(0.5)
        print('Control state set to API')

    replay_node = ReplayPoses(robot_id=args.bot_id)

    try:
        print("Streaming...")
        replay_node.start(log_paths=args.log_paths, rate_hz=args.rate_hz)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()

        with sdk.connection():
            sdk.ros.control.update_ros_control_state(
                models.ROSControlUpdateRequest(
                    action=models.ROSControlStateEnum.Disabled,
                )
            )

            print('Control state set to disabled')
