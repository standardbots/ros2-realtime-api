import time
import rclpy
import argparse
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from detect_bot_id import detect_bot_id

class StreamGripper(Node):
    """Gripper streaming over the external-control bridge: open -> closed -> open."""

    def __init__(self, robot_id: str | None = None):
        super().__init__(f"stream_gripper")

        if robot_id is None:
            self.robot_id = detect_bot_id(self)
        else:
            self.robot_id = robot_id

        # The external-control bridge publishes/subscribes under /<robot_id>/external
        base = f"/{self.robot_id}/external"

        self.publisher = self.create_publisher(
            Float64MultiArray,
            f"{base}/gripper",
            10
        )

    def start(self):
        # data is [position 0..1, velocity 0..1]
        for label, position in (("open", 1.0), ("closed", 0.0), ("open", 1.0)):
            print(f"gripper -> {label}")
            self.publisher.publish(Float64MultiArray(data=[position, 0.3]))

            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)

        print("Done")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Stream gripper commands to a robot over the external-control bridge')
    parser.add_argument('--bot-id', dest='bot_id', type=str, help='Robot ID to stream gripper commands to')
    args = parser.parse_args()

    rclpy.init()

    if args.bot_id is None:
        args.bot_id = detect_bot_id()

    stream_gripper_node = StreamGripper(robot_id=args.bot_id)

    print('Streaming gripper commands to robot: ', stream_gripper_node.robot_id)

    try:
        print("Spinning...")
        stream_gripper_node.start()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()
