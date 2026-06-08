import time
import rclpy
import argparse
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

from detect_bot_id import detect_bot_id

class StreamJoints(Node):
    """Joint streaming over the external-control bridge: rotate J4 & J5 +0.1 rad and back."""

    def __init__(self, robot_id: str | None = None):
        super().__init__(f"stream_joints")

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

    def start(self):
        print("Waiting for start joint state...")
        deadline = time.monotonic() + 10
        while self.start_positions is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.start_positions is None:
            print("No robot_joints received - is the bridge engaged?")
            return

        print(self.start_positions)

        print("Rotating J4 & J5 +0.1 rad and back")
        for leg in (1, -1):
            for i in range(1, 91):
                f = (i if leg > 0 else 90 - i) / 90.0

                positions = list(self.start_positions)
                for j in (4, 5):
                    if j < len(positions):
                        positions[j] = self.start_positions[j] + 0.1 * f

                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = self.joint_names
                joint_state.position = positions

                self.publisher.publish(joint_state)
                rclpy.spin_once(self, timeout_sec=0)
                time.sleep(1 / 30)

            time.sleep(1)

        print("Done")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Stream joint positions to a robot over the external-control bridge')
    parser.add_argument('--bot-id', dest='bot_id', type=str, help='Robot ID to stream joints to')
    args = parser.parse_args()

    rclpy.init()

    if args.bot_id is None:
        args.bot_id = detect_bot_id()

    stream_joints_node = StreamJoints(robot_id=args.bot_id)

    print('Streaming joints to robot: ', stream_joints_node.robot_id)

    try:
        print("Spinning...")
        stream_joints_node.start()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()
