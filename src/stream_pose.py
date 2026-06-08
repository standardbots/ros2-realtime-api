import time
import rclpy
import argparse
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped

from detect_bot_id import detect_bot_id

class StreamPose(Node):
    """Pose streaming over the external-control bridge: nudge the tooltip +Z 2cm and back."""

    def __init__(self, robot_id: str | None = None):
        super().__init__(f"stream_pose")

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

    def start(self):
        print("Waiting for start pose...")
        deadline = time.monotonic() + 10
        while self.start_pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.start_pose is None:
            print("No robot_pose received - is the bridge engaged?")
            return

        print(self.start_pose)

        print("Nudging tooltip +Z 2cm and back")
        for leg in (1, -1):
            for i in range(1, 91):
                f = (i if leg > 0 else 90 - i) / 90.0

                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "world"
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.pose.position.x = self.start_pose.position.x
                pose_stamped.pose.position.y = self.start_pose.position.y
                pose_stamped.pose.position.z = self.start_pose.position.z + 0.02 * f
                pose_stamped.pose.orientation = self.start_pose.orientation

                self.publisher.publish(pose_stamped)
                rclpy.spin_once(self, timeout_sec=0)
                time.sleep(1 / 30)

            time.sleep(1)

        print("Done")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Stream a pose nudge to a robot over the external-control bridge')
    parser.add_argument('--bot-id', dest='bot_id', type=str, help='Robot ID to stream poses to')
    args = parser.parse_args()

    rclpy.init()

    if args.bot_id is None:
        args.bot_id = detect_bot_id()

    stream_pose_node = StreamPose(robot_id=args.bot_id)

    print('Streaming poses to robot: ', stream_pose_node.robot_id)

    try:
        print("Spinning...")
        stream_pose_node.start()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()
