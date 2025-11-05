import time
import rclpy
import json
import sys
import argparse
import time

from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped

from standardbots import StandardBotsRobot, models

from detect_bot_id import detect_bot_id
from read_joint_states import ReadJointState

class WritePose(Node):
    def __init__(self, robot_id: str | None = None):
        super().__init__(f"write_pose")

        if robot_id is None:
            self.robot_id = detect_bot_id()
        else:
            self.robot_id = robot_id

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        print(f"subscribing to pose /{self.robot_id}/ro1/hardware/pose")

        self.subscription = self.create_subscription(
            PoseStamped,
            f"/{self.robot_id}/ro1/hardware/pose",
            self.pose_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            f"/{self.robot_id}/ro1/hardware/pose/write",
            10
        )

        self.start_pose = None

    def pose_callback(self, msg):
        if self.start_pose is None:
            self.start_pose = msg.pose


    def start(self):
        print('Waiting for start pose...')
        while not self.start_pose:
            rclpy.spin_once(self, timeout_sec=0.5)

        print(self.start_pose)

        print("Moving to end pose")
        for i in range(100):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "world"

            pose_stamped.pose.position.x = self.start_pose.position.x
            pose_stamped.pose.position.y = self.start_pose.position.y
            pose_stamped.pose.position.z = self.start_pose.position.z
            pose_stamped.pose.orientation.w = self.start_pose.orientation.w
            pose_stamped.pose.orientation.x = self.start_pose.orientation.x
            pose_stamped.pose.orientation.y = self.start_pose.orientation.y
            pose_stamped.pose.orientation.z = self.start_pose.orientation.z

            pose_stamped.pose.position.z = self.start_pose.position.z - i * 0.001

            self.publisher.publish(pose_stamped)

            print(f"Published pose {i}")
            rclpy.spin_once(self, timeout_sec=0.2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Write poses to a robot')
    parser.add_argument('--bot-id', type=str, help='Robot ID to write poses to')
    args = parser.parse_args()

    rclpy.init()

    if args.bot_id is None:
        args.bot_id = detect_bot_id()

    print('Connecting to robot: ', args.bot_id)

    sdk = StandardBotsRobot(
        url='<ROBOT_URL>',
        token='<ROBOT_TOKEN>',
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

        # Get teleoperation state
        print(dir(sdk.ros.control))
        state = sdk.ros.status.get_ros_control_state().ok()
        time.sleep(0.5)
        print('Control state set to API')

    write_pose_node = WritePose(robot_id=args.bot_id)

    try:
        print("Spinning...")
        write_pose_node.start()
        rclpy.spin(write_pose_node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()
