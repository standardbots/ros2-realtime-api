import time
import rclpy
import json
import sys
import argparse
import time

from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped

from standardbots import StandardBotsRobot, models

from detect_bot_id import detect_bot_id
from read_joint_states import ReadJointState

class WritePose(Node):
    def __init__(self, poses, robot_id: str | None = None):
        super().__init__(f"write_joint_state")

        if robot_id is None:
            self.robot_id = detect_bot_id()
        else:
            self.robot_id = robot_id

        self.publisher = self.create_publisher(
            PoseStamped,
            f"/{self.robot_id}/ro1/hardware/pose/write",
            10
        )

        self.poses = poses

        self.start()

    def start(self):
        for i in range(10):
            raw_pose = self.poses[0]

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "world"

            pose_stamped.pose.position.x = raw_pose['x']
            pose_stamped.pose.position.y = raw_pose['y']
            pose_stamped.pose.position.z = raw_pose['z']
            pose_stamped.pose.orientation.w = raw_pose['w']
            pose_stamped.pose.orientation.x = raw_pose['i']
            pose_stamped.pose.orientation.y = raw_pose['j']
            pose_stamped.pose.orientation.z = raw_pose['k']

            self.publisher.publish(pose_stamped)

            time.sleep(0.02)
        
        for i, raw_pose in enumerate(self.poses):
            print(raw_pose)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "world"

            pose_stamped.pose.position.x = raw_pose['x']
            pose_stamped.pose.position.y = raw_pose['y']
            pose_stamped.pose.position.z = raw_pose['z']
            pose_stamped.pose.orientation.w = raw_pose['w']
            pose_stamped.pose.orientation.x = raw_pose['i']
            pose_stamped.pose.orientation.y = raw_pose['j']
            pose_stamped.pose.orientation.z = raw_pose['k']

            self.publisher.publish(pose_stamped)

            time.sleep(0.2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Write poses to a robot')
    parser.add_argument('file', type=str, help='Path to the pose file')
    parser.add_argument('--bot-id', type=str, help='Robot ID to write poses to')
    args = parser.parse_args()

    rclpy.init()

    if args.bot_id is None:
        args.bot_id = detect_bot_id()

    # read_joint_state_node = ReadJointState(robot_id=args.bot_id)

    # print('Reading joint states from robot: ', read_joint_state_node.robot_id)


    # rclpy.spin(node)

    # sys.exit(0)

    print('Connecting to robot: ', args.bot_id)

    sdk = StandardBotsRobot(
        url='http://192.168.0.102:3000',
        token='zmokpsd-0wzz4il0-gw6xw-tvyo3gc',
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

    print('Writing poses to robot: ', args.bot_id)

    poses = []
    with open(args.file) as fin:
        for line in fin:
            poses.append(json.loads(line))

    write_joint_state_node = WritePose(poses=poses, robot_id=args.bot_id)

    try:
        rclpy.spin(write_joint_state_node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()
