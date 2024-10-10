import time
import rclpy
import json
import sys

from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped

from get_bot_id import get_bot_id

class WritePose(Node):
    def __init__(self, poses, robot_id: str | None = None):
        super().__init__(f"write_joint_state")

        if robot_id is None:
            self.robot_id = get_bot_id(self)

        self.publisher = self.create_publisher(
            PoseStamped,
            f"/{self.robot_id}/ro1/hardware/pose/write", 
            10
        )

        self.poses = poses

        self.start()

    def start(self):
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

            time.sleep(0.02)       


if __name__ == "__main__":
    rclpy.init()
    
    poses = []
    with open(sys.argv[1]) as fin:
        for line in fin:
            poses.append(json.loads(line))

    write_joint_state_node = WritePose(poses=poses)

    try:
        rclpy.spin(write_joint_state_node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()
