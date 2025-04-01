import rclpy
import argparse
from rclpy.node import Node
from sensor_msgs.msg import JointState

from get_bot_id import get_bot_id

class ReadJointState(Node):
    def __init__(self, robot_id: str | None = None):
        super().__init__(f"read_joint_state")

        if robot_id is None:
            self.robot_id = get_bot_id(self)
        else:
            self.robot_id = robot_id

        self.joint_positions = None
        self.subscription = self.create_subscription(
            JointState,
            f"/{self.robot_id}/ro1/hardware/joint_state",
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        print([val for val in msg.position])

    def get_joint_positions(self):
        return self.joint_positions

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Read joint states from a robot')
    parser.add_argument('--bot-id', type=str, help='Robot ID to read joint states from')
    args = parser.parse_args()

    rclpy.init()

    read_joint_state_node = ReadJointState(robot_id=args.bot_id)

    try:
        print("Spinning...")
        rclpy.spin(read_joint_state_node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()
