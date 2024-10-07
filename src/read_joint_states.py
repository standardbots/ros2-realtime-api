import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState

from get_bot_id import get_bot_id

class ReadJointState(Node):
    def __init__(self, robot_id: str | None = None):
        super().__init__(f"read_joint_state")

        if robot_id is None:
            self.robot_id = get_bot_id(self)

        self.joint_positions = None
        self.subscription = self.create_subscription(
            JointState,
            f"/{self.robot_id}/ro1/hardware/joint_state",
            self.joint_state_callback,
            10
        )


    def joint_state_callback(self, msg):
        print(msg.position)

    def get_joint_positions(self):
        return self.joint_positions

def main():
    rclpy.init()
    
    read_joint_state_node = ReadJointState()

    try:
        rclpy.spin(read_joint_state_node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()

if __name__ == "__main__":
    main()