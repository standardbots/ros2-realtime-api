import time
import rclpy
import os
import sys
import argparse

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState

from standardbots import StandardBotsRobot, models

from detect_bot_id import detect_bot_id


class WriteJointState(Node):
    def __init__(self, robot_id: str | None = None):
        super().__init__("write_joint_state")

        if robot_id is None:
            self.robot_id = detect_bot_id()
        else:
            self.robot_id = robot_id

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        print(f"subscribing to joint state /{self.robot_id}/ro1/hardware/joint_state")

        self.subscription = self.create_subscription(
            JointState,
            f"/{self.robot_id}/ro1/hardware/joint_state",
            self.joint_state_callback,
            qos_profile,
        )

        self.publisher = self.create_publisher(
            JointState,
            f"/{self.robot_id}/ro1/hardware/joint_state/write",
            10,
        )

        self.start_joint_state = None

    def joint_state_callback(self, msg):
        if self.start_joint_state is None:
            self.start_joint_state = msg

    def start(self):
        print("Waiting for start joint state...")
        while not self.start_joint_state:
            rclpy.spin_once(self, timeout_sec=0.5)

        print(self.start_joint_state)

        print("Moving joints j0 and j2")

        side_steps = 100
        step = 0.00025  # radians per tick
        names = list(self.start_joint_state.name)
        start_positions = list(self.start_joint_state.position)

        # Triangle wave: up then back down on j0 and j2 simultaneously
        deltas = list(range(side_steps)) + list(range(side_steps, -1, -1))

        for cycle in range(10):
            for i in deltas:
                positions = list(start_positions)
                positions[0] = start_positions[0] + i * step
                positions[2] = start_positions[2] + i * step

                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = names
                joint_state.position = positions

                self.publisher.publish(joint_state)

                print(f"Cycle {cycle} step {i}")
                rclpy.spin_once(self, timeout_sec=0.1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Write joint states to a robot")
    parser.add_argument("--bot-id", dest="bot_id", type=str, help="Robot ID to write joint states to")
    parser.add_argument("--url", dest="robot_url", type=str, help="Robot URL to write joint states to")
    parser.add_argument("--token", dest="robot_token", type=str, help="Robot api token to write joint states to")
    args = parser.parse_args()

    if args.robot_url is None:
        args.robot_url = os.getenv("ROBOT_URL")
    if args.robot_token is None:
        args.robot_token = os.getenv("ROBOT_TOKEN")

    if args.robot_url is None or args.robot_token is None:
        print("Error: Robot URL and token are required")
        sys.exit(1)

    rclpy.init()

    if args.bot_id is None:
        args.bot_id = detect_bot_id()

    print("Connecting to robot: ", args.bot_id)

    sdk = StandardBotsRobot(
        url=args.robot_url,
        token=args.robot_token,
        robot_kind=StandardBotsRobot.RobotKind.Live,
    )

    with sdk.connection():
        print("Unbraking robot")
        sdk.movement.brakes.unbrake().ok()
        print("Setting control state to API")

        sdk.ros.control.update_ros_control_state(
            models.ROSControlUpdateRequest(
                action=models.ROSControlStateEnum.Enabled,
            )
        )

        state = sdk.ros.status.get_ros_control_state().ok()
        time.sleep(0.5)
        print("Control state set to API")

    write_joint_state_node = WriteJointState(robot_id=args.bot_id)

    try:
        print("Spinning...")
        write_joint_state_node.start()
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

            print("Control state set to disabled")
