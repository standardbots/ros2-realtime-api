import time
import rclpy
import argparse
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from detect_bot_id import detect_bot_id

class StreamJointTrajectory(Node):
    """Joint-trajectory streaming over the external-control bridge: J4 & J5 +0.1 rad via a timed trajectory."""

    def __init__(self, robot_id: str | None = None):
        super().__init__(f"stream_joint_trajectory")

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
            JointTrajectory,
            f"{base}/trajectory",
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

    def make_point(self, positions, time_from_start):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(time_from_start),
            nanosec=int((time_from_start % 1) * 1e9)
        )
        return point

    def start(self):
        print("Waiting for start joint state...")
        deadline = time.monotonic() + 10
        while self.start_positions is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.start_positions is None:
            print("No robot_joints received - is the bridge engaged?")
            return

        print(self.start_positions)

        target = list(self.start_positions)
        for j in (4, 5):
            if j < len(target):
                target[j] = self.start_positions[j] + 0.1

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points = [
            self.make_point(target, 2.0),
            self.make_point(list(self.start_positions), 4.0),
        ]

        self.publisher.publish(trajectory)
        print("Trajectory sent (J4 & J5 +0.1 rad over 2s, back at 4s)")

        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        print("Done")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Stream a joint trajectory to a robot over the external-control bridge')
    parser.add_argument('--bot-id', dest='bot_id', type=str, help='Robot ID to stream the trajectory to')
    args = parser.parse_args()

    rclpy.init()

    if args.bot_id is None:
        args.bot_id = detect_bot_id()

    stream_trajectory_node = StreamJointTrajectory(robot_id=args.bot_id)

    print('Streaming joint trajectory to robot: ', stream_trajectory_node.robot_id)

    try:
        print("Spinning...")
        stream_trajectory_node.start()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        print("Cleaning up...")
        rclpy.shutdown()
