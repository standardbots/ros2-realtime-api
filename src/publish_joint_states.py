from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from standardbots import StandardBotsRobot, models

import rclpy
import traceback
import csv
import sys
import time

from typing import List, Optional
from matplotlib import pyplot as plt


class JointTrajectoryPublisher(Node):
    def __init__(
            self,
            bot_id: str,
            Ts: float = 0.005,
            time_data: list = [],
            position_data: list = [],
            velocity_data: list = [],
            acceleration_data: list = [],
    ):
        super().__init__("joint_trajectory_publisher")
        self.bot_id = bot_id

        self.time_data = time_data
        self.position_data = position_data
        self.velocity_data = velocity_data
        self.acceleration_data = acceleration_data

        self.topic = f"/{bot_id}/ro1/hardware/joint_trajectory"
        self.joint_publisher = self.create_publisher(
            JointTrajectory,
            self.topic,
            10
        )
        self.index = 0
        self.interval = Ts
        self.timer = self.create_timer(Ts, self.timer_callback)
        self.is_publishing = False

        self.get_logger().info("Joint Publisher has Started.")

    def send_positions(
            self,
            positions: List[List[float]],
            times: Optional[List[float]] = None,
            velocities: Optional[List[List[float]]] = None,
            accelerations: Optional[List[List[float]]] = None
    ):
        """
        Update the trajectory data and start publishing.

        Args:
            positions: List of joint position arrays
            times: List of timestamps for each position
            velocities: Optional list of joint velocity arrays
            accelerations: Optional list of joint acceleration arrays
        """
        self.position_data = positions

        if times is not None:
            self.time_data = times
        elif not self.time_data or len(self.time_data) != len(positions):
            # Create default time data if not provided
            self.time_data = [i * self.interval for i in range(len(positions))]

        if velocities is not None:
            self.velocity_data = velocities

        if accelerations is not None:
            self.acceleration_data = accelerations

        # Reset index to start from beginning
        self.index = 0
        self.is_publishing = True

        # Restart timer if it was cancelled
        if not self.timer.is_ready():
            self.timer = self.create_timer(self.interval, self.timer_callback)

        self.get_logger().info(f"Starting to publish {len(positions)} trajectory points")

    def timer_callback(self):
        if not self.is_publishing or self.index >= len(self.position_data):
            # If we've finished publishing or aren't supposed to publish, stop
            self.is_publishing = False
            self.timer.cancel()
            self.get_logger().info("Trajectory publishing complete")
            return

        if self.index < len(self.position_data):
            point = JointTrajectoryPoint()
            point.positions = self.position_data[self.index]

            print(self.position_data[self.index])

            point.velocities = self.velocity_data[self.index] if len(self.velocity_data) > 0 else []
            point.accelerations = self.acceleration_data[self.index] if len(self.acceleration_data) > 0 else []

            # Use provided time_data to set the time_from_start field [seconds]
            t = self.time_data[self.index]
            # point.time_from_start.sec = int(t)
            # point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            point.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))

            msg = JointTrajectory()
            msg.joint_names = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5"]
            msg.points.append(point)
            self.joint_publisher.publish(msg)
            self.get_logger().info(
                f"Published point {self.index} with time: {point.time_from_start.sec}.{point.time_from_start.nanosec}"
            )
            self.index += 1


def create_plot(data: List, title: str, ylabel: str, xlabel: str):
    fig, ax = plt.subplots(1, 1)

    # Multiple lines
    for j in range(0, len(data), 2):
        ax.plot(data[j], data[j + 1])
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.set_xlabel(xlabel)

    return fig, ax


def read_list_data_from_csv(filename) -> List:
    def convert_to_number(item):
        """Convert string to integer or float as appropriate."""
        try:
            return int(item)
        except ValueError:
            try:
                return float(item)
            except ValueError:
                return item

    csv.field_size_limit(sys.maxsize)
    with open(filename, 'r', newline='') as file:
        reader = csv.reader(file)
        rows = list(reader)

    if not rows or len(rows) < 3:
        raise ValueError("CSV file does not have the expected format.")

    # Row 0: Read num_dimensions
    num_dimensions = int(rows[0][0])

    # Row 2 always contains the flattened data
    flattened_data = [convert_to_number(item) for item in rows[2]]

    # Case 1: Regular list (1D)
    if num_dimensions == 1:
        # Row 1: Contains a single cell with the outer list length, e.g., "[6]"
        structure_str = rows[1][0].strip()
        # Remove the surrounding brackets and split on comma-space
        outer_vals = [int(x) for x in structure_str.strip('[]').split(', ') if x]
        # The outer_vals list should contain one number representing the length
        if outer_vals and outer_vals[0] != len(flattened_data):
            raise ValueError("Mismatch between structure length and flattened data length.")
        return flattened_data

    # Case 2: Nested list (list of lists)
    else:
        # Row 1: Should have two cells:
        #   - First cell: outer list length, e.g., "[5]"
        #   - Second cell: list of inner list lengths, e.g., "[3, 3, 3, 3, 3]"
        outer_str = rows[1][0].strip()
        inner_str = rows[1][1].strip()

        outer_vals = [int(x) for x in outer_str.strip('[]').split(', ') if x]
        inner_vals = [int(x) for x in inner_str.strip('[]').split(', ') if x]

        if not outer_vals:
            raise ValueError("Missing outer structure information.")
        outer_count = outer_vals[0]
        if outer_count != len(inner_vals):
            raise ValueError("Mismatch between the declared outer count and inner list lengths.")

        result = []
        index = 0
        for length in inner_vals:
            sublist = flattened_data[index:index + length]
            if len(sublist) != length:
                raise ValueError("Flattened data does not match the expected inner list length.")
            result.append(sublist)
            index += length
        if index != len(flattened_data):
            raise ValueError("Extra data found in flattened data row.")
        return result


def main():
    # EDIT CODE TO CONNECT TO THE ROBOT
    try:
        # Instantiate robot interface
        sdk = StandardBotsRobot(
            url='http://localhost:3000',
            token='8330nv-cf504-g9t85cp8-0zxbmg',
            robot_kind=StandardBotsRobot.RobotKind.Live,
        )

        print("Connecting to robot...")

        with sdk.connection():
            # Set teleoperation/ROS control state
            sdk.ros.control.update_ros_control_state(
                models.ROSControlUpdateRequest(
                    action=models.ROSControlStateEnum.Enabled,
                    # to disable: action=models.ROSControlStateEnum.Disabled,
                )
            )

            # Get teleoperation state
            state = sdk.ros.status.get_ros_control_state().ok()

        print("robot ROS status: {}".format(state))
        # Enable the robot, make sure the E-stop is released before enabling
        print("Enabling robot...")
        # Unbrake the robot if not operational
        sdk.movement.brakes.unbrake().ok()

        # Load original trajectory data from files
        time_data = read_list_data_from_csv("../data/2025-3-18/time.csv")
        position_data = read_list_data_from_csv("../data/2025-3-18/joint_positions.csv")  # N x 6 list of joint positions [[j0, j1, j2, j3, j4, j5], ...]
        velocity_data: List[List[float]] = []
        acceleration_data: List[List[float]] = []

        bot_id = "bot_0sapi_zq17Nir8hAy"  # ADD YOUR BOT ID HERE
        Ts = 0.005

        # Initialize ROS
        rclpy.init()

        # Create publisher node without initial positions
        joint_publisher = JointTrajectoryPublisher(
            bot_id,
            Ts
        )

        # Create a separate thread for processing callbacks
        import threading

        def spin_thread():
            rclpy.spin(joint_publisher)

        ros_thread = threading.Thread(target=spin_thread, daemon=True)
        ros_thread.start()

        # ---- First execution ----
        # Get current robot position before first trajectory
        position1 = sdk.movement.position.get_arm_position().ok()
        print('Initial Position (run 1): ', position1.tooltip_position)
        print('Initial Joints (run 1): ', position1.joint_rotations)

        # Add current robot position to position_data for first run
        current_joint_positions1 = list(position1.joint_rotations)
        shifted_position_data1 = [
            [x + y for x, y in zip(current_joint_positions1, position_data[i])]
            for i in range(len(position_data))
        ]

        print("Starting first trajectory execution...")
        # Then send the positions to publish (first time)
        joint_publisher.send_positions(
            positions=shifted_position_data1,
            times=time_data,
            velocities=velocity_data,
            accelerations=acceleration_data
        )

        # Wait for trajectory to complete
        while joint_publisher.is_publishing:
            import time
            time.sleep(0.1)

        print("First trajectory execution completed!")

        with sdk.connection():
            time.sleep(1.0)  # 1 second pause

            target_position = (1.07, 1.07, -1.07, 0, 0, 0)
            body = models.ArmPositionUpdateRequest(
                kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
                joint_rotation=models.ArmJointRotations(joints=target_position),
            )

            sdk.movement.position.set_arm_position(body=body)

            sdk.ros.control.update_ros_control_state(
                models.ROSControlUpdateRequest(
                    action=models.ROSControlStateEnum.Enabled,
                    # to disable: action=models.ROSControlStateEnum.Disabled,
                )
            )

            # Get teleoperation state
            state = sdk.ros.status.get_ros_control_state().ok()

        print("robot ROS status: {}".format(state))


        print("First trajectory execution completed!")

        # ---- Second execution ----
        # Get current robot position before second trajectory
        position2 = sdk.movement.position.get_arm_position().ok()
        print('Initial Position (run 2): ', position2.tooltip_position)
        print('Initial Joints (run 2): ', position2.joint_rotations)

        # Add current robot position to position_data for second run
        current_joint_positions2 = list(position2.joint_rotations)
        shifted_position_data2 = [
            [x + y for x, y in zip(current_joint_positions2, position_data[i])]
            for i in range(len(position_data))
        ]

        print("Starting second trajectory execution...")
        # Send the positions again with the new current position
        joint_publisher.send_positions(
            positions=shifted_position_data2,
            times=time_data,
            velocities=velocity_data,
            accelerations=acceleration_data
        )

        # Wait for trajectory to complete
        while joint_publisher.is_publishing:
            time.sleep(0.1)

        print("Second trajectory execution completed!")

        # Stop the ROS thread and cleanup
        rclpy.shutdown()
        joint_publisher.destroy_node()

    except Exception as e:
        # Print exception error message
        print(str(e))
        raise RuntimeError("Error getting the robot operational: {}".format(str(e)))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
