from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState, Imu
from builtin_interfaces.msg import Duration

from standardbots import StandardBotsRobot, models

import rclpy
import traceback
import csv
import sys
from datetime import datetime
from typing import List
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
        self.timer = self.create_timer(Ts, self.timer_callback)

        self.get_logger().info("Joint Publisher has Started.")

    def timer_callback(self):
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
        else:
            # Once the trajectory is complete, cancel the timer
            self.timer.cancel()
            self.get_logger().info("Trajectory publishing complete")

def create_plot(data: List, title: str, ylabel: str, xlabel: str):
    fig, ax = plt.subplots(1,1)

    # Multiple lines
    for j in range(0,len(data),2):
        ax.plot(data[j], data[j+1])
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
            sublist = flattened_data[index:index+length]
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

        with sdk.connection():
            ## Set teleoperation/ROS control state
            sdk.ros.control.update_ros_control_state(
                models.ROSControlUpdateRequest(
                    action=models.ROSControlStateEnum.Enabled,
                    # to disable: action=models.ROSControlStateEnum.Disabled,
                )
            )

            # Get teleoperation state
            state = sdk.ros.status.get_ros_control_state().ok()

        print(f"robot ROS status: {state}")
        # Enable the robot, make sure the E-stop is released before enabling
        print("Enabling robot...")
        # Unbrake the robot if not operational
        sdk.movement.brakes.unbrake().ok()

        position = sdk.movement.position.get_arm_position().ok()
        print('Initial Position: ', position.tooltip_position)
        print('Initial Joints: ', position.joint_rotations)

    except Exception as e:
        # Print exception error message
        print(str(e))
        raise RuntimeError(f"Error getting the robot operational: {str(e)}")

    bot_id = "bot_0sapi_zq17Nir8hAy" # *** ADD YOUR BOT ID HERE ***
    Ts = 0.005

    # Load position data
    time_data = read_list_data_from_csv(f"../data/2025-3-18/time.csv")
    position_data = read_list_data_from_csv(f"../data/2025-3-18/joint_positions.csv") # N x 6 list of joint positions [[j0, j1, j2, j3, j4, j5], ...]
    velocity_data = []
    acceleration_data = []

    # Add current robot position to position_data to account for the initial position
    current_joint_positions = list(position.joint_rotations)
    shifted_position_data = [[x + y for x, y in zip(current_joint_positions, position_data[i])]
                             for i in range(len(position_data))]

    # ---- Plot trajectory ----
    # data = [time_data, shifted_position_data]
    # subtitles = ["Trajectory"]
    # ylabels = ["Position [rad]"]
    # xlabel = "Time [s]"
    # fig9, _ = create_plot(data=data, title=subtitles[0],
    #                         ylabel=ylabels[0], xlabel=xlabel)
    # fig9.set_label("Trajectory")

    # plt.show()
    # ------------------------

    rclpy.init()

    joint_publisher = JointTrajectoryPublisher(
        bot_id,
        Ts,
        time_data,
        shifted_position_data,
        velocity_data,
        acceleration_data
    )

    rclpy.spin(joint_publisher)

    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except:
        traceback.print_exc()
