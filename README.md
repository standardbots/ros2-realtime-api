# ROS Realtime API
Sample code to call the ros2 apis

## Setup

1. Enable the ROS2 API and ROS2 Bridge features from the Standard Bots app (Settings -> Configure Developer API)
   - ROS2 API enables control of the robot via ROS
   - ROS2 bridge publishes ROS2 topics to the local network. This may take up to 60 seconds.
   - If "Configure Developer API" is missing, reach out to Standard Bots and we can enable it for you.
   - ![image](https://github.com/user-attachments/assets/9d84639b-19e9-4fdf-a0d2-6564e592f56b)
2. Update the `cyclonedds.xml` NetworkInterface name to the adapter you will be using
3. Build
4. Run
5. Confirm the following ROS2 topics are visible `ros2 topic list`
```
/{BOT_ID}/ro1/can/send
/{BOT_ID}/ro1/hardware/end_effector_imu
/{BOT_ID}/ro1/hardware/jacobian
/{BOT_ID}/ro1/hardware/joint_state
/{BOT_ID}/ro1/hardware/joint_trajectory
/{BOT_ID}/ro1/hardware/pose
/{BOT_ID}/ro1/hardware/pose/write
/{BOT_ID}/ro1/hardware/scaling_factor
/{BOT_ID}/ro1/hardware/tooltip_velocity
/{BOT_ID}/ro1/modbus/receive
/{BOT_ID}/ro1/modbus/send
```

## Building

```
./build.sh
```

## Running

### Run the default script

```
./run.sh
```

### Run a different script

```
./run.sh python3 ./src/write_poses.py --bot-id=<ROBOT_ID> ./data/pose1.log"
```

## Run in docker environement

1. `./run_shell.sh`
1. `python3 ./src/read_joint_states.py --bot-id=<ROBOT_ID>`

