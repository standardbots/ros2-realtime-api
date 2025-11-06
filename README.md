# ROS Realtime API
Sample code to call the ROS2 APIs.  By default we use cyclone dds implementation for the ROS2 networking.  Additionally, we use ROS_DOMAIN_ID=1 to expose the topics

## Setup

1. Clear any errors on the move page 
1. Enable the ROS2 API and ROS2 Bridge features from the Standard Bots app 
   - Click menu -> Settings -> Configure Developer API
   - If "Configure Developer API" is missing, reach out to Standard Bots and we can enable it for you.
   - Turn the `Enable Developer API` on.  This enables the python / REST API
   - Turn the `Enable ROS2 bridge` on.  This enables the ROS2 topics to be published on the local network
   - ![image](https://github.com/user-attachments/assets/9d84639b-19e9-4fdf-a0d2-6564e592f56b)


## Building

1. Update the `cyclonedds.xml` `<NetworkInterface>` `name` attribute to the adapter you will be using.  This adapter should be on the same network that the control box is on
  - For example if your adapter is named `eth0` change the line to this`<NetworkInterface name="eth0" priority="0" presence_required="true" />`
1. Build the docker image

```
./build.sh
```

## Running

### Run the default script

```
./run.sh
```

The default script will auto detect the bot id and then just read and print out the default joint states

### Run a different script

#### Print out the auto detected bot id

```
./run.sh python3 ./src/detect_bot_id.py
```

#### Read the joint states

```
./run.sh python3 ./src/read_joint_states.py
```

#### Move the robot down from current position
```
./run.sh python3 ./src/write_poses.py
```

## Run a shell in docker environement

You can also run an interactive shell in the docker container.  It will automatically set the correct permissions and set the domain id, as well as mount the current directory into the container.  You can use this iteratively develop a script while running an set up environment

1. `./run_shell.sh`

### List the ROS2 topics
1. `ros2 topic list`

### Run a script
1. `python3 ./src/read_joint_states.py --bot-id=<ROBOT_ID>`


# Available ROS2 API / Topics

- `/<BOT_ID>/ro1/hardware/end_effector_imu`
  - end effector imu data
  - sensor_msgs/msg/Imu
- `/<BOT_ID>/ro1/hardware/jacobian`
  - std_msgs/msg/Float64MultiArray
- `/<BOT_ID>/ro1/hardware/joint_state`
  - sensor_msgs/msg/JointState
- `/<BOT_ID>/ro1/hardware/joint_trajectory`
  - trajectory_msgs/msg/JointTrajectory
- `/<BOT_ID>/ro1/hardware/pose`
  - geometry_msgs/msg/PoseStamped
- `/<BOT_ID>/ro1/hardware/pose/write`
  - geometry_msgs/msg/PoseStamped
