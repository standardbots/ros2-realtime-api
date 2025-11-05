# ROS Realtime API
Sample code to call the ROS2 APIs.  By default we use cyclone dds implementation for the ROS2 networking.  Additionally, we use ROS_DOMAIN_ID=1 to expose the topics

## Setup

1. Enable the ROS2 API and ROS2 Bridge features from the Standard Bots app (Settings -> Configure Developer API)
   - ROS2 API enables control of the robot via ROS
   - ROS2 bridge publishes ROS2 topics to the local network. This may take up to 60 seconds.
   - If "Configure Developer API" is missing, reach out to Standard Bots and we can enable it for you.
   - ![image](https://github.com/user-attachments/assets/9d84639b-19e9-4fdf-a0d2-6564e592f56b)


## Building

1. Update the `cyclonedds.xml` `<NetworkInterface>` name to the adapter you will be using
1. Build the docker image

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
./run.sh python3 ./src/detect_bot_id.py
```

```
./run.sh python3 ./src/read_joint_states.py
```

## Run a shell in docker environement

You can run an interactive shell in the docker container.  It will automatically set the correct permissions and set the domain id, as well as mount the current directory into the container

1. `./run_shell.sh`

### Run a script
1. `python3 ./src/read_joint_states.py --bot-id=<ROBOT_ID>`

### List the ROS2 topics
1. `ros2 topic list`

