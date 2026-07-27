# ROS Realtime API
Sample code to call the ROS2 APIs.  By default we use cyclone dds implementation for the ROS2 networking.  Additionally, we use `ROS_DOMAIN_ID=1` to expose the topics

# Robot Setup

1. Clear any errors on the move page 
1. Enable the ROS2 API and ROS2 Bridge features from the Standard Bots app 
   - Click menu -> Settings -> Configure Developer API
   - If "Configure Developer API" is missing, reach out to Standard Bots and we can enable it for you.
   - Turn the `Enable Developer API` on.  This enables the python / REST API
   - Turn the `Enable ROS2 bridge` on.  This enables the ROS2 topics to be published on the local network
   - ![image](https://github.com/user-attachments/assets/9d84639b-19e9-4fdf-a0d2-6564e592f56b)


# Client Setup

Next, we set up the client side to talk to the robot.  First you'll need to setup a machine on the same network that the robot is currently on.  Ethernet is preferred, but wifi will work as well.

Next, we require docker to run the sample code.  We use docker to set up the ROS environment so that it can communication over the ROS2 protocols

## Pull repository

Download this repository on your machine.  

`git clone git@github.com:standardbots/ros2-realtime-api.git`

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

We need to explicitly use the SDK to unbrake and enable the ROS2, so we need the url and api token.
These can be obtained from the developer api menu, under menu -> settings -> configure developer api

```
./run.sh python3 ./src/write_poses.py --bot-id=<BOT_ID> --token=<TOKEN> --url=<REMOTE_ROBOT_URL>
```

#### Stream commands over the external-control bridge

These scripts drive the robot through the external-control bridge (the "ROS Humble streaming" path) rather than the `/ro1/hardware` topics.

Prerequisites:
- A ROS Humble streaming session must be active so the bridge is up and engaged: menu -> Settings -> External Control -> Streaming -> Start (Controlled By = ROS Humble), or an External Control step with Controlled By = ROS Humble.
- The arm must be unbraked to actually move.

Each script auto-detects the bot id (or pass `--bot-id=<BOT_ID>`) and reads the current pose / joint state off the bridge, so streams always start from the live configuration.

Nudge the tooltip +Z 2cm and back:

```
./run.sh python3 ./src/stream_pose.py
```

Rotate J4 & J5 +0.1 rad and back (direct joint stream):

```
./run.sh python3 ./src/stream_joints.py
```

Rotate J4 & J5 +0.1 rad via a timed joint trajectory:

```
./run.sh python3 ./src/stream_joint_trajectory.py
```

Trace a 10cm axis-aligned cube with the tooltip via **joint streaming with client-supplied velocities** (client-side IK against the URDFs in `src/robot_urdfs/`):

```
./run.sh python3 ./src/stream_cube_vel.py [--robot thor] [--size 0.10] [--speed 0.05] [--rate 100]
```

- The robot only *uses* the streamed velocities when the `trustClientStreamVelocity` feature flag is enabled on the control box (arm-control-bot restart required). With the flag off, or with `--no-velocities`, the robot ignores velocity and smooths the position stream as before — run both to A/B the difference.
- Pass `--dry-run` first to precompute and validate the trajectory (workspace + IK checks) without moving the arm. The cube extends +X/+Y/+Z from the current tooltip pose.
- The script prints tracking-error stats (commanded vs `robot_joints`) at the end of each run.

Trace a circle with the tooltip (same streaming machinery as the cube, but one smooth constant-curvature path with no corner stops — a cleaner probe of steady-state tracking lag):

```
./run.sh python3 ./src/stream_circle_vel.py [--robot thor] [--radius 0.05] [--plane xy] [--laps 2] [--speed 0.05] [--rate 100]
```

- The circle passes through the current tooltip and extends away from it in the plane's first axis (+X for `xy`/`xz`, +Y for `yz`), so make sure that side is clear.
- Same velocity flags as `stream_cube_vel.py`: `--no-velocities`, `--zero-velocities`, `--dry-run`.

Measure stream-path jitter, and what it does to trusted-velocity tracking:

```
./run.sh python3 ./src/measure_stream_jitter.py --passive-only          # cadence + loop stats only, no motion
./run.sh python3 ./src/measure_stream_jitter.py                         # + clean single-joint wave baseline
./run.sh python3 ./src/measure_stream_jitter.py --inject delay --delay-prob 0.05 --delay-max 50
```

- Phases: (1) `robot_joints` inter-arrival stats, (2) the client send-loop's own wake-up jitter, (3) a raised-cosine wave on one joint (default joint5) streamed with velocities, under an optional injected fault model: `delay`, `drop`, `burst`, or `stale-vel`.
- With `trustClientStreamVelocity` on, a sample arriving `d` seconds late while the joint moves at `v` rad/s becomes a velocity transient of roughly `v*d/fohLpfTau` — so use this to sanity-check a small `fohLpfTau` against realistic jitter before trusting it. Compare `--inject none` against the injection modes, and add `--no-velocities` to A/B how the position-smoothing path digests the same abuse.

Open -> closed -> open the gripper:

```
./run.sh python3 ./src/stream_gripper.py
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
  - sensor_msgs/msg/Imu
- `/<BOT_ID>/ro1/hardware/jacobian`
  - std_msgs/msg/Float64MultiArray
- `/<BOT_ID>/ro1/hardware/joint_state`
  - Read the current joint state of the arm
  - sensor_msgs/msg/JointState
- `/<BOT_ID>/ro1/hardware/joint_trajectory`
  - Write a joint trajectory for the arm to perform
  - trajectory_msgs/msg/JointTrajectory
- `/<BOT_ID>/ro1/hardware/pose`
  - geometry_msgs/msg/PoseStamped
- `/<BOT_ID>/ro1/hardware/pose/write`
  - Read an write 
  - geometry_msgs/msg/PoseStamped

# External-control bridge topics (ROS Humble streaming)

These topics are published under `/<BOT_ID>/external` while a ROS Humble streaming session is active (see "Stream commands over the external-control bridge" above). The robot's current state is republished live so streams can start from the current configuration.

- `/<BOT_ID>/external/robot_pose`
  - Current tooltip pose (read)
  - geometry_msgs/msg/PoseStamped
- `/<BOT_ID>/external/pose`
  - Stream a target tooltip pose (write)
  - geometry_msgs/msg/PoseStamped
- `/<BOT_ID>/external/robot_joints`
  - Current joint state (read)
  - sensor_msgs/msg/JointState
- `/<BOT_ID>/external/joints`
  - Stream target joint positions (write)
  - sensor_msgs/msg/JointState
- `/<BOT_ID>/external/trajectory`
  - Stream a joint trajectory (write)
  - trajectory_msgs/msg/JointTrajectory
- `/<BOT_ID>/external/gripper`
  - Stream a gripper command `[position 0..1, velocity 0..1]` (write)
  - std_msgs/msg/Float64MultiArray
