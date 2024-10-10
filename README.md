# ROS Realtime API
Sample code to call the ros2 apis

## Setup
There is one time setup that needs to be done by a standard bots engineer. Please ask standardbots to do setup for you

1. Update the `cyclonedds.xml` NetworkInterface name to the adapter you will be using
2. Build
3. Run

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
./run.sh python3 ./src/write_poses.py ./data/pose3.log"
```

## Run in docker environement

1. `./run_shell.sh`
1. `cd /src`
1. `python3 ./src/plan_and_execute.py`

