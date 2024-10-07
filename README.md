# moveit2_examples
Sample code to call the moveit2 api

## Setup
The robot must be set up in a certain way before the it will work with moveit2 api

1. Disable the firewall: `sudo ufw disable`
2. Set the cyclone dds interface port to `wireless0` or `external0`

```
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"    xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <Interfaces>
                 <NetworkInterface name="internal0" priority="0"/>
            </Interfaces>
        </General>
    </Domain>
</CycloneDDS>
```
3. Turn `Enable ROS2 Control` and `Enable ROS2 motions`
3. Restart the services

## Building

```
./build.sh
```

## Running

```
./run.sh
```

## Run in docker environement

1. `./run_shell.sh`
1. `cd /src`
1. `python3 ./src/plan_and_execute.py`

