FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y \
    python3 \
    python3-pip \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-moveit-msgs \
    ros-humble-control-msgs \
    ros-humble-sensor-msgs \
    ros-humble-trajectory-msgs \
    python3-colcon-common-extensions

WORKDIR /app

RUN mkdir -p /etc/standardbots/configuration/
COPY ./cyclonedds.xml /etc/standardbots/configuration/cyclonedds.xml

COPY ./ ./

# Build standard_bots_msgs package
RUN source /opt/ros/humble/setup.bash \
    && colcon build --packages-select standard_bots_msgs

# Add ROS environment setup to .bashrc for interactive shells
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /app/install/setup.bash' >> ~/.bashrc

# Install Python dependencies
RUN source /opt/ros/humble/setup.bash \
    && source ./install/setup.bash \
    && pip3 install rclpy

# Copy requirements.txt and install Python dependencies
COPY ./requirements.txt .
RUN pip3 install -r requirements.txt

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/etc/standardbots/configuration/cyclonedds.xml

# CMD [ "python3", "src/write_poses.py" ]
CMD [ "python3", "src/plan_motion_plan.py", "data/joint_positions.json" ]
