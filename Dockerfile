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
    ros-humble-geometry-msgs \
    ros-humble-trajectory-msgs


WORKDIR /app

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN pip install standardbots
# ikpy: client-side FK/IK for the joint-space streaming examples (stream_cube_vel.py)
RUN pip install ikpy

RUN mkdir -p /etc/standardbots/configuration/
COPY ./cyclonedds.xml /etc/standardbots/configuration/cyclonedds.xml

RUN source /opt/ros/humble/setup.bash && pip3 install rclpy

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/etc/standardbots/configuration/cyclonedds.xml
ENV ROS_DOMAIN_ID=1
# Flush stdout/stderr in real time instead of block-buffering (no TTY under `docker run`)
ENV PYTHONUNBUFFERED=1

COPY ./src/* ./src/

CMD [ "python3", "src/read_joint_states.py" ]
