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
    ros-humble-trajectory-msgs


WORKDIR /app

RUN mkdir -p /etc/standardbots/configuration/
COPY ./cyclonedds.xml /etc/standardbots/configuration/cyclonedds.xml

RUN source /opt/ros/humble/setup.bash && pip3 install rclpy

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/etc/standardbots/configuration/cyclonedds.xml

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

COPY ./src/* ./app/

CMD [ "python3", "app/read_joint_states.py" ]
