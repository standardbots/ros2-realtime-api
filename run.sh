#!/bin/bash

# ./run.sh
# ./run.sh "python3 ./src/write_poses.py ./data/pose1.log"

# Define the base Docker run command
DOCKER_CMD="docker run --rm --net=host --privileged ros2-realtime-api"

# Check if an argument is provided
if [ $# -eq 0 ]; then
    # If no argument, run the container with default settings
    echo "Running Docker container with default settings..."
    $DOCKER_CMD
else
    # Pass all arguments directly to the container
    echo "Running Docker container with arguments: $@"
    $DOCKER_CMD "$@"
fi
