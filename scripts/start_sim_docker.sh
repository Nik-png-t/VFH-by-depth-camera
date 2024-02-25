#!/bin/bash

# Function to display the script usage
usage() {
    echo "Usage: $0 [-h] [-c image_name]"
    echo "  -h              Display this help message."
    echo "  -c image_name     Specify the name of docker image."
    exit 1
}

# Default values
image_name=""

# Parse the command-line arguments
while getopts ":hc:n:" opt; do
    case $opt in
        h)
            usage
            ;;
        c)
            image_name="$OPTARG"
            ;;
        \?)
            echo "Invalid option: -$OPTARG"
            usage
            ;;
        :)
            echo "Option -$OPTARG requires an argument."
            usage
            ;;
    esac
done

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Container run
CONTAINER_ID=$(docker run -it --rm -d --privileged --user=$(id -u $USER):$(id -g $USER) --network=host --name px4_simulation --env="DISPLAY" --env="LIBGL_ALWAYS_SOFTWARE=0" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$SCRIPT_DIR/..:/home/docker/app" $image_name)
# ./QGroundControl.AppImage --appimage-extract-and-run

# Run gazebo
docker exec -it $CONTAINER_ID /bin/bash -c "sudo apt-get install -y libpulse-dev && \
                                            source /opt/ros/noetic/setup.bash && \
                                            source /PX4-Autopilot/Tools/setup_gazebo.bash \
                                            /PX4-Autopilot /PX4-Autopilot/build/px4_sitl_default && \
                                            export ROS_PACKAGE_PATH=/opt/ros/noetic/share:/PX4-Autopilot:/PX4-Autopilot/Tools/sitl_gazebo && \
                                            export GAZEBO_MODEL_PATH=/home/docker/app/sim/models:/PX4-Autopilot/Tools/sitl_gazebo/models
                                            /home/docker/app/scripts/start_sim.sh"

echo "STOP DOCKER CONTAINER!"
docker stop $CONTAINER_ID
