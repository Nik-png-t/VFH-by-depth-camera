#!/bin/bash

# Setup environment
SCRIPT_DIR=/home/nikita/catkin_ws/src/BuildMapByCamera/scripts #$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
echo $ROS_MASTER_URI
MODEL="uav_vision"
RVIZ_ENABLE=true
WORLD="mapping_test" # mapping_village - , mapping_test - simple objects for test
roslaunch $SCRIPT_DIR/../launch/uav_simulator.launch \
          sdf:=$SCRIPT_DIR/../sim/models/$MODEL/$MODEL.sdf \
          world:=$SCRIPT_DIR/../sim/worlds/$WORLD.world \
          script_directory:=$SCRIPT_DIR \
          rviz_enable:=$RVIZ_ENABLE
