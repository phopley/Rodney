#!/bin/bash

function log() {
  logger -s -p user.$1 ${@:2}
}

log info "rodney-base: Using workspace setup file /home/ubuntu/catkin_ws/devel/setup.bash"
source /home/ubuntu/rodney_ws/devel/setup.bash

source /etc/ubiquity/env.sh
log info "rodney-base: Launching ROS_HOSTNAME=$ROS_HOSTNAME, ROS_IP=$ROS_IP, ROS_MASTER_URI=$ROS_MASTER_URI, ROS_LOG_DIR=$log_path"

export ROS_HOME=$(echo ~ubuntu)/.ros
roslaunch rodney rodney.launch &
PID=$!

log info "rondey-base: Started roslaunch as background process, PID $PID, ROS_LOG_DIR=$ROS_LOG_DIR"
wait "$PID"
