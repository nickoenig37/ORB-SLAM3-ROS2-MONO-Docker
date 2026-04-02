#!/bin/bash

# Function to terminate all background processes and exit the script
function cleanup_and_exit {
    echo "Ctrl+C detected. Terminating all background processes..."
    sleep 1
    pid=$(pgrep -f "/root/colcon_ws/install/orb_slam3_ros2_wrapper/lib/orb_slam3_ros2_wrapper/(rgbd|rgbd_imu)")
    if [[ -n "$pid" ]]; then
        echo "Process is running with PID: $pid"
        echo "Killing process..."
        kill $pid
    fi
    sleep 5
    echo "Process killed successfully."
    exit 1
}

# Trap Ctrl+C signal and call the cleanup_and_exit function
trap cleanup_and_exit INT

export ROBOT_NAMESPACE=""
export ROBOT_X="0.0"
export ROBOT_Y="0.0"

# Set SENSOR_CONFIG=rgbd_imu to enable inertial RGB-D mode.
SENSOR_CONFIG=${SENSOR_CONFIG:-rgbd}
ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py sensor_config:=${SENSOR_CONFIG} &

wait