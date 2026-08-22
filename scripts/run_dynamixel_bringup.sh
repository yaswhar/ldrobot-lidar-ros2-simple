#!/bin/bash
# Run BOTH DYNAMIXEL nodes (actuator + oscillation planner) in one container.
# Convenient for bench testing; the two nodes stay separate processes coupled
# only by /joint_goal.
#
#   MODE=1 slow(6.0s)  MODE=2 normal(4.5s)  MODE=3 fast(3.0s)
# Set DXL_PORT to a stable /dev/serial/by-id/... path if available.

export DISPLAY=:0
xhost +local:root

DXL_PORT="${DXL_PORT:-/dev/ttyUSB0}"
MODE="${MODE:-2}"
REPEAT="${REPEAT:-1}"
ACTUATOR_PARAMS="${ACTUATOR_PARAMS:-/mnt/host_desktop/dynamixel_actuator_param.yaml}"
PLANNER_PARAMS="${PLANNER_PARAMS:-/mnt/host_desktop/oscillation_planner_param.yaml}"

docker run --rm --name ros2_dynamixel --privileged --network host \
  --device="${DXL_PORT}" -v /dev:/dev \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/mohammad/Desktop:/mnt/host_desktop \
  ros2_lidar_image bash -lc "source /opt/ros/humble/setup.bash && \
  source ~/ros2_ws/install/setup.bash && \
  ros2 launch ldlidar_node dynamixel_bringup.launch.py \
  mode:=${MODE} repeat:=${REPEAT} \
  actuator_params_file:=${ACTUATOR_PARAMS} \
  planner_params_file:=${PLANNER_PARAMS}"

docker stop ros2_dynamixel 2>/dev/null
docker container prune -f
