#!/bin/bash
# Run the dynamixel_actuator_node (HARDWARE LAYER only) in the ros2_lidar_image.
#
# The DYNAMIXEL adapter and the LIDAR both show up as /dev/ttyUSB*. Set DXL_PORT
# to a stable /dev/serial/by-id/... path so they don't collide. The whole /dev
# is bind-mounted so by-id symlinks resolve inside the container.
#
# NOTE: the ROS package that hosts these nodes is 'ldlidar_node' (there is no
#       package literally named 'ldrobot-lidar-ros2').

export DISPLAY=:0
xhost +local:root

DXL_PORT="${DXL_PORT:-/dev/ttyUSB0}"
PARAMS="${PARAMS:-/mnt/host_desktop/dynamixel_actuator_param.yaml}"

docker run --rm --name ros2_dynamixel_actuator --privileged --network host \
  --device="${DXL_PORT}" -v /dev:/dev \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/mohammad/Desktop:/mnt/host_desktop \
  ros2_lidar_image bash -lc "source /opt/ros/humble/setup.bash && \
  source ~/ros2_ws/install/setup.bash && \
  ros2 launch ldlidar_node dynamixel_actuator.launch.py \
  params_file:=${PARAMS}"

docker stop ros2_dynamixel_actuator 2>/dev/null
docker container prune -f
