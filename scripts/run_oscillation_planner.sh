#!/bin/bash
# Run the oscillation_planner_node (MOTION-PLANNING LAYER only) in the
# ros2_lidar_image, as a SECOND container/terminal alongside the actuator.
#
# The planner talks to the actuator ONLY over the /joint_goal topic (host
# network), so it needs no serial device. MODE selects the speed:
#   MODE=1 slow(6.0s)  MODE=2 normal(4.5s)  MODE=3 fast(3.0s)

export DISPLAY=:0
xhost +local:root

MODE="${MODE:-2}"
REPEAT="${REPEAT:-1}"
PARAMS="${PARAMS:-/mnt/host_desktop/oscillation_planner_param.yaml}"

docker run --rm --name ros2_oscillation_planner --network host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/mohammad/Desktop:/mnt/host_desktop \
  ros2_lidar_image bash -lc "source /opt/ros/humble/setup.bash && \
  source ~/ros2_ws/install/setup.bash && \
  ros2 launch ldlidar_node oscillation_planner.launch.py \
  mode:=${MODE} repeat:=${REPEAT} params_file:=${PARAMS}"

docker stop ros2_oscillation_planner 2>/dev/null
docker container prune -f
