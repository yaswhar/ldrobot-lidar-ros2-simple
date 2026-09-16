#!/bin/bash
# /home/mohammad/Desktop/run_dynamixel.sh -- actuator + planner in one container.
#
# 2026-09-16: the launch arguments are actuator_params_file / planner_params_file
# (the old params_file:= was silently ignored, so the desktop actuator YAML was
# never used). /root/.cache/ldlidar_platform is now bind-mounted from the host
# so the IK lookup table AND the last-pose file survive the --rm container: the
# actuator needs that file to resolve the 7:1 multi-turn ambiguity at startup.
# Run calibrate_dynamixel.sh once after fitting/re-calibrating the gearboxes.
export DISPLAY=:0
xhost +local:root

DXL_PORT="${DXL_PORT:-/dev/ttyUSB0}"
DESKTOP=/home/mohammad/Desktop
CACHE=/home/mohammad/ldlidar_platform          # ik_lut.npz + last_pose.yaml live here
mkdir -p "$CACHE"

docker run --rm --name ros2_dynamixel --privileged --network host \
  --device="${DXL_PORT}" \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DESKTOP":/mnt/host_desktop \
  -v "$CACHE":/root/.cache/ldlidar_platform \
  ros2_lidar_image bash -lc "source /opt/ros/humble/setup.bash && \
  source ~/ros2_ws/install/setup.bash && \
  ros2 launch ldlidar_node dynamixel_bringup.launch.py \
    actuator_params_file:=/mnt/host_desktop/dynamixel_actuator_param.yaml \
    planner_params_file:=/mnt/host_desktop/oscillation_planner_param.yaml"

docker stop ros2_dynamixel 2>/dev/null
docker container prune -f
