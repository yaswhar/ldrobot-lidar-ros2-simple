#!/bin/bash
# /home/mohammad/Desktop/calibrate_dynamixel.sh -- motor-driven homing tool.
# Interactive: jog the joints, check the signs, `norm`, measure the platform
# height with it level, `cap h <metres>`. Writes the EEPROM Homing Offsets, the
# record, and the last-pose file the actuator needs at its next start.
# Same mounts as run_dynamixel.sh so the pose file lands where the actuator
# looks for it. Keep hands clear: the joints move under torque.
DXL_PORT="${DXL_PORT:-/dev/ttyUSB0}"
DESKTOP=/home/mohammad/Desktop
CACHE=/home/mohammad/ldlidar_platform
mkdir -p "$CACHE"

docker run --rm -it --name ros2_dxl_cal --privileged --network host \
  --device="${DXL_PORT}" \
  -v "$DESKTOP":/mnt/host_desktop \
  -v "$CACHE":/root/.cache/ldlidar_platform \
  ros2_lidar_image bash -lc "source /opt/ros/humble/setup.bash && \
  source ~/ros2_ws/install/setup.bash && \
  ros2 run ldlidar_node dynamixel_calibrate.py \
    --params /mnt/host_desktop/dynamixel_actuator_param.yaml \
    --planner-params /mnt/host_desktop/oscillation_planner_param.yaml \
    --out /mnt/host_desktop/dynamixel_offsets.yaml"
echo; read -p "Done. Press Enter to close... "
