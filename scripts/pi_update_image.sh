#!/bin/bash
# scripts/pi_update_image.sh -- update the ldlidar workspace INSIDE ros2_lidar_image
# on the Raspberry Pi and commit the result back as ros2_lidar_image.
#
#   1. starts a container from the current image,
#   2. pulls branch $BRANCH of $REPO_URL into /root/ros2_ws/src/ldrobot-lidar-ros2
#      (re-clones if the folder is not a git checkout), DISCARDING any local
#      edits made inside the image,
#   3. makes sure dynamixel_sdk / yaml / numpy import,
#   4. rebuilds ldlidar_node (Python nodes, params, launch files),
#   5. commits the container as ros2_lidar_image (the old image stays reachable
#      as ros2_lidar_image:before-<date>),
#   6. copies the desktop files (run/calibrate scripts, .desktop entries, the
#      two params YAMLs) to $DESKTOP, backing up the ones already there.
#
# Run ON THE PI (host, not inside a container):   bash pi_update_image.sh
set -euo pipefail
IMAGE="${IMAGE:-ros2_lidar_image}"
BRANCH="${BRANCH:-devel}"
REPO_URL="${REPO_URL:-https://github.com/yaswhar/ldrobot-lidar-ros2-simple.git}"
DESKTOP="${DESKTOP:-/home/mohammad/Desktop}"
STAMP=$(date +%Y%m%d-%H%M)

echo "== 0. keep the current image reachable as $IMAGE:before-$STAMP"
docker tag "$IMAGE" "$IMAGE:before-$STAMP"

echo "== 1-4. update + rebuild inside a container"
docker rm -f dxl_update >/dev/null 2>&1 || true
docker run --name dxl_update --network host "$IMAGE" bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  cd /root/ros2_ws/src
  # find the existing checkout by its package, whatever the folder is called
  PKG=\$(find . -maxdepth 3 -path '*/ldlidar_node/package.xml' | head -1)
  if [ -n \"\$PKG\" ]; then SRC=\$(dirname \$(dirname \$PKG)); else SRC=./ldrobot-lidar-ros2; fi
  echo \"   workspace source folder: \$SRC\"
  if [ -d \"\$SRC/.git\" ]; then
    cd \"\$SRC\"
    git remote set-url origin '$REPO_URL'
    git fetch origin '$BRANCH'
    git checkout -B '$BRANCH' 'origin/$BRANCH'
  else
    rm -rf \"\$SRC\"
    git clone -b '$BRANCH' '$REPO_URL' \"\$SRC\"
    cd \"\$SRC\"
  fi
  echo '   source now at:' && git log --oneline -1
  if ! python3 -c 'import dynamixel_sdk, yaml, numpy' 2>/dev/null; then
    apt-get update
    apt-get install -y --no-install-recommends ros-humble-dynamixel-sdk python3-yaml python3-numpy
    rm -rf /var/lib/apt/lists/*
  fi
  chmod +x ldlidar_node/dynamixel/*.py
  cd /root/ros2_ws
  colcon build --packages-select ldlidar_node --symlink-install
  source install/setup.bash
  grep -q 'gear:' \$(ros2 pkg prefix ldlidar_node)/share/ldlidar_node/params/dynamixel_actuator.yaml \
    && echo '   ldlidar_node rebuilt; new params present (gear model)'
"
echo "== 5. commit as $IMAGE"
docker commit dxl_update "$IMAGE" >/dev/null
docker rm dxl_update >/dev/null

echo "== 6. desktop files -> $DESKTOP (old ones backed up as *.bak-$STAMP)"
mkdir -p "$DESKTOP"
for f in run_dynamixel.sh calibrate_dynamixel.sh dynamixel_actuator_param.yaml \
         oscillation_planner_param.yaml launch_dynamixel.desktop calibrate_dynamixel.desktop; do
  [ -f "$DESKTOP/$f" ] && cp "$DESKTOP/$f" "$DESKTOP/$f.bak-$STAMP" || true
done
docker run --rm -v "$DESKTOP":/mnt/host_desktop "$IMAGE" bash -lc \
  "cp \$(dirname \$(dirname \$(find /root/ros2_ws/src -maxdepth 3 -path '*/ldlidar_node/package.xml' | head -1)))/scripts/desktop/* /mnt/host_desktop/"
[ -f "$DESKTOP/run_dynamixel.sh" ] && [ -f "$DESKTOP/dynamixel_actuator_param.yaml" ] \
  || { echo "ERROR: desktop files were not copied out of the image (scripts/desktop missing?)"; exit 1; }
chmod +x "$DESKTOP"/run_dynamixel.sh "$DESKTOP"/calibrate_dynamixel.sh \
         "$DESKTOP"/launch_dynamixel.desktop "$DESKTOP"/calibrate_dynamixel.desktop
chown "$(stat -c %u:%g "$DESKTOP")" "$DESKTOP"/run_dynamixel.sh "$DESKTOP"/calibrate_dynamixel.sh \
         "$DESKTOP"/*.desktop "$DESKTOP"/dynamixel_actuator_param.yaml "$DESKTOP"/oscillation_planner_param.yaml 2>/dev/null || true
mkdir -p /home/mohammad/ldlidar_platform

echo
echo "DONE. Next: double-click 'Calibrate Dynamixel' (writes the Homing Offsets and the"
echo "last-pose file), then 'Launch Dynamixel'. Roll back with:"
echo "   docker tag $IMAGE:before-$STAMP $IMAGE"
