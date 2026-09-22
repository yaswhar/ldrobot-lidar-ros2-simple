#!/bin/bash
# scripts/pi_sync.sh -- one-shot sync of the Raspberry Pi with the dev machine / GitHub.
#
# Runs ON THE PI HOST (as mohammad, not inside a container). It drives the persistent
# dev container and commits it back as the runtime image, i.e. exactly the manual
# sequence:  docker exec ros2_lidar_dev -> git pull -> colcon build -> desktop files
#            -> exit -> docker commit ros2_lidar_dev ros2_lidar_image
#
#   bash ~/pi_sync.sh                     # pull branch $BRANCH from GitHub
#   bash ~/pi_sync.sh ~/gearbox.bundle    # offline: apply a git bundle made on the dev
#                                         #   machine with  git bundle create x.bundle <old>..devel
#
# Dev-machine side of the loop:  git add -A && git commit -m "..." && git push origin devel
#
# What it does, in order:
#   0. tags the current runtime image as $IMAGE:before-<stamp>  (rollback point)
#   1. in $DEV: stashes any local edits, fast-forward pulls (GitHub, or the bundle);
#      if GitHub's certificate cannot be verified (no CA bundle in the container) it
#      installs ca-certificates and retries, then falls back to sslVerify=false
#   2. prints the resulting commit hash -- compare with `git rev-parse devel` on the dev machine
#   3. checks dynamixel_sdk / yaml / numpy import, chmod +x the node scripts
#   4. colcon build --packages-select ldlidar_node, verifies the new params are installed
#   5. copies scripts/desktop/* to $DESKTOP (old files backed up as *.bak-<stamp>)
#   6. docker commit $DEV $IMAGE
#   7. creates $CACHE (mounted by run/calibrate as /root/.cache/ldlidar_platform)
#   8. copies the newest version of this script to ~/pi_sync.sh for next time
set -euo pipefail
DEV="${DEV:-ros2_lidar_dev}"
IMAGE="${IMAGE:-ros2_lidar_image}"
BRANCH="${BRANCH:-devel}"
SRC="${SRC:-/root/ros2_ws/src/ldrobot-lidar-ros2-simple}"
DESKTOP="${DESKTOP:-$HOME/Desktop}"
CACHE="${CACHE:-$HOME/ldlidar_platform}"
BUNDLE="${1:-}"
STAMP=$(date +%Y%m%d-%H%M)

say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }

docker inspect "$DEV" >/dev/null 2>&1 || { echo "ERROR: dev container '$DEV' does not exist"; exit 1; }
docker start "$DEV" >/dev/null 2>&1 || true

say "0. rollback point: $IMAGE:before-$STAMP"
docker tag "$IMAGE" "$IMAGE:before-$STAMP"

if [ -n "$BUNDLE" ]; then
  [ -f "$BUNDLE" ] || { echo "ERROR: bundle '$BUNDLE' not found"; exit 1; }
  say "1. copying bundle into $DEV"
  docker cp "$BUNDLE" "$DEV:/root/sync.bundle"
  PULL_SRC=/root/sync.bundle
else
  PULL_SRC=origin
fi

say "1-4. pull + build inside $DEV"
docker exec "$DEV" bash -lc "
  set -eo pipefail
  cd '$SRC'
  if [ -n \"\$(git status --porcelain)\" ]; then
    echo '   local edits found -> git stash (recover with: git stash pop)'
    git stash push -m 'pi_sync $STAMP' >/dev/null
  fi
  if [ '$PULL_SRC' = origin ]; then
    if ! git pull --ff-only origin '$BRANCH' 2>/tmp/pull.err; then
      if grep -q 'certificate' /tmp/pull.err; then
        echo '   TLS: no CA bundle in the container -> installing ca-certificates'
        (apt-get update -qq && apt-get install -y -qq ca-certificates >/dev/null) || true
        git pull --ff-only origin '$BRANCH' || {
          echo '   still failing -> this route intercepts TLS; pulling with sslVerify=false (verify the hash below!)'
          git -c http.sslVerify=false pull --ff-only origin '$BRANCH'
        }
      else
        cat /tmp/pull.err; exit 1
      fi
    fi
  else
    git pull --ff-only '$PULL_SRC' '$BRANCH'
  fi
  echo '   HEAD:' \$(git rev-parse HEAD)
  git log --oneline -1
  chmod +x ldlidar_node/dynamixel/*.py
  cd /root/ros2_ws
  source /opt/ros/humble/setup.bash            # puts ros-humble-dynamixel-sdk on the Python path
  python3 -c 'import dynamixel_sdk, yaml, numpy' && echo '   deps ok' \
    || { echo '   ERROR: dynamixel_sdk / yaml / numpy not importable with ROS sourced'; exit 1; }
  colcon build --packages-select ldlidar_node 2>&1 | grep -E 'Starting|Finished|Failed|error|Summary'
  source install/setup.bash
  grep -q 'gear:' \$(ros2 pkg prefix ldlidar_node)/share/ldlidar_node/params/dynamixel_actuator.yaml \
    && grep -q GearModel \$(ros2 pkg prefix ldlidar_node)/lib/ldlidar_node/dynamixel_actuator_node.py \
    && echo '   installed node + params are the new ones' \
    || { echo '   ERROR: install space does not contain the new files'; exit 1; }
"

say "5. desktop files -> $DESKTOP"
TMP=$(mktemp -d)
docker cp "$DEV:$SRC/scripts/desktop/." "$TMP/"
mkdir -p "$DESKTOP"
# Scripts and .desktop entries are regenerated from the repo: never backed up (a
# .desktop.bak still shows up as a launcher icon). A YAML is backed up only if
# it was hand-edited (differs from the version being installed).
for f in "$TMP"/*; do
  b=$(basename "$f")
  case "$b" in
    *.yaml)
      if [ -f "$DESKTOP/$b" ] && ! cmp -s "$f" "$DESKTOP/$b"; then
        cp -p "$DESKTOP/$b" "$DESKTOP/$b.bak-$STAMP"
        echo "   $b differed from the repo version -> kept as $b.bak-$STAMP"
      fi ;;
  esac
  cp "$f" "$DESKTOP/$b"
done
chmod +x "$DESKTOP"/*.sh "$DESKTOP"/*.desktop
rm -rf "$TMP"
# redundant files: every older backup; hand-made launchers that duplicate
# launch_dynamixel.desktop (same Exec target)
find "$DESKTOP" -maxdepth 1 -name '*.bak-*' ! -name "*.bak-$STAMP" -print -delete | sed 's|.*/|   removed old backup |'
for f in "$DESKTOP"/*.desktop; do
  case "$(basename "$f")" in launch_dynamixel.desktop|calibrate_dynamixel.desktop) continue ;; esac
  if grep -q 'Exec=.*run_dynamixel.sh' "$f" 2>/dev/null; then
    echo "   removed duplicate launcher $(basename "$f")"; rm -f "$f"
  fi
done
ls -1 "$DESKTOP" | grep -E 'dynamixel|oscillation' | sed 's/^/   /'

say "6. docker commit $DEV -> $IMAGE"
docker commit "$DEV" "$IMAGE" >/dev/null && echo "   committed"

say "7. cache dir $CACHE (IK table + last-pose file)"
mkdir -p "$CACHE"

say "8. self-update ~/pi_sync.sh"
docker cp "$DEV:$SRC/scripts/pi_sync.sh" "$HOME/pi_sync.sh.new" && mv -f "$HOME/pi_sync.sh.new" "$HOME/pi_sync.sh"

echo
echo "DONE. Rollback:  docker tag $IMAGE:before-$STAMP $IMAGE"
