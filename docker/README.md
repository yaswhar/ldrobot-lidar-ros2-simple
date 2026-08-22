# DYNAMIXEL platform — Docker notes

The two servo nodes reuse the existing **`ros2_lidar_image`**. That image did
**not** ship with DynamixelSDK, so it must be added and the image rebuilt /
committed / pushed for the change to reach the Pi.

## 1. Add DynamixelSDK to the image

Option A — layer it with the provided Dockerfile (recommended):

```bash
cd ~/ros2_ws/src/ldrobot-lidar-ros2
# Re-tag as ros2_lidar_image so the run scripts need no change:
docker build -f docker/Dockerfile.dynamixel -t ros2_lidar_image .
```

Option B — install into a running container and commit:

```bash
docker run -it --name dxl_setup ros2_lidar_image bash
#   inside the container:
apt-get update && apt-get install -y ros-humble-dynamixel-sdk python3-yaml
#   (or: pip3 install dynamixel-sdk pyyaml)
exit
docker commit dxl_setup ros2_lidar_image
docker rm dxl_setup
```

Verify the SDK is importable:

```bash
docker run --rm ros2_lidar_image bash -lc \
  "source /opt/ros/humble/setup.bash && python3 -c 'import dynamixel_sdk; print(dynamixel_sdk.__file__)'"
```

## 2. Rebuild the ROS workspace inside the image

The new nodes/launch/params are part of `ldlidar_node`, so rebuild that package
(the Dockerfile attempts this automatically; do it manually if your workspace
path differs from `/root/ros2_ws`):

```bash
colcon build --packages-select ldlidar_component ldlidar_node \
  --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
```

## 3. Push so it reaches the Pi

```bash
# tag + push to your registry, then pull on the Pi
docker tag ros2_lidar_image <registry>/ros2_lidar_image:dynamixel
docker push <registry>/ros2_lidar_image:dynamixel
# on the Pi:
docker pull <registry>/ros2_lidar_image:dynamixel
docker tag <registry>/ros2_lidar_image:dynamixel ros2_lidar_image
```

## 4. Run

Use the scripts in `../scripts/`:

- `run_dynamixel_actuator.sh` — hardware layer only
- `run_oscillation_planner.sh` — planner only (`mode` 1|2|3)
- `run_dynamixel_bringup.sh`   — both nodes in one container

> The DYNAMIXEL adapter and the LIDAR both enumerate as `/dev/ttyUSB*`. Prefer a
> stable `/dev/serial/by-id/...` path for the servos so they don't collide. Run
> `find_servos.py` first to discover the baud/IDs and pin them into the params.
