# Build and Run

## Reference environment

The verified local environment is:

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- PX4 SITL
- Gazebo Harmonic
- MAVSDK

The ROS workspace contains three `ament_python` packages:

- `arayuz_pkg`
- `iha_pkg`
- `g_isleme_pkg`

## Runtime dependencies

The source imports or invokes:

- `rclpy`, `rcl_interfaces`, `std_msgs`, `sensor_msgs`, and `cv_bridge`
- `mavsdk`
- `geographiclib`
- `PyQt5` and Qt WebEngine
- `opencv-python`/system OpenCV and NumPy
- `gz.transport13` and `gz.msgs10`
- PX4 SITL, Gazebo, and QGroundControl

The desktop orchestration helper additionally uses Tilix, `dconf`, `wmctrl`,
and `xdotool`.

## Build

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select arayuz_pkg iha_pkg g_isleme_pkg
source install/setup.bash
```

This three-package build completed successfully in the verified local
environment on 2026-07-28.

## Run the ROS 2 components

Start the ground-control station:

```bash
ros2 run arayuz_pkg qgc_interface
```

Start one mission controller for each PX4 instance:

```bash
ros2 run iha_pkg uav_control --ros-args \
  -p udp_port:=14541 -p mavsdk_port:=50051

ros2 run iha_pkg uav_control --ros-args \
  -p udp_port:=14542 -p mavsdk_port:=50052

ros2 run iha_pkg uav_control --ros-args \
  -p udp_port:=14543 -p mavsdk_port:=50053
```

Start the Gazebo camera processing nodes:

```bash
ros2 run g_isleme_pkg udp_img_prc --ros-args \
  -r __node:=img_prc_cam1 \
  -p udp_port:=14541 \
  -p image_source:=gz \
  -p auto_discover_topic:=false \
  -p gz_image_topic:=/world/baylands/model/x500_down_cam_1/link/down_cam_link/sensor/down_camera/image \
  -p display_window_name:="IHA KAMERA SOL"

ros2 run g_isleme_pkg udp_img_prc --ros-args \
  -r __node:=img_prc_cam2 \
  -p udp_port:=14542 \
  -p image_source:=gz \
  -p auto_discover_topic:=false \
  -p gz_image_topic:=/world/baylands/model/x500_down_cam_2/link/down_cam_link/sensor/down_camera/image \
  -p display_window_name:="IHA KAMERA LIDER"

ros2 run g_isleme_pkg udp_img_prc --ros-args \
  -r __node:=img_prc_cam3 \
  -p udp_port:=14543 \
  -p image_source:=gz \
  -p auto_discover_topic:=false \
  -p gz_image_topic:=/world/baylands/model/x500_down_cam_3/link/down_cam_link/sensor/down_camera/image \
  -p display_window_name:="IHA KAMERA SAG"
```

## PX4 SITL reference commands

The local simulation uses the custom `gz_x500_down_cam` airframe/model and the
`baylands` world. Set the PX4 source location before using these commands:

```bash
PX4_AUTOPILOT_DIR=/path/to/PX4-Autopilot
cd "$PX4_AUTOPILOT_DIR"
```

Instance 1:

```bash
PX4_GZ_FOLLOW_OFFSET_X=-7 \
PX4_GZ_FOLLOW_OFFSET_Y=0 \
PX4_GZ_FOLLOW_OFFSET_Z=3 \
PX4_SYS_AUTOSTART=4022 \
PX4_GZ_WORLD=baylands \
PX4_GZ_MODEL_POSE="0,5" \
PX4_SIM_MODEL=gz_x500_down_cam \
./build/px4_sitl_default/bin/px4 -i 1
```

Instance 2:

```bash
PX4_GZ_FOLLOW_OFFSET_X=-7 \
PX4_GZ_FOLLOW_OFFSET_Y=0 \
PX4_GZ_FOLLOW_OFFSET_Z=3 \
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4022 \
PX4_GZ_WORLD=baylands \
PX4_GZ_MODEL_POSE="0,0" \
PX4_SIM_MODEL=gz_x500_down_cam \
./build/px4_sitl_default/bin/px4 -i 2
```

Instance 3:

```bash
PX4_GZ_FOLLOW_OFFSET_X=-7 \
PX4_GZ_FOLLOW_OFFSET_Y=0 \
PX4_GZ_FOLLOW_OFFSET_Z=3 \
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4022 \
PX4_GZ_WORLD=baylands \
PX4_GZ_MODEL_POSE="0,-5" \
PX4_SIM_MODEL=gz_x500_down_cam \
./build/px4_sitl_default/bin/px4 -i 3
```

## Current simulation portability boundary

The simulation uses project-specific changes in a separate PX4 checkout:

- Airframe `4022_gz_x500_down_cam`.
- The `x500_down_cam` vehicle model.
- Downward camera, QR, red-field, and blue-field models.
- A modified Baylands world.
- PX4/Gazebo bridge and camera-stream changes.

At the time of this audit, those PX4/Gazebo assets were not tracked by the
`orion_swarm` repository. The local PX4 checkout was based on commit
`f38aba3c5be9`, with project-specific working-tree changes. They must be
packaged or represented as reviewed patches before the repository can claim a
fully reproducible simulation.

## Desktop helper

`scripts/launch_swarm_tilix.sh` creates Tilix sessions and arranges the PX4,
Gazebo, QGroundControl, camera, and ground-control windows.

The current helper invokes shell aliases named:

- `baydrone1`, `baydrone2`, `baydrone3`
- `drone14541`, `drone14542`, `drone14543`
- `gisleme1`, `gisleme2`

This helper is useful in the original development environment but is not yet a
portable launcher. It currently starts two image-processing aliases even
though the source supports a third camera node.

## Source checks

The following checks were completed successfully:

```bash
python3 -m py_compile <all Python source files>
bash -n scripts/launch_swarm_tilix.sh
colcon build --symlink-install \
  --packages-select arayuz_pkg iha_pkg g_isleme_pkg
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests
```

The focused unit suite currently contains 14 passing tests for the pure
navigation, swarm-message, and vision-message helpers.

The generated ROS linter scaffold is not treated as a functional flight test.
Real-flight, hardware-in-the-loop, and end-to-end simulation claims require
separate evidence.
