# robotics

ROS 2 (Jazzy) workspace for the Argo Mini differential-drive robot: RPLidar
A1M8 + ESP32 motor/odometry bridge + slam_toolbox mapping.

## Layout

```
.                        <- colcon workspace root, build from here
├── src/
│   ├── robot_bringup/    <- this robot's ROS package
│   │   ├── launch/       <- bringup_launch.py (state publisher + lidar + serial bridge + SLAM)
│   │   ├── config/       <- slam_toolbox params
│   │   ├── urdf/         <- argo_mini.urdf.xacro
│   │   ├── udev/         <- 99-argo-mini.rules, stable /dev/esp32 + /dev/rplidar symlinks
│   │   ├── firmware/     <- active ESP32 sketch (argo_mini/argo_mini.ino)
│   │   ├── legacy/       <- superseded node/firmware/config, not built - see legacy/README.md
│   │   └── robot_bringup/serial_bridge_node.py  <- the node bringup_launch.py actually runs
│   └── rplidar_ros       <- NOT vendored here, see below
├── build/, install/, log/  <- colcon output, gitignored
```

`rplidar_ros` is installed as a system package (`ros-jazzy-rplidar-ros`) via
rosdep rather than checked into `src/`, so there's nothing to clone or keep in
sync there.

## First-time setup

```bash
cd /home/shivu/intern_codes/d/robotics      # workspace root
rosdep install --from-paths src --ignore-src -r -y
sudo cp src/robot_bringup/udev/99-argo-mini.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The udev rule file has instructions at the top for finding your ESP32's and
RPLidar's actual USB Product ID and filling them in - it ships with
placeholders (`XXXX`/`YYYY`) that must be replaced before it does anything.

## Build

Always from the workspace root (this directory, the one with `src/`):

```bash
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch robot_bringup bringup_launch.py
```

Defaults to `/dev/esp32` and `/dev/rplidar` (the udev symlinks above). Override
with `esp32_port:=` / `lidar_port:=` launch arguments if you haven't installed
the udev rule yet.

## Firmware

Flash `src/robot_bringup/firmware/argo_mini/argo_mini.ino` to the ESP32 with
the Arduino IDE. It speaks the `V <rpm_l> <rpm_r>` / `O <left_ticks> <right_ticks>`
protocol that `serial_bridge_node.py` expects; see that file's docstring for
the full serial protocol.
