# Maps

Saved maps live here. navigation_launch.py defaults to `office_map.yaml`
in this directory.

Save the current slam_toolbox map (run bringup_launch.py and drive/teleop
around the space first) with:

```
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/robot_bringup/maps/office_map --ros-args -p save_map_timeout:=10000.0
```

This writes `office_map.yaml` + `office_map.pgm`. Re-run `colcon build`
(or copy both files straight into `install/robot_bringup/share/robot_bringup/maps/`
for a quick test) so navigation_launch.py can find them.
