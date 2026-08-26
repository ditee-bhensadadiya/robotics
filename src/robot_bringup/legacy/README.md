# Legacy (unused) ESP32 bridge

Not built, not installed, not wired into any launch file. Kept only as a
reference for an earlier design that was superseded by
`robot_bringup/serial_bridge_node.py` + `firmware/argo_mini/argo_mini.ino`.

| File | Paired with | Superseded because |
|---|---|---|
| `esp32_bridge_node.py` | `firmware/esp32_motor_odom/esp32_motor_odom.ino` | Kinematics (cmd_vel -> normalized wheel speed) ran on the Jetson side with no closed-loop RPM control. `serial_bridge_node.py` instead sends RPM targets and lets the ESP32 run a PI loop on-board, which tracks speed more accurately and survives serial hiccups better. |
| `robot_bringup.yaml` | `esp32_bridge_node.py` | Parameter names (`serial_port`, `baud_rate`, ...) match the old node, not `serial_bridge_node.py` (`port`, `baud`). Never loaded by `bringup_launch.py`. |
| `ekf.yaml` | `esp32_bridge_node.py` + `esp32_motor_odom.ino` | Fuses `/odom` with `/imu/data_raw`, a topic only the old node published (the old firmware read an onboard MPU6050; `argo_mini.ino` doesn't). `serial_bridge_node.py` publishes `odom->base_footprint` directly instead. |

If the robot gets an IMU again and you want Jetson-side kinematics + EKF
fusion back, this is the starting point - but check it still matches
`serial_bridge_node.py`'s current topics/frames first.
