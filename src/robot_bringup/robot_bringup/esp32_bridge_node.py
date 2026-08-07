"""Bridges the ESP32 serial link to standard ROS2 topics.

Subscribes /cmd_vel (geometry_msgs/Twist), converts to per-wheel normalized
speed commands using differential-drive kinematics, and sends them to the
ESP32 as "V,<left_norm>,<right_norm>\\n".

Reads "O,<dt_ms>,<left_ticks>,<right_ticks>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\\n"
lines from the ESP32, integrates wheel odometry from the tick deltas, and
publishes /odom (nav_msgs/Odometry) and /imu/data_raw (sensor_msgs/Imu).

wheel_base_m and max_wheel_speed_mps are placeholders - measure your chassis
and your motors' real top speed, then override via the config yaml or
launch arguments instead of editing this file.
"""
import math
import threading

import rclpy
import serial
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class Esp32BridgeNode(Node):

    def __init__(self):
        super().__init__('esp32_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_diameter_m', 0.1524)   # 6 inch, confirmed
        self.declare_parameter('wheel_base_m', 0.30)          # TODO: measure actual track width
        self.declare_parameter('ticks_per_rev', 60)
        self.declare_parameter('max_wheel_speed_mps', 0.5)    # TODO: tune to your motors
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)  # False when robot_localization publishes odom->base_link

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.wheel_diameter = self.get_parameter('wheel_diameter_m').value
        self.wheel_base = self.get_parameter('wheel_base_m').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.max_speed = self.get_parameter('max_wheel_speed_mps').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.dist_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port {self.port}: {e}')
            raise

        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._stop = False
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info(
            f'esp32_bridge up on {self.port}@{self.baud}, '
            f'wheel_diameter={self.wheel_diameter:.4f}m wheel_base={self.wheel_base:.4f}m'
        )

    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        v_l = v - w * self.wheel_base / 2.0
        v_r = v + w * self.wheel_base / 2.0

        if self.max_speed <= 0.0:
            return
        norm_l = max(-1.0, min(1.0, v_l / self.max_speed))
        norm_r = max(-1.0, min(1.0, v_r / self.max_speed))

        line = f'V,{norm_l:.3f},{norm_r:.3f}\n'
        try:
            self.ser.write(line.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().warn(f'serial write failed: {e}')

    def read_loop(self):
        while not self._stop:
            try:
                raw = self.ser.readline()
            except serial.SerialException as e:
                self.get_logger().warn(f'serial read failed: {e}')
                continue
            if not raw:
                continue
            line = raw.decode('ascii', errors='ignore').strip()
            if line:
                self.handle_line(line)

    def handle_line(self, line: str):
        if not line.startswith('O,'):
            return
        parts = line.split(',')
        if len(parts) != 10:
            return
        try:
            dt_ms = float(parts[1])
            left_ticks = int(parts[2])
            right_ticks = int(parts[3])
            ax, ay, az, gx, gy, gz = (float(p) for p in parts[4:10])
        except ValueError:
            return

        now = self.get_clock().now()
        self.update_odom(dt_ms / 1000.0, left_ticks, right_ticks, now)
        self.publish_imu(ax, ay, az, gx, gy, gz, now)

    def update_odom(self, dt: float, left_ticks: int, right_ticks: int, stamp):
        d_left = left_ticks * self.dist_per_tick
        d_right = right_ticks * self.dist_per_tick
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        v = d_center / dt if dt > 0.0 else 0.0
        w = d_theta / dt if dt > 0.0 else 0.0

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = yaw_to_quaternion(self.theta)
            self.tf_broadcaster.sendTransform(t)

    def publish_imu(self, ax, ay, az, gx, gy, gz, stamp):
        imu = Imu()
        imu.header.stamp = stamp.to_msg()
        imu.header.frame_id = 'imu_link'
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        # No onboard orientation estimate; tell consumers not to trust it.
        imu.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu)

    def destroy_node(self):
        self._stop = True
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
