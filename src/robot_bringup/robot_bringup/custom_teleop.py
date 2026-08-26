"""Standalone keyboard teleop - publishes geometry_msgs/Twist to /cmd_vel.

Same key layout as teleop_twist_keyboard. The "stop one wheel, pivot with the
other" turning behavior is NOT implemented here - /cmd_vel only carries
linear/angular velocity, not per-wheel commands. That conversion happens
downstream in serial_bridge_node.py (disable_tank_turns), which already
zeroes the slower wheel whenever a turn is commanded.
"""
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MOVE_BINDINGS = {
    'i': (1, 0),
    ',': (-1, 0),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    'o': (1, -1),
    'm': (-1, 1),
    '.': (-1, -1),
}

SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

HELP = """
Moving around:
   u    i    o
   j    k    l
   m    ,    .

k : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = Node('custom_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    speed = 0.5
    turn = 1.0

    print(HELP)
    print(f'currently:\tspeed {speed}\tturn {turn}')

    try:
        while True:
            key = get_key(settings)
            if key in MOVE_BINDINGS:
                x, th = MOVE_BINDINGS[key]
            elif key in SPEED_BINDINGS:
                speed *= SPEED_BINDINGS[key][0]
                turn *= SPEED_BINDINGS[key][1]
                x, th = 0, 0
                print(f'currently:\tspeed {speed:.3f}\tturn {turn:.3f}')
            else:
                x, th = 0, 0
                if key == '\x03':  # Ctrl-C
                    break

            twist = Twist()
            twist.linear.x = x * speed
            twist.angular.z = th * turn
            pub.publish(twist)
    finally:
        twist = Twist()
        pub.publish(twist)  # stop the robot on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
