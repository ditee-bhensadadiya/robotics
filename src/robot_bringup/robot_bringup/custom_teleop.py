"""Standalone keyboard teleop - publishes geometry_msgs/Twist to /cmd_vel.

Same key layout as teleop_twist_keyboard, except 'j'/'l' (pure left/right
turn) send a specific (linear.x, angular.z) combination - not angular-only -
chosen so that serial_bridge_node.py's own kinematics formula
(v_l = lin - ang*WHEEL_BASE/2, v_r = lin + ang*WHEEL_BASE/2, with
ang = angular.z * angular_scale) works out to exactly 0 for one wheel.
No serial_bridge_node.py changes needed - this is done entirely with the
numbers sent from here.

WHEEL_BASE and ANGULAR_SCALE below must match serial_bridge_node.py's
WHEEL_BASE constant and bringup_launch.py's angular_scale default - if either
changes, update the matching value here too.
"""
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

WHEEL_BASE = 0.41
ANGULAR_SCALE = 0.5

MOVE_BINDINGS = {
    'i': (1, 0),
    ',': (-1, 0),
    'u': (1, 1),
    'o': (1, -1),
    'm': (-1, 1),
    '.': (-1, -1),
}

# Pure turn keys: (x, th) computed per-key below, not from a fixed ratio -
# see pivot_turn().
PIVOT_KEYS = {'j', 'l'}


def pivot_turn(key, turn):
    """(linear.x, angular.z) for a single-wheel pivot turn at the given key.
    'j' zeroes the left wheel (turns left); 'l' zeroes the right wheel
    (turns right). Derived from serial_bridge_node.py's kinematics formula -
    see the module docstring."""
    lin = turn * ANGULAR_SCALE * (WHEEL_BASE / 2.0)
    ang = turn if key == 'j' else -turn
    return lin, ang

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
j/l : pivot turn in place - only one wheel drives, the other stays at 0
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
            if key in PIVOT_KEYS:
                # pivot_turn() already returns final linear.x/angular.z values -
                # no further scaling by speed/turn here.
                x, th = pivot_turn(key, turn)
            elif key in MOVE_BINDINGS:
                mx, mth = MOVE_BINDINGS[key]
                x, th = mx * speed, mth * turn
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
            twist.linear.x = x
            twist.angular.z = th
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
