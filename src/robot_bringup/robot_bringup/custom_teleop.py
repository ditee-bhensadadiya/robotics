"""Standalone keyboard teleop - publishes geometry_msgs/Twist to /cmd_vel.

The two wheels are driven independently (they're different types), so every
turn key fully stops the inside wheel and drives only the outside wheel,
rather than mixing both wheels at different speeds. Only 'i'/'k' (straight
forward/backward) drive both wheels equally.

Each turn key sends a specific (linear.x, angular.z) combination - not
angular-only - chosen so that serial_bridge_node.py's own kinematics formula
(v_l = lin - ang*WHEEL_BASE/2, v_r = lin + ang*WHEEL_BASE/2, with
ang = angular.z * angular_scale) works out to exactly 0 for the inside wheel.
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
    'k': (-1, 0),
}

PIVOT_KEYS = {'j', 'l'}

# key -> side: +1 stops the left wheel (turns left, right wheel drives),
# -1 stops the right wheel (turns right, left wheel drives).
PIVOT_SIDES = {
    'j': 1,
    'l': -1,
}


def pivot_turn(key, turn):
    """(linear.x, angular.z) for a single-wheel pivot turn at the given key -
    the inside wheel stops completely, the other wheel does all the driving.
    Derived from serial_bridge_node.py's kinematics formula - see the module
    docstring."""
    side = PIVOT_SIDES[key]
    lin = turn * ANGULAR_SCALE * (WHEEL_BASE / 2.0)
    ang = side * turn
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
        i
   j         l
        k

i   : forward - both wheels equal
k   : backward - both wheels equal
j   : turn left - left wheel stops, right wheel drives
l   : turn right - right wheel stops, left wheel drives
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
