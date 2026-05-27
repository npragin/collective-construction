#! /usr/bin/env python

"""
    Author: Gaurav Gupta
    Email: gaurav@blackcoffeerobotics.com
    Ref: https://stackoverflow.com/questions/510357/how-to-read-a-single-character-from-the-user
"""


import rclpy
from geometry_msgs.msg import Twist
import time


def publish_event(event):
    vel_pub.publish(cmd_vel_msg)


class _Getch:
    """Gets a single character from standard input. Does not echo to the
    screen."""

    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty
        import sys

    def __call__(self):
        import sys
        import tty
        import termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


if __name__ == '__main__':

    rclpy.init_node('bcr_teleop_node')
    vel_pub = rclpy.Publisher('/cmd_vel', Twist, queue_size=1)
    rclpy.Timer(rclpy.Duration(0.05), publish_event)
    cmd_vel_msg = Twist()
    getch = _Getch()
    rclpy.loginfo("\n\tw: increment linear velocity by 0.1,\n\
        s: decrement linear velocity by 0.1,\n\
        a: increment angular velocity by 0.1,\n\
        d: decrement angular velocity by 0.1,\n\
        space: zero velocity command,\n\
        q: QUIT")

    try:
        while (not rclpy.is_shutdown()):
            key_in = getch()
            if key_in == "w":
                cmd_vel_msg.linear.x += 0.1
            elif key_in == "s":
                cmd_vel_msg.linear.x -= 0.1
            elif key_in == "d":
                cmd_vel_msg.angular.z -= 0.1
            elif key_in == "a":
                cmd_vel_msg.angular.z += 0.1
            else:
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = 0
                if (key_in == "q"):
                    break
            rclpy.loginfo("linear: %f, angular: %f"
                          % (cmd_vel_msg.linear.x, cmd_vel_msg.angular.z))
    except rclpy.ROSInterruptException:
        pass