"""
Module for creating a teleop ros2 node.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import tty
import select
import sys
import termios

DIRECTION = {"w": 0.3, "a": 1.0, "s": -0.3, "d": -1.0, "x": 0.0}


class Teleop(Node):
    """
    A Teleop node to control a Neato robot with user input.

    Attributes:
        vel_pud - publisher node
            A node which publishes the 'cmd_vel' topic.
        timer - a timer node
            A node which governs the node's loop rate.

    Methods:
        run_loop():
            Executes the Node's runtime loop.
    """

    def __init__(self):
        super().__init__("teleop_node")
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, callback=self.run_loop)

    def run_loop(self):
        msg = Twist()

        def get_key():
            tty.setraw(sys.stdin.fileno())
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return key

        settings = termios.tcgetattr(sys.stdin)
        key = None

        if key in DIRECTION or not key:
            key = get_key()
            if key == "\x03":
                rclpy.shutdown()

            if key == "a" or key == "d":
                msg.angular.z = DIRECTION[key]
            elif key == "w" or key == "s":
                msg.linear.x = DIRECTION[key]
            elif key == "x":
                msg.linear.x = DIRECTION[key]
                msg.angular.z = DIRECTION[key]

        self.vel_pub.publish(msg)


def main():
    rclpy.init()

    teleop_publish = Teleop()

    rclpy.spin(teleop_publish)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleop_publish.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
