"""
Module for creating a ros2 node which drives a neato in a square.
"""

import rclpy
from math import acos, dist
from numpy import rad2deg
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odom


class DriveSquare(Node):
    """
    A ros2 Node which directs a Neato to drive in a 1x1 meter square.

    Attributes:
        odom_sub - subscription node
            A node which subscribes to the 'odom' topic.
        vel_pub - publisher node
            A node which publishes the 'cmd_vel' topic.
        timer - timer node
            A node which governs DriveSquare's loop timer.
        lin_vel - float
            A float representing the Neato's linear velocity.
        ang_vel - float
            A float representing the Neato's angular velocity.
        prev_coord - list of floats
            A list of floats representing the Neato's previous coordinates
        goal_angle - float
            A float representing how far the Neato should turn per rotation.
        lin_goal_reached - boolean
            A boolean indicating whether the Neato has driven far enough to
            complete one edge of the square.
        done - boolean
            A boolean indicating whether the Neato has completed the square.

    Methods:
        get_angle(w):
                Returns the current angular orientation of the Neato.
            Args:
                w - float
                    A scalar value representing the Neato's rotation about
                    a vector perpendicular to the ground.
            Returns:
                    The scalar w converted to a degree value.
        square_draw(msg):
                Based on the current position of the Neato, as well as it's previous
                rotation, will set the Neato's linear and angular velocity to continue
                driving in a square.
            Args:
                msg - ros2 Odom topic
        run_loop():
                Executes the Node runtime loop and publishes the "cmd_vel" topic

    """

    def __init__(self):
        """
        Constructs all required attributes for the DriveSquare Node

        Parameters:
                odom_sub - subscription node
                A node which subscribes to the 'odom' topic.
            vel_pub - publisher node
                A node which publishes the 'cmd_vel' topic.
            timer - timer node
                A node which governs DriveSquare's loop timer.
            lin_vel - float
                A float representing the Neato's linear velocity.
            ang_vel - float
                A float representing the Neato's angular velocity.
            prev_coord - list of floats
                A list of floats representing the Neato's previous coordinates
            goal_angle - float
                A float representing how far the Neato should turn per rotation.
            lin_goal_reached - boolean
                A boolean indicating whether the Neato has driven far enough to
                complete one edge of the square.
            done - boolean
                A boolean indicating whether the Neato has completed the square.
        """
        super().__init__("drive_square")

        self.odom_sub = self.create_subscription(
            Odom, "odom", callback=self.square_draw, qos_profile=10
        )

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, callback=self.run_loop)

        self.lin_vel = 0.0
        self.ang_vel = 0.0

        self.prev_coord = [0, 0]
        self.goal_angle = 90

        self.lin_goal_reached = False
        self.done = False

    def get_angle(self, w):
        return rad2deg(acos(w) * 2)

    def square_draw(self, msg):
        crnt_coord = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        crnt_angle = self.get_angle(msg.pose.pose.orientation.w)

        if self.done:
            self.lin_vel = 0.0
            self.ang_vel = 0.0

        elif dist(self.prev_coord, crnt_coord) <= 1 and not self.lin_goal_reached:
            self.lin_vel = 0.2

        else:
            self.lin_goal_reached = True
            self.lin_vel = 0.0
            self.prev_coord = crnt_coord

            if crnt_angle <= self.goal_angle:
                self.lin_vel = 0.0
                self.ang_vel = 0.15
            else:
                print("Turn Complete")
                self.ang_vel = 0.0
                self.goal_angle += 90
                print(f"Next angle goal: {self.goal_angle} degrees")
                self.lin_goal_reached = False

    def run_loop(self):
        msg = Twist()
        msg.linear.x = self.lin_vel
        msg.angular.z = self.ang_vel

        self.vel_pub.publish(msg)


def main():
    rclpy.init()
    square_publisher = DriveSquare()
    rclpy.spin(square_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    square_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
