import rclpy
from rclpy.node import Node
from math import acos
from numpy import rad2deg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry as Odom


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__("obstacle_avoider")

        self.laser_sub = self.create_subscription(
            LaserScan, "scan", callback=self.get_laser, qos_profile=10
        )

        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.odom_sub = self.create_subscription(
            Odom, "odom", callback=self.avoid_obstacle, qos_profile=10
        )

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, callback=self.run_loop)
        #
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        #
        self.current_angle = None
        self.prev_coord = [0, 0]
        self.goal_angle = 90
        self.lin_goal_reached = False
        self.done = False
        self.avoid_turn = False
        self.begin_turn = False
        self.begin_obstacle_move = False
        self.dists = None
        self.obstacle_range = 1.0
        self.ang_direction = None
        self.turn_reset = True
        self.avoid_move = False

    def get_angle(self, w):
        """
        Get current neato angle
        """
        return rad2deg(acos(w) * 2)

    def get_laser(self, msg):
        """
        Get current neato laser data
        """
        self.dists = {
            "deg0": msg.ranges[0],
            "deg90": msg.ranges[90],
            "deg270": msg.ranges[270],
        }

    def check_obstacle(self):

        if self.dists["deg0"] > self.obstacle_range or self.dists["deg0"] == 0.0:
            return False
        self.begin_turn = False
        return True

    def avoid_obstacle(self, msg):
        if self.avoid_turn is True:
            self.turn_reset = False
            self.begin_turn = False
            return
        self.current_angle = self.get_angle(msg.pose.pose.orientation.w)
        if self.begin_turn is False:
            self.lin_vel = 0.0
            if self.dists["deg90"] >= self.dists["deg270"]:
                self.ang_direction = -1.0
            else:
                self.ang_direction = 1.0
            self.ang_vel = 0.3 * self.ang_direction
            self.goal_angle = self.current_angle + 90 * self.ang_direction
            self.begin_turn = True
            self.avoid_turn = False

        if self.ang_direction == -1.0 and self.avoid_turn is False:
            if self.current_angle <= self.goal_angle:
                self.ang_vel = 0.0
                self.avoid_turn = True
                self.begin_obstacle_move = False
        elif self.ang_direction == 1.0 and self.avoid_turn is False:
            if self.current_angle >= self.goal_angle:
                self.ang_vel = 0.0
                self.avoid_turn = True
                self.begin_obstacle_move = False

    def obstacle_move(self):
        if self.avoid_move is True:
            return
        if self.begin_obstacle_move is False:
            self.lin_vel = 0.3
            self.avoid_move = False
            self.begin_obstacle_move = True
        if self.ang_direction == -1.0 and self.avoid_move is False:
            if self.dists["deg90"] > self.obstacle_range or self.dists["deg90"] == 0.0:
                self.lin_vel = 0.0
                self.avoid_move = True
        if self.ang_direction == 1.0 and self.avoid_move is False:
            if (
                self.dists["deg270"] < self.obstacle_range
                or self.dists["deg270"] == 0.0
            ):
                self.lin_vel = 0.0
                self.avoid_move = True

    def continue_path(self, msg):

        self.current_angle = self.get_angle(msg.pose.pose.orientation.w)
        if self.begin_turn is False:
            self.ang_vel = -0.3 * self.ang_direction
            self.begin_turn = True
        if self.ang_direction == -1.0:
            if self.current_angle >= 0.0:
                self.ang_vel = 0.0
                self.turn_reset = True
        elif self.ang_direction == 1.0:
            if self.current_angle <= 0.0:
                self.ang_vel = 0.0
                self.turn_reset = True

    def run_loop(self):
        """
        Executes the Node runtime loop and publishes the "cmd_vel" topic
        """
        msg = Twist()
        odom = Odom()
        self.check_obstacle()
        if self.begin_turn is False and self.turn_reset is True:
            self.avoid_obstacle(odom)
        elif self.avoid_turn is True and self.avoid_move is False:
            self.obstacle_move()
        elif self.avoid_move is True and self.turn_reset is False:
            self.continue_path(odom)

        msg.linear.x = self.lin_vel
        msg.angular.z = self.ang_vel

        self.vel_pub.publish(msg)


def main():
    rclpy.init()
    obstacle_publisher = ObstacleAvoider()
    rclpy.spin(obstacle_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
