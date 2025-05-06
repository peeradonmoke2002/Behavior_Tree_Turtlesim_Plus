#!/usr/bin/env python3

# import os
# os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

import rclpy
from rclpy.node import Node, Parameter
import py_trees
import py_trees_ros as pt_ros
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
import math
import numpy as np
from turtlesim.msg import Pose as TurtlePose
from bt_turtlesim_plus.bt_method import *


class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')
        # 1) State
        self.pose = None
        self.waypoints = [
            (9.18, 9.64),
            (0.57, 9.79),
            (0.89, 1.24),
            (9.57, 1.37)
        ]
        self.current_index = 0

        # 2) Controller params
        self.declare_parameter('speed', 1.0)
        self.K_theta   = 2.0
        self.tol_dist  = 0.1

        # 3) ROS interfaces
        self.create_subscription(TurtlePose, '/turtle1/pose', self.pose_cb, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 4) Timer: runs at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("🔲 SquareMover started, looping through corners")

    def pose_cb(self, msg: TurtlePose):
        self.pose = msg

    def normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def timer_callback(self):
        # 1) Wait for first pose
        if self.pose is None:
            return

        # 2) Which waypoint?
        gx, gy = self.waypoints[self.current_index]

        # 3) Compute errors
        dx = gx - self.pose.x
        dy = gy - self.pose.y
        rho = math.hypot(dx, dy)
        theta_goal = math.atan2(dy, dx)
        e_theta = self.normalize_angle(theta_goal - self.pose.theta)

        cmd = Twist()
        # 4) If not at goal: send P-control command
        if rho > self.tol_dist:
            v = min(self.get_parameter('speed').value, rho)
            w = self.K_theta * e_theta
            cmd.linear.x  = v
            cmd.angular.z = w
        else:
            # 5) Arrived: stop, advance to next waypoint
            cmd = Twist()  # zeros
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self.current_index = 0  # wrap for continuous loop
                self.get_logger().info("✅ Completed all corners, looping back to start")
            else:
                self.get_logger().info(f"→ Moving to corner #{self.current_index+1}")

        # 6) Publish
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = SquareMover()
    # the run_square() method already does the work, so just spin to keep callbacks alive
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
