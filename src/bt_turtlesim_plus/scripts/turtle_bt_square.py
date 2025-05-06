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

from bt_turtlesim_plus.bt_method import *


class TurtleBTNode(Node):
    def __init__(self):
        super().__init__('turtle_bt_node')
        # 1) Build & setup the tree
        self.root = self.create_tree()
        self.tree = pt_ros.trees.BehaviourTree(self.root)
        self.tree.setup(
            timeout=5.0,
            node=self,
            name='TurtleBT',
            blackboard_clients=[self],  # if you use the node itself as a BB client
            record_rosbag=False,
            publish_tree=True       # enable RQT visualization if desired
        )
        # 2) Start ticking
        self.timer = self.create_timer(0.1, self.tick_callback)
        self.get_logger().info("[MAIN] TurtleBTNode initialized")

    def create_tree(self):
        """
        Constructs the Behavior Tree:
        
        Root Selector (no memory)
        ├─ Stop Seq    (Sequence, no memory)
        │    ├─ AtCenter? [Condition]
        │    └─ Stop      [Action]
        └─ Go Seq      (Sequence, with memory)
             ├─ ReadPose   [Action]
             └─ MoveCenter [Action]
        """
        root = py_trees.composites.Selector('Root Fallback', memory=False)

        # Branch 1: when at center → Stop
        stop_seq = py_trees.composites.Sequence('Stop Seq', memory=False)
        at_center   = AtCenter('At Center?', self)
        stop        = Stop('Stop', self)
        stop_seq.add_children([at_center, stop])

        # Branch 2: otherwise ReadPose → MoveToCenter
        go_seq = py_trees.composites.Sequence('Go Seq', memory=True)
        read_pose   = ReadPose('Read Pose', self)
        move_center = MoveToCenter('Move Home', self)
        go_seq.add_children([read_pose, move_center])

        root.add_children([stop_seq, go_seq])
        return root

    def tick_callback(self):
        """
        Called at 10 Hz to tick the tree; stops once the root returns SUCCESS.
        """
        self.tree.tick()
        if self.root.status == py_trees.common.Status.SUCCESS:
            self.get_logger().info("[MAIN] 🎉 Tree succeeded, stopping ticks")
            # self.timer.cancel()

def main():
    rclpy.init()
    node = TurtleBTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()