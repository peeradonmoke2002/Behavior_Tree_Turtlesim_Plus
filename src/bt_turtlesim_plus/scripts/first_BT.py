#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import py_trees

from bt_turtlesim_plus.dummy_module import dummy_function, dummy_var


class DummyBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name="DummyBehavior"):
        super().__init__(name)

    def setup(self, timeout):
        return True

    def initialise(self):
        self.logger.info("Initialising DummyBehavior...")

    def update(self):
        dummy_function()
        self.logger.info(f"Dummy var: {dummy_var}")
        return py_trees.common.Status.SUCCESS


class BT_Test(Node):
    def __init__(self):
        super().__init__('bt_test_node')
        self.tree = self.create_behavior_tree()
        self.timer = self.create_timer(1.0, self.tick_tree)  # tick every 1 sec

    def create_behavior_tree(self):
        root = py_trees.composites.Sequence(name="Root", memory=False)
        dummy = DummyBehavior()
        root.add_child(dummy)
        return py_trees.trees.BehaviourTree(root)

    def tick_tree(self):
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = BT_Test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
