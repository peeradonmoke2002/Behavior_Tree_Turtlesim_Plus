#!/usr/bin/env python3
import os
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros as pt_ros
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty

# ── 1) Leaf behaviors ─────────────────────────────────────

class ReadPose(py_trees.behaviour.Behaviour):
    def __init__(self, name, node: Node):
        super().__init__(name)
        self.node = node
        self.pose = None
        self.sub = None

    def setup(self, **kwargs):
        self.sub = self.node.create_subscription(
            Pose, '/turtle1/pose', self.cb, 10
        )
    def cb(self, msg: Pose):
        self.pose = msg
    def update(self):
        return py_trees.common.Status.SUCCESS if self.pose else py_trees.common.Status.FAILURE

class MoveToCenter(py_trees.behaviour.Behaviour):
    def __init__(self, name, node: Node):
        super().__init__(name)
        self.node = node
        self.pub = None

    def setup(self, **kwargs):
        self.pub = self.node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def update(self):
        # simple proportional drive to (5.5,5.5)
        msg = Twist()
        dx = 5.5 - ReadPose.pose.x
        dy = 5.5 - ReadPose.pose.y
        msg.linear.x = 1.0 * (dx**2 + dy**2)**0.5
        self.pub.publish(msg)
        if abs(dx) < 0.1 and abs(dy) < 0.1:
            self.pub.publish(Twist())  # stop
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class EatPizza(py_trees.behaviour.Behaviour):
    def __init__(self, name, node: Node):
        super().__init__(name)
        self.node = node
        self.cli = None

    def setup(self, **kwargs):
        self.cli = self.node.create_client(Empty, '/turtle1/eat')
        self.cli.wait_for_service()

    def update(self):
        req = Empty.Request()
        self.cli.call(req)
        self.node.get_logger().info('🍕 Ate one pizza!')
        return py_trees.common.Status.SUCCESS

# ── 2) Build the tree ──────────────────────────────────────

def create_tree(node: Node):
    # give Sequence its memory flag
    root = py_trees.composites.Sequence('BT Sequence', memory=False)

    read_pose   = ReadPose('Read Pose', node)
    move_center = MoveToCenter('Go Home', node)
    eat         = EatPizza('Eat Pizza', node)

    root.add_children([read_pose, move_center, eat])
    return root

# ── 3) Main ────────────────────────────────────────────────

def main(args=None):
    # 1) Init ROS 2
    rclpy.init(args=args)

    # 2) Create your node
    node = Node('turtle_bt_node')

    # 3) Build the behaviour tree
    root = create_tree(node)
    tree = pt_ros.trees.BehaviourTree(root)

    # 4) Wire up ROS: pass node, name, blackboard clients, etc.
    tree.setup(
        timeout=5.0,
        node=node,
        name='TurtleBT',
        blackboard_clients=[node],
        record_rosbag=False
    )

    # 5) Tick the tree at 10 Hz
    try:
        while rclpy.ok():
            tree.tick_tock(
                period_ms=int(1000 / 10),
                pre_tick_handler=None,
                post_tick_handler=None
            )
    finally:
        # 6) Clean shutdown
        node.get_logger().info('Shutting down TurtleBT')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

