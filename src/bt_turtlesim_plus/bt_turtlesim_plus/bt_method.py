#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import py_trees
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
import math
import numpy as np


# ── 1) New Condition leaf ───────────────────────────────
class AtCenter(py_trees.behaviour.Behaviour):
    """
    Check if turtle is within 0.1m of (5.5,5.5).
    """
    def __init__(self, name, node: Node):
        super().__init__(name)
        self.node = node
        self.bb = py_trees.blackboard.Blackboard()
        self.logger = node.get_logger()

    def update(self):
        pose = self.bb.get('pose')
        if pose is None:
            return py_trees.common.Status.FAILURE
        x, y, theta = pose.x, pose.y, pose.theta
        inside = abs(x-5.5)<0.3 and abs(y-5.5)<0.3
        if inside:
            self.node.get_logger().info(f"[{self.name}] at center → SUCCESS")
            self.bb.set('at_center', True)
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(f"[{self.name}] not at center → FAILURE")
            self.bb.set('at_center', False)
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(f"Condition:: [{self.name}] terminating with status {new_status}")
        

# ── 2) Your existing Action leaves ──────────────────────
class ReadPose(py_trees.behaviour.Behaviour):
    def __init__(self, name, node: Node):
        super().__init__(name)
        self.node = node
        self.pose = None
        self.bb = py_trees.blackboard.Blackboard()


    def setup(self, **kwargs):
        self.subscriber = self.node.create_subscription(
            Pose, 'turtle1/pose', self.cb, 10
        )
        self.node.get_logger().info(f"[{self.name}] subscription ready")

    def cb(self, msg: Pose):
        # self.node.set_parameters([
        #     Parameter('last_pose_x', Parameter.Type.DOUBLE, msg.x),
        #     Parameter('last_pose_y', Parameter.Type.DOUBLE, msg.y),
        #     Parameter('last_pose_theta', Parameter.Type.DOUBLE, msg.theta)
        # ])
        self.pose = msg
        self.bb.set('pose', msg)

    def update(self):
        return py_trees.common.Status.SUCCESS if self.bb.get('pose') else py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.info(f"Action:: [{self.name}] terminating with status {new_status}")



class MoveToCenter(py_trees.behaviour.Behaviour):
    def __init__(self, name, node: Node):
        super().__init__(name)
        self.node          = node
        self.pub           = None
        self.gx, self.gy   = 5.5, 5.5
        self.K             = 0.60
        self.linear_speed  = 0.30
        self.rho_thresh    = 0.3
        self.bb            = py_trees.blackboard.Blackboard()
        self.logger = node.get_logger()

    def setup(self, **kwargs):
        self.pub = self.node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def update(self):
        # 1) read current pose
        pose  = self.bb.get('pose')
        if pose is None:
            return py_trees.common.Status.FAILURE
        x     = pose.x
        y     = pose.y
        theta = pose.theta


        # 2) compute relative error
        dx  = self.gx - x
        dy  = self.gy - y
        rho = np.hypot(dx, dy)

        # correct bearing error
        desired_yaw      = math.atan2(dy, dx)
        raw_error        = desired_yaw - theta
        angular_velocity = self.K * math.atan2(math.sin(raw_error),
                                               math.cos(raw_error))

        # 3) decide speeds
        if rho > self.rho_thresh:
            linear_velocity = self.linear_speed
        else:
            linear_velocity = 0.0
            angular_velocity = 0.0

        # 4) publish
        cmd = Twist()
        cmd.linear.x  = linear_velocity
        cmd.angular.z = angular_velocity
        self.pub.publish(cmd)

        # 5) finish when close
        if rho <= self.rho_thresh:
            self.bb.set('at_center', True)
            self.node.get_logger().info(f"[{self.name}] at center → SUCCESS")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        self.logger.debug(f"Action:: [{self.name}] terminated with status {new_status}")



class EatPizza(py_trees.behaviour.Behaviour):
    def __init__(self, name, node: Node):
        super().__init__(name)
        self.node = node
        self.cli = None

    def setup(self, **kwargs):
        self.cli = self.node.create_client(Empty, '/turtle1/eat')
        self.cli.wait_for_service()

    def update(self):
        self.cli.call(Empty.Request())
        return py_trees.common.Status.SUCCESS
    

class Stop(py_trees.behaviour.Behaviour):
    """
    Leaf node that publishes a zero‐velocity Twist once upon entry,
    writes a 'stopped' flag to the blackboard, and returns SUCCESS thereafter.
    """
    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self.node    = node
        self.pub     = None
        self.stopped = False          # local guard
        self.bb      = py_trees.blackboard.Blackboard()
        self.logger  = node.get_logger()

    def setup(self, **kwargs):
        """
        Called once during tree.setup(). Initialize publisher.
        """
        self.pub = self.node.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        self.node.get_logger().info(f"[{self.name}] setup complete")

    def initialise(self):
        """
        Called each time the behaviour is entered.
        Reset both local and blackboard flags.
        """
        self.stopped = False
        self.bb.set('stopped', False)

    def update(self) -> py_trees.common.Status:
        """
        On first tick after entry:
        - publish zero velocities
        - write 'stopped' flag to blackboard
        Always return SUCCESS.
        """
        if not self.stopped:
            zero_cmd = Twist()  # all zeros
            self.pub.publish(zero_cmd)
            self.node.get_logger().info(f"[{self.name}] published zero-velocity")
            # set both flags
            self.stopped = True
            self.bb.set('stopped', True)
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        self.logger.debug(f"Action:: [{self.name}] terminating with status {new_status}")

