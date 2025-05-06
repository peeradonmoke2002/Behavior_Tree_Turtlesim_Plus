#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class ClickPos(Node):
    def __init__(self):
        super().__init__('click_pos_node')
        # Subscribe to your click-topic (here “mouse_position”)
        self.create_subscription(
            Point,
            'mouse_position',
            self.listener_callback,
            10
        )
        self.get_logger().info('Waiting for click positions on “mouse_position”…')

    def listener_callback(self, msg: Point):
        # Print the x,y each time you click
        self.get_logger().info(f'Clicked position → x: {msg.x:.2f}, y: {msg.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ClickPos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
