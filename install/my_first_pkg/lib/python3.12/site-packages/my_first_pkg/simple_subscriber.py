#!/usr/bin/env python3
"""
My First ROS2 Subscriber!
Listens to messages on /my_topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """A simple subscriber node."""
    
    def __init__(self):
        super().__init__('simple_subscriber')
        
        # Create a subscription
        self.subscription = self.create_subscription(
            String,           # Message type
            '/my_topic',      # Topic name
            self.callback,    # Callback function
            10                # Queue size
        )
        
        self.get_logger().info('Simple Subscriber has started!')

    def callback(self, msg):
        """Called when a message is received."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
