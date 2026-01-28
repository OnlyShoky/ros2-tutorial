#!/usr/bin/env python3
"""
My First ROS2 Publisher!
Publishes a counter message every second.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """A simple publisher node."""
    
    def __init__(self):
        # Initialize the node with a name
        super().__init__('simple_publisher')
        
        # Create a publisher
        # - Topic name: /my_topic
        # - Message type: String
        # - Queue size: 10
        self.publisher = self.create_publisher(String, '/my_topic', 10)
        
        # Create a timer (calls callback every 1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Counter for our messages
        self.counter = 0
        
        self.get_logger().info('Simple Publisher has started!')

    def timer_callback(self):
        """Called every second by the timer."""
        # Create a message
        msg = String()
        msg.data = f'Hello ROS2! Count: {self.counter}'
        
        # Publish the message
        self.publisher.publish(msg)
        
        # Log what we published
        self.get_logger().info(f'Published: "{msg.data}"')
        
        self.counter += 1


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create our node
    node = SimplePublisher()
    
    try:
        # Keep the node running
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
