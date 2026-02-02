#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from simple_arm_interfaces.srv import GripperControl

class GripperServer(Node):
    def __init__(self):
        super().__init__('gripper_server')
        
        # Create the service
        # - Type: GripperControl
        # - Name: /control_gripper
        # - Callback: self.handle_request
        self.srv = self.create_service(
            GripperControl, 
            'control_gripper', 
            self.handle_request
        )
        self.get_logger().info('Gripper Server Ready! Waiting for commands...')

    def handle_request(self, request, response):
        """Callback function that runs when a request is received"""
        self.get_logger().info(f'Received command: "{request.command}"')
        
        # Process the command
        if request.command == "open":
            response.success = True
            response.message = "Gripper is now OPEN"
        elif request.command == "close":
            response.success = True
            response.message = "Gripper is now CLOSED"
        else:
            response.success = False
            response.message = "Unknown command"
            
        self.get_logger().info(f'Sending response: {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GripperServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()