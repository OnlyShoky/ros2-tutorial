#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from simple_arm_interfaces.srv import GripperControl

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client')
        self.cli = self.create_client(GripperControl, 'control_gripper')
        
        # Wait for server to start
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gripper Server...')
            
    def send_request(self, command_str):
        req = GripperControl.Request()
        req.command = command_str
        
        # Call service asynchronously
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = GripperClient()
    
    # Get command from terminal args, or default to "open"
    cmd = sys.argv[1] if len(sys.argv) > 1 else "open"
    
    response = client.send_request(cmd)
    client.get_logger().info(f'Result: {response.success}, Message: "{response.message}"')
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()