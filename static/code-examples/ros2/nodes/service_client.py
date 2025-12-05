#!/usr/bin/env python3
"""
ROS 2 Service Client Example

This demonstrates calling a service that adds two integers.

ROS 2 Version: Humble
Dependencies: rclpy, example_interfaces
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    """
    Service client that adds two numbers using a ROS 2 service.
    """

    def __init__(self):
        super().__init__('minimal_client_async')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service client ready')

    def send_request(self, a, b):
        """
        Send request to add two integers.

        Args:
            a (int): First number
            b (int): Second number

        Returns:
            Future: Async future for the response
        """
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        self.get_logger().info(f'Sending request: {a} + {b}')
        return self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print('Usage: service_client.py <num1> <num2>')
        return

    node = MinimalClientAsync()

    # Send request
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    future = node.send_request(a, b)

    # Wait for response
    rclpy.spin_until_future_complete(node, future)

    try:
        response = future.result()
        node.get_logger().info(f'Result: {a} + {b} = {response.sum}')
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
