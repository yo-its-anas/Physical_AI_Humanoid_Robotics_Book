#!/usr/bin/env python3
"""
ROS 2 Action Server Example

This demonstrates a simple action server for Fibonacci sequence.

ROS 2 Version: Humble
Dependencies: rclpy, action_tutorials_interfaces
"""

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    """
    Action server that computes Fibonacci sequence.
    """

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        self.get_logger().info('Fibonacci action server started')

    def execute_callback(self, goal_handle):
        """
        Execute the Fibonacci action.

        Args:
            goal_handle: Goal handle from action client

        Returns:
            Result: Fibonacci sequence
        """
        self.get_logger().info('Executing goal...')

        # Get requested sequence length
        order = goal_handle.request.order

        # Initialize feedback message
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Compute Fibonacci sequence with feedback
        for i in range(1, order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Compute next Fibonacci number
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] +
                feedback_msg.partial_sequence[i-1]
            )

            # Publish feedback
            self.get_logger().info(
                f'Feedback: {feedback_msg.partial_sequence}'
            )
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1)  # Simulate work

        # Mark goal as succeeded
        goal_handle.succeed()

        # Create result message
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence

        self.get_logger().info(f'Goal succeeded: {result.sequence}')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
