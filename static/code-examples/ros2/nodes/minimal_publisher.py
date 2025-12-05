#!/usr/bin/env python3
"""
Minimal ROS 2 Publisher Example

This node publishes String messages to a topic at regular intervals.

ROS 2 Version: Humble
Dependencies: rclpy, std_msgs
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    """
    A simple publisher node that sends messages periodically.

    This demonstrates the basic pattern for ROS 2 publishers:
    creating a publisher, setting up a timer, and publishing messages.
    """

    def __init__(self):
        super().__init__('minimal_publisher')

        # Create publisher (topic: 'chatter', queue size: 10)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create timer (0.5 second interval)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.counter = 0
        self.get_logger().info('Minimal publisher node started')

    def timer_callback(self):
        """
        Timer callback function - called every 0.5 seconds.

        Creates and publishes a String message with incrementing counter.
        """
        msg = String()
        msg.data = f'Hello World: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
