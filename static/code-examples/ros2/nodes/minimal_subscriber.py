#!/usr/bin/env python3
"""
Minimal ROS 2 Subscriber Example

This node subscribes to String messages and logs them.

ROS 2 Version: Humble
Dependencies: rclpy, std_msgs
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    """
    A simple subscriber node that receives and logs messages.
    """

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create subscription (topic: 'chatter', queue size: 10)
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Minimal subscriber node started')

    def listener_callback(self, msg):
        """
        Callback function called when message received.

        Args:
            msg (String): Received message
        """
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
