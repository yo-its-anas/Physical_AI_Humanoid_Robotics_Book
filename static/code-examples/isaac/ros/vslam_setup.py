#!/usr/bin/env python3
"""
Isaac ROS VSLAM Configuration

Setup Visual SLAM using Isaac ROS packages.

ROS 2 Version: Humble
Dependencies: isaac_ros_visual_slam
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

class VSLAMNode(Node):
    """
    Node to interface with Isaac ROS VSLAM.
    """

    def __init__(self):
        super().__init__('vslam_node')

        # Subscribe to camera topics
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Subscribe to VSLAM pose output
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/visual_slam/tracking/slam_pose',
            self.pose_callback,
            10
        )

        self.get_logger().info('VSLAM node initialized')

    def image_callback(self, msg):
        """Process camera images"""
        pass  # Isaac ROS VSLAM handles this

    def camera_info_callback(self, msg):
        """Process camera calibration"""
        pass  # Isaac ROS VSLAM handles this

    def pose_callback(self, msg):
        """
        Receive estimated robot pose from VSLAM.

        Args:
            msg (PoseStamped): Estimated 6-DOF pose
        """
        pos = msg.pose.position
        self.get_logger().info(
            f'VSLAM Pose: x={pos.x:.2f} y={pos.y:.2f} z={pos.z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = VSLAMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
