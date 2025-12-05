"""
Example ROS 2 Launch File

This launch file starts multiple nodes with parameters.

ROS 2 Version: Humble
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate launch description with multiple nodes.
    """

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Publisher node
    publisher_node = Node(
        package='examples',
        executable='minimal_publisher',
        name='publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # Subscriber node (delayed start)
    subscriber_node = TimerAction(
        period=2.0,  # Wait 2 seconds before starting
        actions=[
            Node(
                package='examples',
                executable='minimal_subscriber',
                name='subscriber',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        publisher_node,
        subscriber_node
    ])
