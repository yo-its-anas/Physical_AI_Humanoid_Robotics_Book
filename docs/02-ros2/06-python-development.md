---
id: 06-python-development
title: "Chapter 6: Building with ROS 2 & Python"
sidebar_label: "Ch 6: Python Development"
sidebar_position: 3
description: "Workspace setup, writing nodes, launch files, and parameters"
keywords: [ros2 python, rclpy, launch files, parameters]
tags: [part-ii, ros2, python]
---

# Chapter 6: Building with ROS 2 & Python

## Overview

### What You'll Learn
- **Create** ROS 2 Python packages with proper structure
- **Write** launch files for multi-node systems
- **Configure** parameters for runtime behavior modification
- **Record** and playback sensor data with ROS bags

**Estimated Time**: 7-9 hours

## Core Concepts

### Concept 1: Package Structure

```
my_robot_pkg/
├── package.xml          # Package metadata
├── setup.py             # Python setup configuration
├── setup.cfg            # Installation config
├── resource/            # Package marker
├── my_robot_pkg/        # Python modules
│   ├── __init__.py
│   ├── my_node.py
│   └── utils.py
├── launch/              # Launch files
│   └── robot_launch.py
├── config/              # Parameter files
│   └── params.yaml
└── test/                # Unit tests
```

### Concept 2: Launch Files

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='talker',
            name='talker_node',
            parameters=[{'topic_name': 'chatter'}]
        ),
        Node(
            package='my_robot_pkg',
            executable='listener',
            name='listener_node'
        )
    ])
```

### Concept 3: Parameters

```yaml
# config/params.yaml
talker_node:
  ros__parameters:
    topic_name: "chatter"
    publish_rate: 10.0
    message_prefix: "Hello"
```

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with defaults
        self.declare_parameter('topic_name', 'default_topic')
        self.declare_parameter('publish_rate', 1.0)

        # Get parameter values
        topic = self.get_parameter('topic_name').value
        rate = self.get_parameter('publish_rate').value
```

### Concept 4: ROS Bags

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /camera/image_raw /imu/data

# Playback
ros2 bag play my_recording.db3

# Info
ros2 bag info my_recording.db3
```

## Implementation

### Tutorial: Complete Robot Package

**Step 1: Create Package**
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_controller \
    --dependencies rclpy std_msgs geometry_msgs
```

**Step 2: Implement Node**
```python
# robot_controller/robot_controller/controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Parameters
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('update_rate', 10.0)

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', 10
        )

        # Timer
        rate = self.get_parameter('update_rate').value
        self.timer = self.create_timer(1.0/rate, self.control_loop)

        self.get_logger().info('Robot controller started')

    def control_loop(self):
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.angular.z = 0.1
        self.cmd_pub.publish(cmd)
```

**Step 3: Create Launch File**
```python
# launch/robot_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('robot_controller')

    # Parameter file
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to parameter file'
        ),

        Node(
            package='robot_controller',
            executable='controller',
            name='robot_controller',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        )
    ])
```

**Step 4: Update setup.py**
```python
from setuptools import setup
import os
from glob import glob

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'controller = robot_controller.controller:main',
        ],
    },
)
```

## Lab Exercises

### Lab 1: Multi-Node Robot System

**Objective**: Build system with sensor, controller, and actuator nodes

**Requirements**:
- Sensor node publishes simulated sensor data
- Controller processes sensor data and publishes commands
- Actuator node receives and logs commands
- Use launch file to start all nodes
- Configure behavior via parameters

**Deliverables**:
- Complete ROS 2 package
- Launch file starting all nodes
- Parameter file with at least 3 parameters
- README with usage instructions

## Summary

### Key Takeaways
1. **Package structure** organizes code, configs, and launch files
2. **Launch files** enable complex multi-node system startup
3. **Parameters** allow runtime configuration without code changes
4. **ROS bags** record sensor data for offline analysis

### Next Steps
➡️ [Chapter 7: URDF Robot Description](./07-urdf.md)
