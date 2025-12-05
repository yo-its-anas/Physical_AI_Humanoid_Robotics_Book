---
id: a-ros2-cheatsheet
title: "Appendix A: ROS 2 Cheat Sheets"
sidebar_label: "A: ROS 2 Cheat Sheets"
sidebar_position: 2
description: "Quick reference for ROS 2 commands and patterns"
keywords: [ros2, cheatsheet, reference]
tags: [appendix, reference]
---


# Appendix A: ROS 2 Cheat Sheets

## Common Commands

### Package Management
```bash
# Create package
ros2 pkg create --build-type ament_python my_package

# Build workspace
colcon build
colcon build --packages-select my_package

# Source workspace
source install/setup.bash
```

### Node Management
```bash
# List nodes
ros2 node list

# Node info
ros2 node info /node_name

# Run node
ros2 run package_name executable_name

# Run with parameters
ros2 run package_name node --ros-args -p param:=value
```

### Topic Operations
```bash
# List topics
ros2 topic list

# Topic info
ros2 topic info /topic_name

# Echo topic
ros2 topic echo /topic_name

# Publish to topic
ros2 topic pub /topic_name std_msgs/msg/String "data: 'hello'"

# Topic rate
ros2 topic hz /topic_name
```

### Service Operations
```bash
# List services
ros2 service list

# Service type
ros2 service type /service_name

# Call service
ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

### Parameter Operations
```bash
# List parameters
ros2 param list

# Get parameter
ros2 param get /node_name parameter_name

# Set parameter
ros2 param set /node_name parameter_name value

# Dump parameters to file
ros2 param dump /node_name > params.yaml
```

### Launch Files
```bash
# Run launch file
ros2 launch package_name launch_file.py

# With arguments
ros2 launch package_name launch_file.py arg:=value
```

### ROS Bags
```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /topic1 /topic2

# Play bag
ros2 bag play my_bag.db3

# Info
ros2 bag info my_bag.db3
```

## Python Node Template

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Publisher
        self.pub = self.create_publisher(String, 'topic', 10)

        # Subscriber
        self.sub = self.create_subscription(
            String, 'input_topic',
            self.callback, 10
        )

        # Timer
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Parameter
        self.declare_parameter('my_param', 'default_value')

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Message Types

```python
# String
from std_msgs.msg import String
msg = String()
msg.data = "text"

# Twist (velocity commands)
from geometry_msgs.msg import Twist
msg = Twist()
msg.linear.x = 0.5
msg.angular.z = 0.1

# Image
from sensor_msgs.msg import Image
# Use cv_bridge to convert

# LaserScan
from sensor_msgs.msg import LaserScan

# Point Cloud
from sensor_msgs.msg import PointCloud2
```

## Troubleshooting

**Problem**: `ros2: command not found`
**Solution**: Source ROS 2: `source /opt/ros/humble/setup.bash`

**Problem**: Package not found after build
**Solution**: Source workspace: `source install/setup.bash`

**Problem**: DDS discovery issues
**Solution**: Check `ROS_DOMAIN_ID`, ensure same network

**Problem**: Permission denied on /dev/ttyUSB0
**Solution**: `sudo usermod -aG dialout $USER` (logout/login)

## See Also
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
