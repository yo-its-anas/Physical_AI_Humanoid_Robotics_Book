---
id: 05-fundamentals
title: "Chapter 5: ROS 2 Fundamentals"
sidebar_label: "Ch 5: ROS 2 Fundamentals"
sidebar_position: 2
description: "ROS 2 architecture, nodes, topics, services, and DDS communication"
keywords: [ros2, dds, middleware, nodes, topics]
tags: [part-ii, ros2]
---

# Chapter 5: ROS 2 Fundamentals

## Overview

### What You'll Learn
- **Explain** ROS 2 architecture and design philosophy
- **Understand** nodes, topics, services, and actions
- **Configure** DDS middleware for robot communication
- **Use** ROS 2 CLI tools for debugging

**Estimated Time**: 7-9 hours

## Background

ROS 2 is the second generation of the Robot Operating System, redesigned for production robotics with real-time performance, security, and multi-robot support.

**Key Improvements over ROS 1**:
- DDS middleware (no roscore dependency)
- Real-time capable
- Native Windows/macOS support
- Security (SROS2)
- Better embedded support

## Core Concepts

### Concept 1: Nodes

**Node** = single-purpose executable in the ROS graph

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node started')

def main():
    rclpy.init()
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Concept 2: Topics (Pub-Sub)

**Topic** = named bus for asynchronous message passing

```python
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.pub = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.callback)

    def callback(self):
        msg = String()
        msg.data = 'Hello ROS 2'
        self.pub.publish(msg)
```

### Concept 3: Services (Request-Response)

**Service** = synchronous RPC pattern

```python
from example_interfaces.srv import AddTwoInts

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.callback
        )

    def callback(self, request, response):
        response.sum = request.a + request.b
        return response
```

### Concept 4: Actions (Long-Running Tasks)

**Action** = goal-based asynchronous pattern with feedback

```python
from action_tutorials_interfaces.action import Fibonacci
from rclpy.action import ActionServer

class FibonacciServer(Node):
    def __init__(self):
        super().__init__('fibonacci_server')
        self._action_server = ActionServer(
            self, Fibonacci, 'fibonacci',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        # Long-running computation
        feedback = Fibonacci.Feedback()
        # ... computation ...
        goal_handle.succeed()
        result = Fibonacci.Result()
        return result
```

## Implementation

### Setup: ROS 2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_python my_robot_pkg

# Build
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Tutorial: Publisher-Subscriber

Create `publisher.py`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Lab Exercises

### Lab 1: Build Your First ROS 2 Package

**Objective**: Create functional pub-sub system

**Tasks**:
1. Create ROS 2 workspace
2. Implement publisher node
3. Implement subscriber node
4. Test communication
5. Use ros2 CLI tools for debugging

**Deliverables**:
- Working package with publisher and subscriber
- Screenshot of `ros2 topic echo` output
- Documentation of ROS 2 commands used

**Validation**:
```bash
# Terminal 1
ros2 run my_robot_pkg publisher

# Terminal 2
ros2 topic echo /topic

# Should see messages flowing
```

## Summary

### Key Takeaways
1. **Nodes** are independent processes communicating via DDS
2. **Topics** enable many-to-many async communication
3. **Services** provide request-response patterns
4. **Actions** handle long-running tasks with feedback
5. **Colcon** builds ROS 2 workspaces

### Next Steps
➡️ [Chapter 6: Python Development](./06-python-development.md)
