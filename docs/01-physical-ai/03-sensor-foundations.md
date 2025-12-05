---
id: 03-sensor-foundations
title: "Chapter 3: Sensor Foundations"
sidebar_label: "Ch 3: Sensor Foundations"
sidebar_position: 4
description: "Comprehensive overview of robotic sensors for perception"
keywords: [sensors, perception, cameras, lidar, imu]
tags: [part-i, sensors]
---


# Chapter 3: Sensor Foundations

## Overview

### What You'll Learn
- **Classify** different sensor modalities and their use cases
- **Explain** multimodal sensor fusion strategies
- **Analyze** sensor characteristics (range, accuracy, latency)
- **Design** sensor suites for humanoid robots

**Estimated Time**: 6-7 hours

## Background

Robots perceive the world through sensors. Unlike humans with integrated eyes/ears/touch, robots must combine multiple sensor modalities to build world understanding.

## Core Concepts

### Concept 1: Vision Sensors

**RGB Cameras**:
- Standard cameras (like smartphone cameras)
- Provide color images for object recognition
- Limitations: No depth information
- Use: Object detection, scene understanding

**Depth Cameras**:
- Intel RealSense, Azure Kinect, Stereolabs ZED
- Provide distance to each pixel
- Technologies: Stereo vision, structured light, ToF
- Use: Obstacle avoidance, 3D mapping

**Event Cameras**:
- Output changes in brightness (not full frames)
- Extremely high temporal resolution (microseconds)
- Low latency, high dynamic range
- Use: High-speed motion tracking

### Concept 2: Motion and Position Sensors

**IMU (Inertial Measurement Unit)**:
- Combines accelerometer + gyroscope (+ often magnetometer)
- Measures linear acceleration and angular velocity
- Critical for balance and stabilization
- Example: MPU6050, BMI088

**Encoders**:
- Measure joint positions/velocities
- Types: Absolute vs. incremental
- Essential for motor control
- Resolution: 1024-4096 counts per revolution typical

### Concept 3: Environmental Sensors

**LiDAR (Light Detection and Ranging)**:
- Laser-based distance measurement
- 2D (planar scan) or 3D (spinning/solid-state)
- Range: 0.1m to 100m+ depending on model
- Use: SLAM, navigation, obstacle detection

**Ultrasonic**:
- Sound-based distance (like bat echolocation)
- Short range (few meters), cheap
- Wide beam angle
- Use: Proximity detection, cliff detection

**Force/Torque Sensors**:
- Measure forces at robot's end-effector
- Essential for manipulation tasks
- Enable compliant control (react to contact)
- Example: ATI Mini45, OnRobot HEX-E

### Concept 4: Multimodal Fusion

Combining multiple sensors improves robustness:

```python
class SensorFusion:
    def __init__(self):
        self.camera = RGBDCamera()
        self.imu = IMU()
        self.lidar = LiDAR()
        
    def get_pose_estimate(self):
        # Kalman filter combining IMU + vision + lidar
        imu_pose = self.imu.get_orientation()
        visual_pose = self.camera.visual_odometry()
        lidar_pose = self.lidar.scan_matching()
        
        # Weighted fusion based on confidence
        fused_pose = self.kalman_filter(
            [imu_pose, visual_pose, lidar_pose],
            weights=[0.3, 0.5, 0.2]
        )
        return fused_pose
```

## Implementation

### Setup: Intel RealSense D435i

```bash
# Install RealSense SDK
sudo apt-get install ros-humble-realsense2-camera
sudo apt-get install ros-humble-realsense2-description

# Launch camera
ros2 launch realsense2_camera rs_launch.py
```

### Tutorial: Reading Sensor Data

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2

class SensorReader(Node):
    def __init__(self):
        super().__init__('sensor_reader')
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_callback, 10
        )
        
        # Subscribe to IMU
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data',
            self.imu_callback, 10
        )
        
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Process image
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)
        
    def imu_callback(self, msg):
        # Extract orientation
        self.get_logger().info(
            f'IMU Orient: x={msg.orientation.x:.2f} '
            f'y={msg.orientation.y:.2f} z={msg.orientation.z:.2f}'
        )
```

## Lab Exercises

### Lab 1: Sensor Characterization

**Objective**: Measure and document sensor characteristics

**Tasks**:
1. Measure camera frame rate and resolution
2. Test depth sensor accuracy at different ranges
3. Measure IMU drift over time
4. Document noise characteristics

**Deliverables**:
- Sensor specification table
- Accuracy plots (distance vs. error)
- Noise analysis graphs

## Summary

### Key Takeaways
1. **Vision** (RGB, depth) provides rich semantic information but computationally expensive
2. **IMU** essential for real-time orientation and stabilization
3. **LiDAR** provides accurate ranging for mapping and navigation
4. **Sensor fusion** combines strengths while mitigating individual weaknesses

### Next Steps
➡️ [Chapter 4: Course Overview](./04-weekly-overview.md)
