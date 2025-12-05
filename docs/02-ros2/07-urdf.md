---
id: 07-urdf
title: "Chapter 7: Robot Description with URDF"
sidebar_label: "Ch 7: URDF Robot Description"
sidebar_position: 4
description: "Creating robot models with URDF, links, joints, and visualization"
keywords: [urdf, robot modeling, sdf, xacro]
tags: [part-ii, urdf]
---


# Chapter 7: Robot Description with URDF

## Overview

**What You'll Learn**: URDF structure, building humanoid models, links/joints, visual/collision meshes, RViz visualization

**Estimated Time**: 7-9 hours

## Core Concepts

### URDF (Unified Robot Description Format)

XML format describing robot kinematics and dynamics:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.2 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

### Joint Types

- **fixed**: No movement
- **revolute**: Rotation with limits
- **continuous**: Unlimited rotation
- **prismatic**: Linear sliding
- **planar**: 2D plane movement
- **floating**: 6 DOF (unconstrained)

### Lab: Build Humanoid URDF

Create simple humanoid with torso, legs, arms:
- Base link (torso)
- Hip joints (revolute, 3 DOF)
- Knee joints (revolute, 1 DOF)
- Ankle joints (revolute, 2 DOF)
- Arms similar structure

Validate with: `check_urdf my_robot.urdf`
Visualize in RViz: `ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf`

## Summary

**Key Takeaways**:
1. URDF describes robot as tree of links and joints
2. Each link has visual, collision, and inertial properties
3. Joints define kinematic relationships and constraints
4. RViz and Gazebo use URDF for visualization and simulation

**Next**: [Chapter 8: Control Systems](./08-control-systems.md)
