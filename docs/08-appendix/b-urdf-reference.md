---
id: b-urdf-reference
title: "Appendix B: URDF/SDF Reference"
sidebar_label: "B: URDF/SDF Reference"
sidebar_position: 3
description: "URDF and SDF XML format reference"
keywords: [urdf, sdf, xml, reference]
tags: [appendix, reference]
---


# Appendix B: URDF/SDF Reference

## URDF Link Elements

### Visual
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
    <!-- OR -->
    <cylinder radius="0.5" length="1"/>
    <!-- OR -->
    <sphere radius="0.5"/>
    <!-- OR -->
    <mesh filename="package://pkg/meshes/model.dae" scale="1 1 1"/>
  </geometry>
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
</visual>
```

### Collision
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
  </geometry>
</collision>
```

### Inertial
```xml
<inertial>
  <mass value="10.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="1.0" ixy="0.0" ixz="0.0"
           iyy="1.0" iyz="0.0"
           izz="1.0"/>
</inertial>
```

## Joint Types

```xml
<!-- Fixed joint (no movement) -->
<joint name="base_to_sensor" type="fixed">
  <parent link="base_link"/>
  <child link="sensor_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Revolute joint (rotation with limits) -->
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.2 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>

<!-- Continuous joint (unlimited rotation) -->
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>

<!-- Prismatic joint (linear sliding) -->
<joint name="gripper_joint" type="prismatic">
  <parent link="hand"/>
  <child link="finger"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.1" effort="10" velocity="0.1"/>
</joint>
```

## Validation Tools

```bash
# Check URDF syntax
check_urdf my_robot.urdf

# Convert URDF to SDF
gz sdf -p my_robot.urdf > my_robot.sdf

# Visualize in RViz
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

## Common Mistakes

1. **Missing `<robot>` tag**: URDF must have root `<robot name="...">` element
2. **Disconnected links**: All links must connect to base_link via joint tree
3. **Zero mass**: Inertial mass must be > 0 for simulation
4. **Missing collision geometry**: Needed for physics simulation
5. **Incorrect inertia tensors**: Use tools like MeshLab to calculate

## Resources
- [URDF Specification](http://wiki.ros.org/urdf/XML)
- [SDF Format](http://sdformat.org/)
