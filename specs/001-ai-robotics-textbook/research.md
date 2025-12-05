# Research: AI-Native Robotics Textbook

**Feature**: 001-ai-robotics-textbook
**Date**: 2025-12-05
**Phase**: 0 - Technology Research & Decisions

## Overview

This document consolidates research findings and technology decisions for building a comprehensive Physical AI & Humanoid Robotics textbook using Docusaurus and deploying to GitHub Pages.

## Technology Stack Research

### 1. Static Site Generator: Docusaurus

**Decision**: Use Docusaurus 3.x for the textbook site

**Rationale**:
- **Documentation-focused**: Built specifically for technical documentation and educational content
- **React-based**: Modern, performant, with excellent developer experience
- **MDX support**: Allows embedding interactive React components in Markdown
- **Versioning**: Built-in support for multiple versions (useful for course iterations)
- **Search**: Integrated Algolia DocSearch and local search plugins
- **Sidebar navigation**: Automatic sidebar generation from folder structure
- **Mobile-responsive**: Out-of-the-box mobile support
- **GitHub Pages integration**: First-class deployment support
- **Active ecosystem**: Large community, extensive plugins, well-maintained

**Alternatives Considered**:
- **MkDocs**: Python-based, simpler but less flexible for interactive content
- **Jekyll**: GitHub's default, but older architecture and limited interactivity
- **VuePress**: Vue-based alternative, smaller ecosystem than Docusaurus
- **GitBook**: Commercial platform, less control over deployment

**Why Docusaurus Chosen**: Best balance of educational content features, modern tech stack, and deployment simplicity. MDX support enables future interactive robotics visualizations.

---

### 2. Content Format: Markdown + MDX

**Decision**: Use Markdown for all chapter content with MDX for interactive elements

**Rationale**:
- **Simplicity**: Markdown is easy to write, read, and maintain
- **Portability**: Standard format, can be converted to other formats (PDF, EPUB)
- **Version control**: Plain text works perfectly with Git
- **Code blocks**: Native support for syntax highlighting
- **Extensibility**: MDX allows embedding React components when needed
- **Tooling**: Excellent editor support (VS Code, etc.)

**Content Structure Standards**:
```markdown
---
id: chapter-id
title: Chapter Title
sidebar_label: Short Title
sidebar_position: 1
---

# Chapter Title

## Overview
[Learning objectives, prerequisites]

## Background
[Contextual knowledge]

## Core Concepts
[Theory and principles]

## Implementation
[Step-by-step tutorials, code examples]

## Lab Exercises
[Hands-on practice activities]

## Summary
[Key takeaways, next steps]
```

---

### 3. Code Examples: Multi-language Support

**Decision**: Fenced code blocks with language tags, organized by technology

**Languages Used**:
- **Python**: ROS 2 nodes, Isaac Sim scripts, VLA integrations
- **XML**: URDF robot descriptions, ROS launch files
- **YAML**: ROS 2 parameters, configuration files
- **Bash**: Installation scripts, environment setup
- **JSON**: Configuration files, API contracts
- **C++**: Optional advanced ROS 2 examples

**Code Organization**:
```text
static/code-examples/
├── ros2/
│   ├── nodes/              # Python rclpy nodes
│   ├── launch/             # Launch files
│   ├── urdf/               # Robot descriptions
│   └── params/             # Parameter files
├── gazebo/
│   ├── worlds/             # SDF world files
│   └── plugins/            # Custom plugins
├── isaac/
│   ├── sim/                # Isaac Sim scripts
│   └── ros/                # Isaac ROS examples
└── vla/
    ├── whisper/            # Speech recognition
    └── llm/                # LLM integration
```

**Code Quality Standards**:
- All examples must be independently runnable or clearly marked as conceptual
- Include comments explaining key concepts
- Provide setup/dependency instructions
- Test examples before publication
- Use consistent naming conventions

---

### 4. Deployment: GitHub Pages + GitHub Actions

**Decision**: Automated deployment via GitHub Actions to GitHub Pages

**Rationale**:
- **Free hosting**: GitHub Pages is free for public repositories
- **Custom domain**: Supports custom domains if needed
- **HTTPS**: Automatic HTTPS support
- **CI/CD**: GitHub Actions provides automated build and deploy
- **Version control**: All content changes tracked in Git
- **Collaboration**: Easy for contributors to submit PRs

**Deployment Workflow**:
```yaml
# .github/workflows/deploy.yml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  pull_request:

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm
      - run: npm ci
      - run: npm run build
      - uses: peaceiris/actions-gh-pages@v3
        if: github.ref == 'refs/heads/main'
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

**Alternatives Considered**:
- **Vercel**: Requires external account, overkill for static docs
- **Netlify**: Similar to Vercel, unnecessary for simple deployment
- **Read the Docs**: Python-focused, less flexible for React components
- **Self-hosted**: Requires server maintenance

---

### 5. Content Pipeline: AI-Native Workflow

**Decision**: Claude Code + Spec-Kit Plus for all content generation

**Workflow**:
1. **Specification** (`/sp.specify`): Define chapter requirements
2. **Planning** (`/sp.plan`): Architecture and structure decisions
3. **Tasking** (`/sp.tasks`): Break down into actionable tasks
4. **Implementation** (`/sp.implement`): Generate chapter content
5. **Review** (`/sp.analyze`): Quality and consistency checks
6. **Commit & PR** (`/sp.git.commit_pr`): Version control integration

**Quality Controls**:
- Prompt History Records (PHR) for all major changes
- Architecture Decision Records (ADR) for significant decisions
- Constitution compliance checks throughout
- Manual review for technical accuracy
- Link validation before deployment

---

### 6. Educational Content Best Practices

**Research Findings**:

**Effective Technical Textbook Structure**:
- **Progressive complexity**: Start simple, build gradually
- **Learning objectives**: Clear goals for each chapter
- **Prerequisites**: Explicit knowledge requirements
- **Visual aids**: Diagrams, architecture illustrations
- **Hands-on practice**: Labs immediately after theory
- **Review questions**: Reinforce understanding
- **Real-world examples**: Industry-relevant scenarios

**Robotics Education Standards**:
- **Theory-to-practice**: Balance conceptual and applied content
- **Safety considerations**: Highlight hardware safety
- **Troubleshooting guides**: Common errors and solutions
- **Version specificity**: Specify software versions (ROS 2 Humble, etc.)
- **Cross-platform notes**: Linux/Windows/macOS differences
- **Resource links**: Official documentation, community resources

**Accessibility**:
- **Clear language**: Avoid unnecessary jargon
- **Consistent terminology**: Define terms, use consistently
- **Code comments**: Explain non-obvious code
- **Alt text**: Describe diagrams for screen readers
- **Responsive design**: Mobile-friendly layout

---

### 7. ROS 2 Content Guidelines

**Decision**: Focus on ROS 2 Humble (LTS) with notes on Iron/Rolling

**Key Topics**:
- **Installation**: Ubuntu 22.04 recommended, Docker alternative
- **Workspace setup**: colcon build system
- **Python rclpy**: Primary language for course
- **Launch system**: Python launch files (preferred over XML)
- **Communication**: Topics, services, actions
- **Tools**: ros2 CLI, rviz2, rqt
- **Best practices**: Node design patterns, parameter management

**Code Example Template**:
```python
#!/usr/bin/env python3
"""
Brief description of what this node does.

ROS 2 Version: Humble
Dependencies: rclpy, std_msgs
"""

import rclpy
from rclpy.node import Node
# ... imports

class ExampleNode(Node):
    """Descriptive class docstring."""

    def __init__(self):
        super().__init__('example_node')
        # ... initialization

    # ... methods

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### 8. Simulation Environment Guidelines

**Gazebo**:
- **Version**: Gazebo Classic 11 + Gazebo Fortress/Garden
- **Integration**: gazebo_ros_pkgs for ROS 2 bridge
- **Focus**: Physics simulation, sensor simulation
- **Examples**: URDF/SDF loading, plugin development

**Unity**:
- **Version**: Unity 2021.3 LTS or later
- **ROS integration**: Unity Robotics Hub packages
- **Focus**: High-fidelity visualization, human-robot interaction
- **Examples**: URDF Importer, TCP endpoint communication

**NVIDIA Isaac Sim**:
- **Version**: Isaac Sim 2023.1 or later (Omniverse)
- **Hardware**: RTX GPU required
- **Focus**: Synthetic data generation, AI model training
- **Integration**: Isaac ROS packages, Nav2
- **Examples**: USD scene creation, VSLAM, object detection

---

### 9. Hardware Lab Documentation

**Simulation Workstation Specs**:
- **CPU**: Intel i7/i9 or AMD Ryzen 7/9 (8+ cores)
- **GPU**: NVIDIA RTX 3060 or better (12GB+ VRAM for Isaac Sim)
- **RAM**: 32GB minimum, 64GB recommended
- **Storage**: 1TB NVMe SSD
- **OS**: Ubuntu 22.04 LTS

**Edge Computing (Jetson)**:
- **Recommended**: Jetson Orin Nano/NX Developer Kit
- **Sensors**: RealSense D435i, IMU (BNO055/ICM-20948)
- **Connectivity**: WiFi 6, Gigabit Ethernet
- **Power**: 5V 4A minimum

**Robot Platforms**:
- **Entry**: Hiwonder mini-humanoids (~$2K-5K)
- **Mid-range**: Robotis OP3 (~$10K-15K)
- **Advanced**: Unitree G1 (~$16K), Unitree Go2 (quadruped proxy)

---

### 10. Vision-Language-Action (VLA) Integration

**Speech Recognition**:
- **Primary**: OpenAI Whisper (open-source)
- **Deployment**: Local inference on Jetson or cloud API
- **Languages**: Multi-language support

**LLM Integration**:
- **Options**: OpenAI API, Anthropic Claude, local LLMs (Llama, Mistral)
- **Use cases**: Task decomposition, natural language commands
- **Safety**: Prompt engineering for robotics constraints
- **ROS 2 bridge**: Text → structured actions

**Multimodal Fusion**:
- **Vision**: Object detection, pose estimation
- **Language**: Natural language understanding
- **Action**: ROS 2 action servers for robot control
- **Architecture**: Perception → Planning → Execution pipeline

---

## Resolved Clarifications

All technical context items from the plan template have been resolved:

1. **Language/Version**: Markdown/MDX for content, Node.js 18+ for Docusaurus ✅
2. **Primary Dependencies**: Docusaurus 3.x, React, MDX ✅
3. **Storage**: Git repository, GitHub Pages deployment ✅
4. **Testing**: Link validation, code syntax checking, manual QA ✅
5. **Target Platform**: Web browsers (static site) ✅
6. **Performance Goals**: <2s page load, responsive navigation ✅
7. **Constraints**: Static site, GitHub Pages limits, Markdown format ✅
8. **Scale/Scope**: 27 chapters + preface + appendices, 100+ code examples ✅

## Next Steps

Proceed to **Phase 1: Design & Contracts** to define:
1. **data-model.md**: Content entities and relationships
2. **contracts/**: Docusaurus configuration, chapter templates, sidebar structure
3. **quickstart.md**: Getting started guide for contributors
4. **Agent context update**: Add new technologies to Claude context

## References

- [Docusaurus Documentation](https://docusaurus.io/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/)
- [Gazebo Documentation](https://gazebosim.org/)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
