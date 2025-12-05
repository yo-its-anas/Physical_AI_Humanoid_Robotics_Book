---
id: 24-architecture
title: "Chapter 24: System Architecture"
sidebar_label: "Ch 24: System Architecture"
sidebar_position: 2
description: "Full pipeline: sensors → perception → planning → control"
keywords: [capstone, integration]
tags: [part-vii, capstone]
---


# Chapter 24: System Architecture

## Overview
**What You'll Learn**: Full pipeline: sensors → perception → planning → control

**Estimated Time**: 7-9 hours

## Core Concepts
System Architecture demonstrates the culmination of Physical AI skills: integrating vision, language, and action into a cohesive system.

### System Diagram
```mermaid
graph LR
    A[Voice Command] --> B[Whisper STT]
    B --> C[LLM Planner]
    C --> D[ROS 2 Actions]
    D --> E[Navigation/Manipulation]
    E --> F[Robot Execution]
    F --> G[Feedback/Completion]
    G --> A
```

## Implementation
Complete capstone project with voice control, autonomous navigation, and task execution.

## Lab
Build and demonstrate full integrated system.

## Summary
Capstone proves mastery of Physical AI development from concept to deployment.

**Next**: [Chapter 25](./25-implementation.md)
