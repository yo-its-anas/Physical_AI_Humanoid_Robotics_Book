---
id: 26-sim-to-real
title: "Chapter 26: Sim-to-Real Transfer"
sidebar_label: "Ch 26: Sim-to-Real Transfer"
sidebar_position: 4
description: "Isaac Sim → Real hardware, domain randomization"
keywords: [capstone, integration]
tags: [part-vii, capstone]
---


# Chapter 26: Sim-to-Real Transfer

## Overview
**What You'll Learn**: Isaac Sim → Real hardware, domain randomization

**Estimated Time**: 7-9 hours

## Core Concepts
Sim-to-Real Transfer demonstrates the culmination of Physical AI skills: integrating vision, language, and action into a cohesive system.

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

**Next**: [Chapter 27](./27-evaluation.md)
