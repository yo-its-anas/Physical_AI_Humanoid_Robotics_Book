---
id: 27-evaluation
title: "Chapter 27: Evaluation & Extensions"
sidebar_label: "Ch 27: Evaluation & Extensions"
sidebar_position: 5
description: "Testing, scoring, future directions"
keywords: [capstone, integration]
tags: [part-vii, capstone]
---


# Chapter 27: Evaluation & Extensions

## Overview
**What You'll Learn**: Testing, scoring, future directions

**Estimated Time**: 7-9 hours

## Core Concepts
Evaluation & Extensions demonstrates the culmination of Physical AI skills: integrating vision, language, and action into a cohesive system.

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

**Next**: [Appendices](../08-appendix/index.md)
