---
id: 16-vla-intro
title: "Chapter 16: VLA Systems Introduction"
sidebar_label: "Ch 16: VLA Systems Introduction"
sidebar_position: 2
description: "LLMs + robotics, VLA architecture"
keywords: [vla, llm, whisper]
tags: [part-v, vla]
---


# Chapter 16: VLA Systems Introduction

## Overview
**What You'll Learn**: LLMs + robotics, VLA architecture

**Estimated Time**: 7-9 hours

## Core Concepts
VLA Systems Introduction enables robots to understand natural language commands and translate them into executable actions.

### Example: Voice to Action Pipeline
```python
import whisper
import openai

# Speech to text
model = whisper.load_model("base")
result = model.transcribe("audio.wav")
command = result["text"]

# LLM task planning
response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[{
        "role": "system",
        "content": "Decompose robot task into ROS actions"
    }, {
        "role": "user",
        "content": f"Task: {command}"
    }]
)

# Execute ROS actions
plan = response.choices[0].message.content
execute_plan(plan)
```

## Lab
Implement voice command recognition or LLM-based task planner.

## Summary
VLA bridges natural language and robotic control, enabling intuitive human-robot interaction.

**Next**: [Chapter 17](./17-voice-whisper.md)
