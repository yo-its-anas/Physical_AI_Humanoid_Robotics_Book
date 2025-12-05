---
id: 18-llm-planning
title: "Chapter 18: LLM Planning"
sidebar_label: "Ch 18: LLM Planning"
sidebar_position: 4
description: "Task decomposition, ROS action mapping"
keywords: [vla, llm, whisper]
tags: [part-v, vla]
---


# Chapter 18: LLM Planning

## Overview
**What You'll Learn**: Task decomposition, ROS action mapping

**Estimated Time**: 7-9 hours

## Core Concepts
LLM Planning enables robots to understand natural language commands and translate them into executable actions.

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

**Next**: [Chapter 19](./19-multimodal.md)
