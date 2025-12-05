---
id: 19-multimodal
title: "Chapter 19: Multimodal Interaction"
sidebar_label: "Ch 19: Multimodal Interaction"
sidebar_position: 5
description: "Vision + speech + gesture fusion"
keywords: [vla, llm, whisper]
tags: [part-v, vla]
---


# Chapter 19: Multimodal Interaction

## Overview
**What You'll Learn**: Vision + speech + gesture fusion

**Estimated Time**: 7-9 hours

## Core Concepts
Multimodal Interaction enables robots to understand natural language commands and translate them into executable actions.

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

**Next**: [Part VI: Hardware Lab](../06-hardware-lab/index.md)
