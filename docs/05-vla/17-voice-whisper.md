---
id: 17-voice-whisper
title: "Chapter 17: Voice with Whisper"
sidebar_label: "Ch 17: Voice with Whisper"
sidebar_position: 3
description: "Speech recognition, voice commands"
keywords: [vla, llm, whisper]
tags: [part-v, vla]
---


# Chapter 17: Voice with Whisper

## Overview
**What You'll Learn**: Speech recognition, voice commands

**Estimated Time**: 7-9 hours

## Core Concepts
Voice with Whisper enables robots to understand natural language commands and translate them into executable actions.

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

**Next**: [Chapter 18](./18-llm-planning.md)
