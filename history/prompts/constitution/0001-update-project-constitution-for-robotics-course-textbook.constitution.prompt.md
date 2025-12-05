---
id: 0001
title: Update Project Constitution for Robotics Course Textbook
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: anaslinux
command: /sp.constitution
labels: ["constitution", "project setup", "robotics", "ai-native book"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:
- none
---

## Prompt

Create a project constitution for the unified AI-native book project titled:
“Textbook for Teaching **Physical AI & Humanoid Robotics Course.**”

Purpose of the Book:
Create an end-to-end textbook that teaches Physical AI, embodied intelligence, humanoid robot systems, and VLA-driven control pipelines—presented through an AI-native workflow using Claude Code and Spec-Kit Plus.

Target Audience:
University-level students, robotics instructors, AI engineers, and advanced learners of ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action robotics.

Tone & Writing Style:
- Clear, technical, and academically rigorous
- Highly structured explanations
- Extensive examples, diagrams (text-based), and code blocks
- Real-world, industry-aligned robotics practices

Structure Rules:
- Each chapter must have: overview → background → core concepts → implementation → labs/exercises → summary
- All chapters output in Markdown
- Docusaurus-compatible structure
- One folder per module with sub-chapters
- Code formatted in fenced codeblocks with language tags (python, xml, bash, etc.)

Tools & Technology Requirements:
- Claude Code for file operations and coding
- Spec-Kit Plus for full project specification, planning, tasking, and execution
- Docusaurus for static book site
- GitHub Pages for hosting
- Git + GitHub repository integration
- Markdown-based content output

Tools:
- Use Claude Code filesystem MCP for all file generations.
- Use NPM/terminal MCP for Docusaurus commands.

Business Requirements:
The book must be created entirely using Claude Code and Spec-Kit Plus.
It must be published as a fully functional Docusaurus website and deployed to GitHub Pages.

References & Source Material (Provided Course Content):
- Physical AI & Humanoid Robotics course overview
- All modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)
- Hardware requirements: Jetson kits, RealSense, IMUs, Unitree robots
- Weekly learning outcomes
- Capstone project description
- Lab infrastructure descriptions (Sim Rig, Edge Kit, Robot Lab)

Quality Standards:
- Every chapter must be technically accurate, reflecting real ROS 2, Gazebo, Isaac Sim, and VLA workflows.
- Code must be runnable or logically consistent.
- Terminology must align with robotics industry standards.
- Clear step-by-step explanations for students.
- Cross-referencing between modules is required.

This constitution defines the rules governing all subsequent stages.

## Response snapshot

The project constitution for the "Physical AI & Humanoid Robotics Course Textbook Project" has been successfully created and saved to `.specify/memory/constitution.md`. The constitution outlines core principles, development tools, project requirements, and governance.

## Outcome

- ✅ Impact: Project constitution file created, establishing core principles and requirements for the AI-native book project.
- 🧪 Tests: none
- 📁 Files: One file modified: `.specify/memory/constitution.md`
- 🔁 Next prompts: Consider running `/sp.specify` next to create the feature specification.
- 🧠 Reflection: The process involved reading the template, extracting detailed requirements from user input, populating placeholders, and performing a consistency check on related templates.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Ensure future constitution updates handle partial input and existing principles more dynamically.
