<!-- Sync Impact Report:
Version change: 0.0.0 → 1.0.0
Modified principles: None (initial creation)
Added sections: All (initial creation)
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
- .claude/commands/sp.adr.md: ✅ updated
- .claude/commands/sp.analyze.md: ✅ updated
- .claude/commands/sp.checklist.md: ✅ updated
- .claude/commands/sp.clarify.md: ✅ updated
- .claude/commands/sp.constitution.md: ✅ updated
- .claude/commands/sp.git.commit_pr.md: ✅ updated
- .claude/commands/sp.implement.md: ✅ updated
- .claude/commands/sp.phr.md: ✅ updated
- .claude/commands/sp.plan.md: ✅ updated
- .claude/commands/sp.specify.md: ✅ updated
- .claude/commands/sp.tasks.md: ✅ updated
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Course Textbook Project Constitution

## Core Principles

### I. Technical Accuracy & Rigor
Every chapter must be technically accurate, reflecting real ROS 2, Gazebo, Isaac Sim, and VLA workflows. Terminology must align with robotics industry standards. Content must be clear, technical, and academically rigorous.

### II. Structured & Comprehensive Content
Each chapter must follow the structure: overview → background → core concepts → implementation → labs/exercises → summary. Content should be highly structured with extensive examples, text-based diagrams, and code blocks. Cross-referencing between modules is required.

### III. AI-Native Development Workflow
The book must be created entirely using Claude Code and Spec-Kit Plus. Claude Code filesystem MCP for all file generations.

### IV. Docusaurus & GitHub Pages Publication
All chapters output in Markdown, using a Docusaurus-compatible structure with one folder per module and sub-chapters. The book must be published as a fully functional Docusaurus website and deployed to GitHub Pages, integrated with Git and GitHub repository. NPM/terminal MCP for Docusaurus commands.

### V. Code Quality & Runnability
Code formatted in fenced codeblocks with language tags (python, xml, bash, etc.). Code must be runnable or logically consistent.

### VI. Real-world Relevance
Content should reflect real-world, industry-aligned robotics practices.

## Development Tools & Technologies

- Claude Code for file operations and coding.
- Spec-Kit Plus for full project specification, planning, tasking, and execution.
- Docusaurus for static book site.
- GitHub Pages for hosting.
- Git + GitHub repository integration.
- Markdown-based content output.
- NPM/terminal MCP for Docusaurus commands.

## Project Requirements

**Purpose:** Create an end-to-end textbook that teaches Physical AI, embodied intelligence, humanoid robot systems, and VLA-driven control pipelines—presented through an AI-native workflow using Claude Code and Spec-Kit Plus.

**Target Audience:** University-level students, robotics instructors, AI engineers, and advanced learners of ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action robotics.

**References & Source Material:** Physical AI & Humanoid Robotics course overview, all modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA), hardware requirements (Jetson kits, RealSense, IMUs, Unitree robots), weekly learning outcomes, capstone project description, lab infrastructure descriptions (Sim Rig, Edge Kit, Robot Lab).

## Governance

This constitution defines the rules governing all subsequent stages. Amendments require documentation, approval, and a migration plan. All changes to the book project must comply with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05