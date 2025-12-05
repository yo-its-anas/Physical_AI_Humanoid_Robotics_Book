# Data Model: AI-Native Robotics Textbook

**Feature**: 001-ai-robotics-textbook
**Date**: 2025-12-05
**Phase**: 1 - Design & Contracts

## Overview

This document defines the content entities, their relationships, and validation rules for the Physical AI & Humanoid Robotics textbook. Unlike a traditional data model with database schemas, this model describes the structure of educational content and metadata.

## Core Entities

### 1. Part

A major section of the textbook containing related chapters.

**Fields**:
- `id`: Unique identifier (e.g., `01-physical-ai`)
- `title`: Full title (e.g., "Foundations of Physical AI & Embodied Intelligence")
- `part_number`: Roman numeral (I through VIII)
- `description`: Brief overview of the part's content
- `chapters`: Array of Chapter entities
- `sidebar_position`: Order in navigation (1-8)

**Validation Rules**:
- `id` must be unique across all parts
- `part_number` must be valid Roman numeral I-VIII
- Must contain at least 1 chapter
- `sidebar_position` determines display order

**Example**:
```yaml
id: 01-physical-ai
title: "Foundations of Physical AI & Embodied Intelligence"
part_number: I
description: "Introduction to physical AI, humanoid robotics landscape, sensor foundations, and course overview"
chapters: [ch01, ch02, ch03, ch04]
sidebar_position: 1
```

---

### 2. Chapter

A single learning unit with consistent structure.

**Fields**:
- `id`: Unique identifier (e.g., `ch01-introduction`)
- `chapter_number`: Sequential number (1-27)
- `title`: Full chapter title
- `sidebar_label`: Short title for navigation
- `part_id`: Reference to parent Part
- `file_path`: Markdown file location (e.g., `docs/01-physical-ai/01-introduction.md`)
- `learning_objectives`: Array of learning goals
- `prerequisites`: Array of prerequisite knowledge/chapters
- `estimated_time`: Reading/lab time estimate (e.g., "2-3 hours")
- `sections`: Array of Section entities
- `code_examples`: Array of CodeExample references
- `lab_exercises`: Array of LabExercise references
- `sidebar_position`: Order within part

**Validation Rules**:
- `chapter_number` must be unique (1-27)
- Must contain all required sections: Overview, Background, Core Concepts, Implementation, Lab Exercises, Summary
- `part_id` must reference a valid Part
- `file_path` must exist in repository
- At least 1 learning objective required

**Required Section Structure**:
1. Overview (learning objectives, prerequisites)
2. Background (contextual knowledge)
3. Core Concepts (theory and principles)
4. Implementation (tutorials, code examples)
5. Lab Exercises (hands-on practice)
6. Summary (key takeaways, next steps)

**Example**:
```yaml
id: ch01-introduction
chapter_number: 1
title: "Introduction to Physical AI"
sidebar_label: "Introduction"
part_id: 01-physical-ai
file_path: docs/01-physical-ai/01-introduction.md
learning_objectives:
  - "Understand the concept of Physical AI and embodied intelligence"
  - "Differentiate between digital AI and Physical AI"
  - "Explain why AI systems need to understand physics"
prerequisites:
  - "Basic understanding of artificial intelligence"
  - "Familiarity with robotics concepts (helpful but not required)"
estimated_time: "1-2 hours"
sidebar_position: 1
```

---

### 3. Section

A subdivision of a chapter with specific purpose.

**Fields**:
- `id`: Unique identifier within chapter
- `title`: Section heading
- `type`: Enumerated type (overview | background | concepts | implementation | labs | summary)
- `content`: Markdown content
- `subsections`: Array of nested sections (optional)
- `order`: Display order within chapter

**Validation Rules**:
- `type` must be one of the allowed section types
- `order` determines sequence within chapter
- Implementation sections must include code examples or step-by-step instructions

**Section Types**:
- **overview**: Learning objectives, prerequisites, chapter roadmap
- **background**: Historical context, foundational knowledge
- **concepts**: Theoretical principles, definitions, explanations
- **implementation**: Step-by-step tutorials, code walkthroughs
- **labs**: Hands-on exercises, practice problems
- **summary**: Key takeaways, further reading, next chapter preview

---

### 4. CodeExample

A runnable or illustrative code snippet.

**Fields**:
- `id`: Unique identifier
- `title`: Descriptive title
- `language`: Programming language (python | xml | yaml | bash | json | cpp)
- `code`: Source code content
- `file_path`: Location in `static/code-examples/` (if downloadable)
- `description`: What the code demonstrates
- `dependencies`: Required packages/libraries
- `execution_environment`: Where to run (local | docker | jetson | isaac-sim)
- `is_runnable`: Boolean indicating if independently executable
- `expected_output`: Description of expected results
- `related_chapters`: Array of chapter IDs

**Validation Rules**:
- `language` must be one of supported languages
- If `is_runnable` is true, must include `dependencies` and `execution_environment`
- Code must be syntactically valid for the specified language
- Must include descriptive comments

**Example**:
```yaml
id: ros2-minimal-publisher
title: "ROS 2 Minimal Publisher Node"
language: python
file_path: static/code-examples/ros2/nodes/minimal_publisher.py
description: "Basic ROS 2 publisher that sends string messages to a topic"
dependencies:
  - rclpy
  - std_msgs
execution_environment: local
is_runnable: true
expected_output: "Publishes 'Hello World: N' messages at 1 Hz to /topic"
related_chapters: [ch06-python-development]
```

---

### 5. LabExercise

A hands-on practice activity for students.

**Fields**:
- `id`: Unique identifier
- `title`: Exercise name
- `chapter_id`: Parent chapter
- `difficulty`: Enumerated (beginner | intermediate | advanced)
- `estimated_time`: Completion time estimate
- `objectives`: Array of specific learning goals
- `setup_requirements`: Hardware/software needed
- `instructions`: Step-by-step procedure
- `deliverables`: What students must produce
- `validation_criteria`: How to verify completion
- `hints`: Optional guidance (array)
- `solution_available`: Boolean

**Validation Rules**:
- `difficulty` must match chapter progression
- `setup_requirements` must be achievable given course resources
- `deliverables` must be specific and measurable
- `validation_criteria` must be objective

**Example**:
```yaml
id: lab-create-urdf
title: "Build Your First Humanoid URDF"
chapter_id: ch07-urdf
difficulty: intermediate
estimated_time: "2-3 hours"
objectives:
  - "Create a multi-link humanoid URDF file"
  - "Define joints and link transformations"
  - "Visualize the robot in RViz"
setup_requirements:
  - "ROS 2 Humble installed"
  - "rviz2 package"
  - "Text editor"
deliverables:
  - "humanoid.urdf file with at least 10 links"
  - "Screenshot of robot in RViz"
validation_criteria:
  - "URDF passes `check_urdf` validation"
  - "All joints have proper limits"
  - "Robot loads without errors in RViz"
hints:
  - "Start with the torso as the base link"
  - "Use continuous joints for wheels, revolute for humanoid joints"
solution_available: true
```

---

### 6. Diagram

A visual representation of concepts.

**Fields**:
- `id`: Unique identifier
- `title`: Diagram caption
- `type`: Enumerated (architecture | flowchart | sequence | state-machine | concept-map)
- `format`: File format (svg | png | mermaid | ascii)
- `file_path`: Location in `static/img/` or inline Mermaid code
- `alt_text`: Accessibility description
- `related_chapters`: Array of chapter IDs
- `description`: Detailed explanation

**Validation Rules**:
- Must include `alt_text` for accessibility
- SVG preferred over raster formats for scalability
- Mermaid diagrams embedded directly in Markdown
- All diagrams must be referenced in chapter text

**Example**:
```yaml
id: ros2-architecture
title: "ROS 2 System Architecture"
type: architecture
format: mermaid
alt_text: "Diagram showing ROS 2 nodes communicating via DDS middleware with topics, services, and actions"
related_chapters: [ch05-fundamentals]
description: "High-level overview of ROS 2 communication patterns"
```

---

### 7. Appendix

Reference material and supplementary content.

**Fields**:
- `id`: Unique identifier (e.g., `appendix-a`)
- `letter`: Appendix letter (A-E)
- `title`: Full title
- `file_path`: Markdown file location
- `content_type`: Enumerated (cheatsheet | reference | troubleshooting | checklist | guide)
- `sidebar_position`: Order in appendix section

**Validation Rules**:
- `letter` must be unique (A-E)
- `content_type` determines format and structure
- Must be reference material, not core curriculum

**Example**:
```yaml
id: appendix-a
letter: A
title: "Complete ROS 2 Cheat Sheets"
file_path: docs/08-appendix/a-ros2-cheatsheet.md
content_type: cheatsheet
sidebar_position: 1
```

---

### 8. Preface

Introductory material before main content.

**Fields**:
- `id`: Unique identifier
- `title`: Section title
- `file_path`: Markdown file location
- `topics_covered`: Array of topics
- `sidebar_position`: Order in preface

**Sections**:
1. What This Textbook Is
2. How This Book Was Created (Claude Code + Spec-Kit Plus)
3. Who This Book Is For
4. How to Use This Book
5. Software + Hardware Requirements Overview

---

## Entity Relationships

```
Textbook
├── Preface (1:N)
│   └── Sections
├── Parts (1:8)
│   └── Chapters (1:N)
│       ├── Sections (1:6 required)
│       ├── CodeExamples (0:N)
│       ├── LabExercises (0:N)
│       └── Diagrams (0:N)
└── Appendices (1:5)
```

**Relationship Rules**:
- Each Chapter belongs to exactly one Part
- Each CodeExample can be referenced by multiple Chapters
- Each LabExercise belongs to exactly one Chapter
- Each Diagram can be shared across multiple Chapters
- Parts must be ordered sequentially (I-VIII)
- Chapters must be numbered sequentially (1-27)

---

## Metadata Schema

### Frontmatter (YAML)

Every Markdown file includes frontmatter:

```yaml
---
id: unique-identifier
title: "Full Title"
sidebar_label: "Short Nav Label"
sidebar_position: 1
description: "Brief summary for SEO"
keywords:
  - keyword1
  - keyword2
tags:
  - category
---
```

**Required Fields**:
- `id`: Unique across all documents
- `title`: Display title
- `sidebar_label`: Navigation label (keep short)
- `sidebar_position`: Ordering

**Optional Fields**:
- `description`: SEO meta description
- `keywords`: Search keywords
- `tags`: Categorization
- `draft`: Boolean, hides from production if true
- `last_updated`: Timestamp of last modification

---

## Content Validation Rules

### Chapter Content

1. **Length**: 2000-5000 words per chapter (excluding code)
2. **Code Examples**: Minimum 3 per implementation chapter
3. **Lab Exercises**: Minimum 1 per chapter (except appendices)
4. **Cross-References**: Link to prerequisite chapters
5. **Learning Objectives**: 3-5 specific, measurable goals
6. **Summary**: Recap key points, preview next chapter

### Code Quality

1. **Syntax**: Must be valid for specified language
2. **Comments**: Explain non-obvious logic
3. **Naming**: Follow language conventions (PEP 8 for Python, etc.)
4. **Dependencies**: Explicitly listed
5. **Testing**: Runnable examples must be tested before publication

### Accessibility

1. **Alt Text**: All images and diagrams
2. **Headings**: Proper hierarchy (H1 → H2 → H3, no skipping)
3. **Link Text**: Descriptive, not "click here"
4. **Code Comments**: Explain for all skill levels

---

## State Transitions

### Chapter Development Lifecycle

```
[Draft] → [Review] → [Revision] → [Approved] → [Published]
   ↑                                              ↓
   └──────────────[Update Required]──────────────┘
```

**States**:
- **Draft**: Initial writing, not ready for review
- **Review**: Technical review, peer feedback
- **Revision**: Addressing review comments
- **Approved**: Ready for publication
- **Published**: Live on GitHub Pages
- **Update Required**: Needs maintenance (dependencies changed, etc.)

### Code Example Lifecycle

```
[Written] → [Tested] → [Validated] → [Published]
                ↑           ↓
                └──[Failed]─┘
```

**Validation Checks**:
- Syntax validation (linting)
- Dependency availability
- Execution test (for runnable examples)
- Output verification

---

## Content Metrics

### Quality Indicators

- **Code Coverage**: % of concepts with code examples
- **Lab Coverage**: % of chapters with hands-on exercises
- **Cross-Reference Density**: Average links per chapter
- **Accessibility Score**: % compliance with WCAG guidelines
- **Technical Accuracy**: Expert review validation

### Success Metrics (from spec)

- **SC-002**: 27 chapters + Preface + Appendices (100% complete)
- **SC-003**: All chapters follow required structure (100% compliance)
- **SC-004**: 95% of code examples runnable/logically consistent
- **SC-005**: Consistent tone and style (manual review)

---

## Data Sources

### External References

1. **ROS 2 Documentation**: https://docs.ros.org/
2. **NVIDIA Isaac Sim**: https://docs.omniverse.nvidia.com/isaacsim/
3. **Gazebo**: https://gazebosim.org/
4. **Unity Robotics**: https://github.com/Unity-Technologies/Unity-Robotics-Hub

### Internal References

1. **Constitution**: `.specify/memory/constitution.md`
2. **Spec**: `specs/001-ai-robotics-textbook/spec.md`
3. **Research**: `specs/001-ai-robotics-textbook/research.md`
4. **Templates**: `specs/001-ai-robotics-textbook/contracts/`

---

## Migration Strategy

### Content Updates

When updating existing chapters:

1. **Version Control**: Create feature branch
2. **Change Documentation**: Update frontmatter `last_updated`
3. **Dependency Check**: Verify code examples still work
4. **Cross-Reference Check**: Ensure links still valid
5. **Review**: Technical review before merging
6. **Deployment**: Merge triggers automated rebuild

### Schema Changes

If entity structure changes:

1. **Document Change**: Create ADR for significant changes
2. **Update Templates**: Modify chapter templates
3. **Migrate Existing**: Update all affected chapters
4. **Validate**: Run consistency checks
5. **Deploy**: Update production site

---

## Next Steps

Proceed to **contracts/** to define:
1. Docusaurus configuration contract
2. Chapter template (concrete Markdown structure)
3. Sidebar structure (navigation organization)
