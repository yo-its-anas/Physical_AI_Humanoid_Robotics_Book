# Implementation Plan: AI-Native Robotics Textbook

**Branch**: `001-ai-robotics-textbook` | **Date**: 2025-12-05 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-ai-robotics-textbook/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create a comprehensive, AI-native textbook for teaching Physical AI & Humanoid Robotics using Claude Code and Spec-Kit Plus. The textbook will be structured as a Docusaurus static site covering 8 parts (27 chapters + preface + appendices), deployed to GitHub Pages. Content includes foundational concepts, ROS 2, simulation (Gazebo/Unity), NVIDIA Isaac Sim, Vision-Language-Action robotics, hardware lab setup, and a capstone project. Each chapter follows a structured format with technical tutorials, code examples, and lab exercises.

## Technical Context

**Language/Version**: Markdown for content, JavaScript/Node.js 18+ for Docusaurus
**Primary Dependencies**: Docusaurus 3.x, React, MDX, @docusaurus/preset-classic
**Storage**: Static files in Git repository, deployed to GitHub Pages
**Testing**: Content validation (link checking, code block syntax), manual QA review
**Target Platform**: Web browsers (GitHub Pages static site hosting)
**Project Type**: Documentation/educational content site (static site generator)
**Performance Goals**: Fast page load (<2s), responsive navigation, mobile-friendly
**Constraints**: Static site only (no backend), GitHub Pages deployment limits, Markdown/MDX content format
**Scale/Scope**: 27 chapters + preface + 5 appendices (~300-500 pages), 100+ code examples, 50+ lab exercises

**Content Technologies Covered**:
- ROS 2 (Humble/Iron): Python rclpy, launch files, URDF/SDF
- Simulation: Gazebo Classic/Fortress, Unity-ROS bridge
- NVIDIA Isaac Sim: Omniverse, Isaac ROS, Nav2
- Vision-Language-Action: Whisper, LLM integration, multimodal interaction
- Hardware: Jetson Orin, RealSense, IMUs, Unitree robots

**Documentation Structure**:
- Docusaurus sidebar with 8 main categories
- Each chapter: Markdown file with frontmatter
- Code examples: Fenced code blocks with syntax highlighting
- Cross-references: Docusaurus internal links
- Assets: Images, diagrams (text-based or SVG preferred)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ I. Technical Accuracy & Rigor
- **Status**: PASS
- **Evidence**: Spec mandates real ROS 2, Gazebo, Isaac Sim, and VLA workflows (FR-007, FR-010). Success criteria SC-007 requires expert validation of technical accuracy.

### ✅ II. Structured & Comprehensive Content
- **Status**: PASS
- **Evidence**: Chapter structure defined (FR-006): overview → background → core concepts → implementation → labs/exercises → summary. All 27 chapters outlined with cross-referencing (FR-016).

### ✅ III. AI-Native Development Workflow
- **Status**: PASS
- **Evidence**: FR-014 mandates use of Claude Code and Spec-Kit Plus for all stages. SC-006 validates this in final deliverable.

### ✅ IV. Docusaurus & GitHub Pages Publication
- **Status**: PASS
- **Evidence**: FR-011, FR-012, FR-013 specify Docusaurus structure and GitHub Pages deployment. SC-001 requires successful public deployment.

### ✅ V. Code Quality & Runnability
- **Status**: PASS
- **Evidence**: FR-009 mandates fenced code blocks with language tags, runnable or logically consistent. SC-004 requires 95% code example validation.

### ✅ VI. Real-world Relevance
- **Status**: PASS
- **Evidence**: FR-010 requires industry-aligned robotics practices. Target audience includes robotics instructors and AI engineers (SC-010).

**Overall Gate Status**: ✅ PASS - All constitutional principles satisfied by feature specification.

## Project Structure

### Documentation (this feature)

```text
specs/001-ai-robotics-textbook/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   ├── docusaurus-config.md    # Docusaurus configuration contract
│   ├── chapter-template.md     # Standard chapter structure
│   └── sidebar-structure.md    # Navigation organization
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Docusaurus Site Structure (repository root)

```text
ai_robotics_book/
├── docs/                           # Main content directory
│   ├── preface/
│   │   ├── index.md                # What this textbook is
│   │   ├── how-created.md          # Claude Code + Spec-Kit Plus workflow
│   │   ├── audience.md             # Who this is for
│   │   ├── how-to-use.md           # Usage guide
│   │   └── requirements.md         # Software + hardware requirements
│   │
│   ├── 01-physical-ai/             # Part I: Foundations
│   │   ├── index.md
│   │   ├── 01-introduction.md      # Chapter 1
│   │   ├── 02-humanoid-landscape.md # Chapter 2
│   │   ├── 03-sensor-foundations.md # Chapter 3
│   │   └── 04-weekly-overview.md    # Chapter 4
│   │
│   ├── 02-ros2/                    # Part II: ROS 2
│   │   ├── index.md
│   │   ├── 05-fundamentals.md      # Chapter 5
│   │   ├── 06-python-development.md # Chapter 6
│   │   ├── 07-urdf.md              # Chapter 7
│   │   └── 08-control-systems.md   # Chapter 8
│   │
│   ├── 03-gazebo-unity/           # Part III: Simulation
│   │   ├── index.md
│   │   ├── 09-gazebo-intro.md      # Chapter 9
│   │   ├── 10-humanoid-sim.md      # Chapter 10
│   │   └── 11-unity-robotics.md    # Chapter 11
│   │
│   ├── 04-nvidia-isaac/           # Part IV: NVIDIA Isaac
│   │   ├── index.md
│   │   ├── 12-isaac-sim.md         # Chapter 12
│   │   ├── 13-perception.md        # Chapter 13
│   │   ├── 14-isaac-ros.md         # Chapter 14
│   │   └── 15-navigation.md        # Chapter 15
│   │
│   ├── 05-vla/                    # Part V: Vision-Language-Action
│   │   ├── index.md
│   │   ├── 16-vla-intro.md         # Chapter 16
│   │   ├── 17-voice-whisper.md     # Chapter 17
│   │   ├── 18-llm-planning.md      # Chapter 18
│   │   └── 19-multimodal.md        # Chapter 19
│   │
│   ├── 06-hardware-lab/           # Part VI: Hardware Lab
│   │   ├── index.md
│   │   ├── 20-workstations.md      # Chapter 20
│   │   ├── 21-jetson.md            # Chapter 21
│   │   ├── 22-robot-options.md     # Chapter 22
│   │   └── 23-cloud-infra.md       # Chapter 23
│   │
│   ├── 07-capstone/               # Part VII: Capstone
│   │   ├── index.md
│   │   ├── 24-architecture.md      # Chapter 24
│   │   ├── 25-implementation.md    # Chapter 25
│   │   ├── 26-sim-to-real.md       # Chapter 26
│   │   └── 27-evaluation.md        # Chapter 27
│   │
│   └── 08-appendix/               # Part VIII: Appendices
│       ├── index.md
│       ├── a-ros2-cheatsheet.md    # Appendix A
│       ├── b-urdf-reference.md     # Appendix B
│       ├── c-troubleshooting.md    # Appendix C
│       ├── d-hardware-checklists.md # Appendix D
│       └── e-instructor-guides.md   # Appendix E
│
├── src/                            # Docusaurus React customizations
│   ├── components/                 # Custom React components
│   ├── css/                        # Custom styles
│   └── pages/                      # Custom pages (homepage)
│
├── static/                         # Static assets
│   ├── img/                        # Images, logos, diagrams
│   └── code-examples/              # Downloadable code samples
│       ├── ros2/
│       ├── gazebo/
│       ├── isaac/
│       └── vla/
│
├── docusaurus.config.js            # Main Docusaurus configuration
├── sidebars.js                     # Sidebar navigation structure
├── package.json                    # Node.js dependencies
├── README.md                       # Repository documentation
└── .github/
    └── workflows/
        └── deploy.yml              # GitHub Pages deployment workflow
```

**Structure Decision**: Docusaurus documentation site with 8 main content sections matching the textbook parts. Each part is a top-level folder under `docs/` containing chapter markdown files. Static code examples stored separately for download. GitHub Actions handles automated deployment to GitHub Pages.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No constitutional violations detected. This section is not applicable.
