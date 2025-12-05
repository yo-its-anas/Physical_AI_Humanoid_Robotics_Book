# Tasks: AI-Native Robotics Textbook

**Input**: Design documents from `/specs/001-ai-robotics-textbook/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: No automated tests requested for this documentation project. Validation will be manual (content review, link checking, build verification).

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

This is a Docusaurus documentation project:
- **Content**: `docs/` at repository root
- **Code examples**: `static/code-examples/`
- **Configuration**: Root level (docusaurus.config.js, sidebars.js, package.json)
- **Spec artifacts**: `specs/001-ai-robotics-textbook/`

---

## Phase 1: Setup (Project Infrastructure)

**Purpose**: Initialize Docusaurus project and establish repository structure

- [ ] T001 Create Docusaurus project structure in repository root using `npx create-docusaurus@latest`
- [ ] T002 Install Node.js dependencies from package.json (Docusaurus 3.x, React, MDX, Mermaid plugin)
- [ ] T003 [P] Create docusaurus.config.js per contracts/docusaurus-config.md specification
- [ ] T004 [P] Create src/css/custom.css with theme customization and robotics-specific styles
- [ ] T005 [P] Create static/img/ directory and add placeholder logo.svg and favicon.ico
- [ ] T006 Configure GitHub repository settings (if not already done)
- [ ] T007 [P] Create .gitignore file for Node.js/Docusaurus (node_modules/, .docusaurus/, build/)
- [ ] T008 Verify local development server starts successfully with `npm start`

**Checkpoint**: Docusaurus project initializes and runs at localhost:3000

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core structure that MUST be complete before ANY content creation can begin

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T009 Create docs/ folder structure with all 8 part directories (preface/, 01-physical-ai/, 02-ros2/, 03-gazebo-unity/, 04-nvidia-isaac/, 05-vla/, 06-hardware-lab/, 07-capstone/, 08-appendix/)
- [ ] T010 [P] Create sidebars.js per contracts/sidebar-structure.md with complete navigation hierarchy
- [ ] T011 [P] Create static/code-examples/ directory structure (ros2/, gazebo/, isaac/, vla/ subdirectories)
- [ ] T012 [P] Create README.md at repository root documenting project purpose and setup
- [ ] T013 [P] Create .github/workflows/deploy.yml for automated GitHub Pages deployment
- [ ] T014 Install @cmfcmf/docusaurus-search-local plugin for local search functionality
- [ ] T015 Install @docusaurus/theme-mermaid for diagram support
- [ ] T016 [P] Create package.json scripts (start, build, serve, deploy, clear)
- [ ] T017 Verify build succeeds with `npm run build`

**Checkpoint**: Foundation ready - content creation can now begin in parallel

---

## Phase 3: User Story 1 - Comprehensive Learning Journey (Priority: P1) 🎯 MVP

**Goal**: Create complete curriculum from foundational concepts to capstone project with all 27 chapters structured and navigable

**Independent Test**: Student can navigate through all chapters in order, find learning objectives, prerequisites, and exercises for each chapter

### Preface Content (Foundation for US1)

- [ ] T018 [P] [US1] Create docs/preface/index.md - "What This Textbook Is" with course overview
- [ ] T019 [P] [US1] Create docs/preface/how-created.md - AI-native workflow with Claude Code + Spec-Kit Plus
- [ ] T020 [P] [US1] Create docs/preface/audience.md - Target audience description
- [ ] T021 [P] [US1] Create docs/preface/how-to-use.md - Usage guide for students and instructors
- [ ] T022 [P] [US1] Create docs/preface/requirements.md - Software + hardware requirements overview

### Part I: Foundations of Physical AI (Chapters 1-4)

- [ ] T023 [P] [US1] Create docs/01-physical-ai/index.md - Part I overview
- [ ] T024 [US1] Create docs/01-physical-ai/01-introduction.md - Chapter 1: Introduction to Physical AI (overview, background, core concepts: Physical AI definition, embodied intelligence, AI + physics, humanoid robotics rise; summary)
- [ ] T025 [US1] Create docs/01-physical-ai/02-humanoid-landscape.md - Chapter 2: Humanoid Robotics Landscape (modern humanoids: Unitree, Tesla, Agility, Robotis; embodiment, morphology, kinematics; why humanoid form matters)
- [ ] T026 [US1] Create docs/01-physical-ai/03-sensor-foundations.md - Chapter 3: Sensor Foundations (cameras, depth sensors, IMUs, LiDAR, force/torque, tactile; multimodal fusion; data rates, latencies, noise)
- [ ] T027 [US1] Create docs/01-physical-ai/04-weekly-overview.md - Chapter 4: Course Overview (weekly breakdown, skills progression, theory→sim→physical→VLA→capstone lifecycle)

### Part II: ROS 2 (Chapters 5-8)

- [ ] T028 [P] [US1] Create docs/02-ros2/index.md - Part II overview
- [ ] T029 [US1] Create docs/02-ros2/05-fundamentals.md - Chapter 5: ROS 2 Fundamentals (architecture, design goals, nodes/topics/services/actions, DDS communication, Humble/Iron tooling)
- [ ] T030 [US1] Create docs/02-ros2/06-python-development.md - Chapter 6: Building with ROS 2 & Python (workspace setup with colcon, writing nodes in Python, launch files, parameters, ROS bags, rclpy integration)
- [ ] T031 [US1] Create docs/02-ros2/07-urdf.md - Chapter 7: Robot Description with URDF (URDF structure, building humanoid model, links/joints/transmissions, visual/collision meshes, exporting to simulation)
- [ ] T032 [US1] Create docs/02-ros2/08-control-systems.md - Chapter 8: Control Systems for Humanoids (controllers, PID, trajectories, ROS 2 control stack, sensor→controller→actuator loops)

### Part III: Gazebo & Unity Simulation (Chapters 9-11)

- [ ] T033 [P] [US1] Create docs/03-gazebo-unity/index.md - Part III overview
- [ ] T034 [US1] Create docs/03-gazebo-unity/09-gazebo-intro.md - Chapter 9: Introduction to Gazebo Simulation (architecture, SDF/URDF, physics engines, gravity, collisions)
- [ ] T035 [US1] Create docs/03-gazebo-unity/10-humanoid-sim.md - Chapter 10: Simulating Humanoids in Gazebo (importing URDF/SDF, joint limits/mass/inertia, testing locomotion/balance/manipulation, simulating sensors)
- [ ] T036 [US1] Create docs/03-gazebo-unity/11-unity-robotics.md - Chapter 11: Unity for Robotics Visualization (Unity-ROS bridge, high-fidelity rendering, human-robot interaction simulation)

### Part IV: NVIDIA Isaac (Chapters 12-15)

- [ ] T037 [P] [US1] Create docs/04-nvidia-isaac/index.md - Part IV overview
- [ ] T038 [US1] Create docs/04-nvidia-isaac/12-isaac-sim.md - Chapter 12: Introduction to NVIDIA Isaac Sim (Omniverse fundamentals, USD format & scenes, Isaac Sim workflow)
- [ ] T039 [US1] Create docs/04-nvidia-isaac/13-perception.md - Chapter 13: Perception & Synthetic Data (photorealistic simulation, generating synthetic datasets, computer vision for humanoids)
- [ ] T040 [US1] Create docs/04-nvidia-isaac/14-isaac-ros.md - Chapter 14: Isaac ROS (VSLAM, navigation & perception pipelines, sensor integration with Jetson hardware)
- [ ] T041 [US1] Create docs/04-nvidia-isaac/15-navigation.md - Chapter 15: Navigation for Humanoids (Nav2 path planning/mapping/localization, biped navigation challenges, integrating Nav2 in Isaac + ROS 2)

### Part V: Vision-Language-Action (Chapters 16-19)

- [ ] T042 [P] [US1] Create docs/05-vla/index.md - Part V overview
- [ ] T043 [US1] Create docs/05-vla/16-vla-intro.md - Chapter 16: Introduction to VLA Systems (LLMs + robotics convergence, representing goals/tasks/actions)
- [ ] T044 [US1] Create docs/05-vla/17-voice-whisper.md - Chapter 17: Voice-to-Action with Whisper (speech recognition pipeline, converting voice commands to structured tasks)
- [ ] T045 [US1] Create docs/05-vla/18-llm-planning.md - Chapter 18: Cognitive Planning with LLMs (task decomposition, ROS 2 action mapping, safety/reliability & prompt engineering for robotics)
- [ ] T046 [US1] Create docs/05-vla/19-multimodal.md - Chapter 19: Multimodal Interaction (speech + gesture + vision fusion, human-robot interaction patterns, embodied dialogue systems)

### Part VI: Hardware Lab (Chapters 20-23)

- [ ] T047 [P] [US1] Create docs/06-hardware-lab/index.md - Part VI overview
- [ ] T048 [US1] Create docs/06-hardware-lab/20-workstations.md - Chapter 20: High-Performance Simulation Workstations (RTX GPU requirements, CPU/RAM/OS Ubuntu 22.04, local vs cloud simulation)
- [ ] T049 [US1] Create docs/06-hardware-lab/21-jetson.md - Chapter 21: Jetson Edge AI Kits (Jetson Orin Nano/NX, deploying ROS 2 to edge, IMU/cameras/microphones, RealSense integration)
- [ ] T050 [US1] Create docs/06-hardware-lab/22-robot-options.md - Chapter 22: Robot Options (proxy quadrupeds Unitree Go2, mini-humanoids Robotis OP3/Hiwonder, full humanoids Unitree G1, lab architecture examples)
- [ ] T051 [US1] Create docs/06-hardware-lab/23-cloud-infra.md - Chapter 23: Cloud-Based Robotics Infrastructure (AWS GPU instances, Omniverse cloud, latency challenges & sim-to-real workflows)

### Part VII: Capstone (Chapters 24-27)

- [ ] T052 [P] [US1] Create docs/07-capstone/index.md - Part VII overview
- [ ] T053 [US1] Create docs/07-capstone/24-architecture.md - Chapter 24: Capstone Architecture Overview (full system: sensors→perception→planning→control, Isaac + ROS 2 + VLA pipeline)
- [ ] T054 [US1] Create docs/07-capstone/25-implementation.md - Chapter 25: Implementing Voice-Driven Humanoid (voice→LLM planner→ROS actions, navigation pipeline, object detection & manipulation)
- [ ] T055 [US1] Create docs/07-capstone/26-sim-to-real.md - Chapter 26: Sim-to-Real Transfer (taking Isaac Sim models to real hardware, training→export→deployment to Jetson, safety considerations)
- [ ] T056 [US1] Create docs/07-capstone/27-evaluation.md - Chapter 27: Final Evaluation & Extensions (testing scenarios, robot performance scoring, future directions: dexterity/collaboration/autonomy)

### Part VIII: Appendices (A-E)

- [ ] T057 [P] [US1] Create docs/08-appendix/index.md - Appendices overview
- [ ] T058 [P] [US1] Create docs/08-appendix/a-ros2-cheatsheet.md - Appendix A: Complete ROS 2 Cheat Sheets (CLI commands, common patterns, quick reference)
- [ ] T059 [P] [US1] Create docs/08-appendix/b-urdf-reference.md - Appendix B: URDF/SDF Reference (XML tags, attributes, examples)
- [ ] T060 [P] [US1] Create docs/08-appendix/c-troubleshooting.md - Appendix C: Isaac Sim, Nav2, VSLAM Troubleshooting (common errors, solutions)
- [ ] T061 [P] [US1] Create docs/08-appendix/d-hardware-checklists.md - Appendix D: Hardware Setup Checklists (installation steps, verification procedures)
- [ ] T062 [P] [US1] Create docs/08-appendix/e-instructor-guides.md - Appendix E: Instructor Guides and Lab Rubrics (teaching notes, grading criteria)

**Checkpoint**: All 27 chapters + preface + appendices exist and are navigable via sidebar. Student can read through entire curriculum.

---

## Phase 4: User Story 2 - Practical Skill Development (Priority: P1)

**Goal**: Add technical tutorials, runnable code examples, and hands-on lab exercises to enable practical learning

**Independent Test**: Student can download code examples, run them successfully, and complete lab exercises with clear deliverables

### ROS 2 Code Examples

- [ ] T063 [P] [US2] Create static/code-examples/ros2/nodes/minimal_publisher.py - Basic ROS 2 publisher example with full comments
- [ ] T064 [P] [US2] Create static/code-examples/ros2/nodes/minimal_subscriber.py - Basic ROS 2 subscriber example
- [ ] T065 [P] [US2] Create static/code-examples/ros2/nodes/service_client.py - ROS 2 service client example
- [ ] T066 [P] [US2] Create static/code-examples/ros2/nodes/action_server.py - ROS 2 action server example
- [ ] T067 [P] [US2] Create static/code-examples/ros2/launch/example_launch.py - Python launch file example
- [ ] T068 [P] [US2] Create static/code-examples/ros2/urdf/simple_humanoid.urdf - Basic humanoid URDF model with 10+ links
- [ ] T069 [P] [US2] Create static/code-examples/ros2/params/node_params.yaml - ROS 2 parameter file example

### Gazebo Code Examples

- [ ] T070 [P] [US2] Create static/code-examples/gazebo/worlds/empty_world.sdf - Basic Gazebo world file
- [ ] T071 [P] [US2] Create static/code-examples/gazebo/worlds/humanoid_test.sdf - Humanoid testing world with obstacles
- [ ] T072 [P] [US2] Create static/code-examples/gazebo/plugins/custom_sensor.cpp - Custom Gazebo sensor plugin example (C++)

### Isaac Sim Code Examples

- [ ] T073 [P] [US2] Create static/code-examples/isaac/sim/load_robot.py - Python script to load robot in Isaac Sim
- [ ] T074 [P] [US2] Create static/code-examples/isaac/sim/synthetic_data_gen.py - Generate synthetic training data script
- [ ] T075 [P] [US2] Create static/code-examples/isaac/ros/vslam_setup.py - Isaac ROS VSLAM configuration example
- [ ] T076 [P] [US2] Create static/code-examples/isaac/ros/nav2_config.yaml - Nav2 configuration for humanoid

### VLA Code Examples

- [ ] T077 [P] [US2] Create static/code-examples/vla/whisper/voice_recognition.py - Whisper speech-to-text integration
- [ ] T078 [P] [US2] Create static/code-examples/vla/llm/task_planner.py - LLM-based task decomposition example
- [ ] T079 [P] [US2] Create static/code-examples/vla/llm/ros_action_mapper.py - Convert LLM output to ROS 2 actions

### Lab Exercises Integration

- [ ] T080 [US2] Add Lab 1 to docs/02-ros2/06-python-development.md - "Create Your First ROS 2 Node" with deliverables and validation criteria
- [ ] T081 [US2] Add Lab 2 to docs/02-ros2/07-urdf.md - "Build Your First Humanoid URDF" with validation via check_urdf
- [ ] T082 [US2] Add Lab 3 to docs/03-gazebo-unity/10-humanoid-sim.md - "Simulate a Humanoid Walking" with expected outputs
- [ ] T083 [US2] Add Lab 4 to docs/04-nvidia-isaac/13-perception.md - "Generate Synthetic Training Data" with dataset requirements
- [ ] T084 [US2] Add Lab 5 to docs/05-vla/17-voice-whisper.md - "Implement Voice Command Recognition" with test cases
- [ ] T085 [US2] Add Lab 6 to docs/07-capstone/25-implementation.md - "Build Voice-Controlled Robot Navigation" with integration criteria

### Tutorial Enhancements (Implementation Sections)

- [ ] T086 [US2] Enhance Ch 6 (docs/02-ros2/06-python-development.md) Implementation section with step-by-step ROS 2 node creation tutorial
- [ ] T087 [US2] Enhance Ch 7 (docs/02-ros2/07-urdf.md) Implementation section with URDF building walkthrough and RViz visualization
- [ ] T088 [US2] Enhance Ch 10 (docs/03-gazebo-unity/10-humanoid-sim.md) Implementation section with Gazebo simulation setup and testing tutorial
- [ ] T089 [US2] Enhance Ch 13 (docs/04-nvidia-isaac/13-perception.md) Implementation section with Isaac Sim scene creation and data generation
- [ ] T090 [US2] Enhance Ch 15 (docs/04-nvidia-isaac/15-navigation.md) Implementation section with Nav2 setup and path planning tutorial
- [ ] T091 [US2] Enhance Ch 18 (docs/05-vla/18-llm-planning.md) Implementation section with LLM integration and task planning examples
- [ ] T092 [US2] Enhance Ch 25 (docs/07-capstone/25-implementation.md) Implementation section with complete capstone system integration tutorial

**Checkpoint**: Code examples are downloadable and runnable. Lab exercises have clear instructions and validation criteria. Students can develop practical skills.

---

## Phase 5: User Story 3 - AI-Native Workflow Adoption (Priority: P2)

**Goal**: Document and demonstrate the AI-native workflow using Claude Code and Spec-Kit Plus throughout the textbook project

**Independent Test**: User can understand the AI-native development methodology by reading the documentation and observing project artifacts

### AI-Native Workflow Documentation

- [ ] T093 [US3] Expand docs/preface/how-created.md with detailed explanation of Spec-Kit Plus workflow (specify→plan→tasks→implement→analyze→commit)
- [ ] T094 [US3] Add section to docs/preface/how-created.md showing example PHR (Prompt History Record) structure and purpose
- [ ] T095 [US3] Add section to docs/preface/how-created.md explaining ADR (Architecture Decision Records) usage for significant decisions
- [ ] T096 [US3] Add section to docs/preface/how-created.md demonstrating how constitution.md governs all development decisions

### Project Artifacts as Examples

- [ ] T097 [P] [US3] Create history/adr/0001-docusaurus-selection.md - ADR documenting why Docusaurus was chosen over alternatives
- [ ] T098 [P] [US3] Create history/adr/0002-ros2-humble-lts.md - ADR documenting ROS 2 Humble selection as primary version
- [ ] T099 [P] [US3] Create history/adr/0003-chapter-structure-standard.md - ADR for standardized chapter template
- [ ] T100 [US3] Update docs/preface/how-created.md to reference actual ADRs created during project (link to ADRs)

### Workflow Demonstration

- [ ] T101 [US3] Add "Behind the Scenes" section to docs/preface/how-created.md showing actual Claude Code commands used (/sp.specify, /sp.plan, /sp.tasks)
- [ ] T102 [US3] Add "Reproducibility" section to docs/preface/how-created.md explaining how others can use similar workflow for their projects
- [ ] T103 [US3] Add "Lessons Learned" section to docs/preface/how-created.md with insights from AI-native content generation

**Checkpoint**: AI-native workflow is thoroughly documented with real examples from this project. Users understand Spec-Kit Plus methodology.

---

## Phase 6: User Story 4 - Accessible & Deployable Resource (Priority: P2)

**Goal**: Deploy fully functional Docusaurus website to GitHub Pages with responsive design and search functionality

**Independent Test**: User can navigate to GitHub Pages URL in web browser and access complete textbook without installation

### Docusaurus Site Enhancements

- [ ] T104 [US4] Create src/pages/index.js - Custom homepage with course overview, getting started links, and feature highlights
- [ ] T105 [P] [US4] Add course hero image to static/img/hero-robotics.png (or create placeholder)
- [ ] T106 [P] [US4] Create custom React component in src/components/CourseHighlights.js for homepage feature cards
- [ ] T107 [US4] Update docusaurus.config.js navbar with links to GitHub repo and key sections
- [ ] T108 [US4] Update docusaurus.config.js footer with proper copyright, external resource links (ROS 2, NVIDIA, Gazebo)
- [ ] T109 [P] [US4] Configure social card metadata in docusaurus.config.js for better social media sharing

### Search and Navigation

- [ ] T110 [US4] Verify @cmfcmf/docusaurus-search-local plugin is working and indexing all content
- [ ] T111 [US4] Test search functionality with common queries (ROS 2, URDF, Isaac Sim, VLA, Jetson)
- [ ] T112 [US4] Verify sidebar collapsible state (Preface and Part I expanded, others collapsed)
- [ ] T113 [US4] Test breadcrumbs navigation on all pages
- [ ] T114 [US4] Test previous/next navigation between chapters

### Responsive Design and Accessibility

- [ ] T115 [US4] Test site on mobile devices (phone, tablet) for responsive layout
- [ ] T116 [US4] Verify all images have alt text for screen readers
- [ ] T117 [US4] Run Lighthouse accessibility audit and address any issues (target: >90 score)
- [ ] T118 [US4] Verify keyboard navigation works (Tab, Enter, Arrow keys)
- [ ] T119 [US4] Test with screen reader (NVDA or VoiceOver) to ensure semantic HTML

### GitHub Pages Deployment

- [ ] T120 [US4] Configure GitHub repository settings for GitHub Pages (enable Pages, set source to gh-pages branch)
- [ ] T121 [US4] Update docusaurus.config.js with correct organizationName and projectName for GitHub deployment
- [ ] T122 [US4] Update docusaurus.config.js with correct url and baseUrl for GitHub Pages
- [ ] T123 [US4] Test .github/workflows/deploy.yml GitHub Actions workflow (trigger on push to main)
- [ ] T124 [US4] Verify build succeeds in GitHub Actions (check workflow logs)
- [ ] T125 [US4] Verify deployment to gh-pages branch succeeds
- [ ] T126 [US4] Access deployed site at https://<username>.github.io/ai_robotics_book/ and verify all content loads
- [ ] T127 [US4] Test all internal links work on deployed site (not just localhost)
- [ ] T128 [US4] Verify code syntax highlighting works on deployed site
- [ ] T129 [US4] Test search functionality on deployed site

### Performance Optimization

- [ ] T130 [P] [US4] Optimize all images in static/img/ to <200KB each (compress with tools like ImageOptim)
- [ ] T131 [P] [US4] Run Lighthouse performance audit (target: >90 score)
- [ ] T132 [US4] Verify page load time <2 seconds on 3G connection
- [ ] T133 [US4] Check bundle size with webpack analyzer (target: <500KB main bundle)

**Checkpoint**: Docusaurus site is fully deployed to GitHub Pages, publicly accessible, responsive, searchable, and performs well.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements, consistency checks, and quality assurance across all content

### Content Quality and Consistency

- [ ] T134 [P] Review all chapters for consistent tone, terminology, and structure (per constitution.md)
- [ ] T135 [P] Verify all chapters follow the chapter template structure (overview→background→concepts→implementation→labs→summary)
- [ ] T136 [P] Check all learning objectives are specific and measurable (using action verbs)
- [ ] T137 [P] Verify all prerequisites are listed and linked correctly
- [ ] T138 Run link checker to find and fix any broken internal links across all chapters
- [ ] T139 Run link checker to verify all external links are accessible (ROS docs, NVIDIA docs, etc.)
- [ ] T140 [P] Verify all code blocks have proper language tags (python, xml, yaml, bash, json, cpp)
- [ ] T141 [P] Check all code examples have descriptive comments and docstrings

### Technical Validation

- [ ] T142 [P] Validate ROS 2 code examples syntax (Python linting with flake8 or ruff)
- [ ] T143 [P] Validate URDF/XML files with xmllint or ROS tools (check_urdf if available)
- [ ] T144 [P] Validate YAML files syntax (yamllint)
- [ ] T145 Review Gazebo examples for correct SDF format and physics parameters
- [ ] T146 Review Isaac Sim examples for compatibility with Isaac Sim 2023.1+
- [ ] T147 Review VLA examples for proper LLM API integration patterns

### Diagram and Media Quality

- [ ] T148 [P] Verify all Mermaid diagrams render correctly in browser
- [ ] T149 [P] Ensure all diagrams have captions with figure numbers
- [ ] T150 [P] Verify all images have descriptive alt text (accessibility requirement)
- [ ] T151 Add placeholder diagrams for chapters that need visual aids (architecture diagrams, flowcharts)

### Documentation and Metadata

- [ ] T152 [P] Verify all chapters have complete frontmatter (id, title, sidebar_label, sidebar_position, description, keywords, tags)
- [ ] T153 [P] Check sidebar.js matches actual file structure (all docs exist, correct paths)
- [ ] T154 Update repository README.md with project description, setup instructions, and contributing guidelines
- [ ] T155 [P] Add CONTRIBUTING.md with guidelines for adding chapters or improving content
- [ ] T156 [P] Add LICENSE file (choose appropriate license for educational content)

### Educational Quality

- [ ] T157 Review lab exercises for appropriate difficulty levels (beginner/intermediate/advanced)
- [ ] T158 Verify lab deliverables are specific and measurable (not vague)
- [ ] T159 Check that hints are helpful without giving away solutions
- [ ] T160 Verify self-assessment questions have answers in collapsible sections
- [ ] T161 Review estimated time allocations for chapters (reading + labs)

### Build and Deployment Final Checks

- [ ] T162 Run full build locally with `npm run build` and verify no errors or warnings
- [ ] T163 Serve production build with `npm run serve` and test navigation/search/links
- [ ] T164 Verify sitemap.xml is generated correctly
- [ ] T165 Verify robots.txt allows search engine indexing (if desired)
- [ ] T166 Test deployment workflow end-to-end (commit→push→GitHub Actions→Pages)
- [ ] T167 Final verification of deployed site functionality and performance

### Optional Enhancements (Future)

- [ ] T168 [P] Consider adding Algolia DocSearch for improved search (requires application)
- [ ] T169 [P] Consider adding version management for course iterations
- [ ] T170 [P] Consider adding blog section for course updates or robotics news
- [ ] T171 [P] Consider PDF export plugin for offline reading

**Checkpoint**: All content is consistent, technically accurate, links work, site performs well, and deployment is automated.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - US1 (Phase 3): Comprehensive Learning Journey - Foundation for all other stories
  - US2 (Phase 4): Practical Skill Development - Depends on US1 chapters existing
  - US3 (Phase 5): AI-Native Workflow - Independent, can run parallel to US2
  - US4 (Phase 6): Deployment - Should wait until substantial content exists (US1 complete minimum)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **US1 (P1)**: Can start after Foundational - No dependencies on other stories (creates all chapter structure)
- **US2 (P1)**: Depends on US1 chapters existing (adds code examples and labs to those chapters)
- **US3 (P2)**: Independent - Can run in parallel with US1/US2 (documents the workflow)
- **US4 (P2)**: Should wait for US1 completion minimum (need content to deploy), benefits from US2/US3 but not strictly dependent

### Recommended Execution Sequence

**MVP Path (Minimal Viable Product)**:
1. Phase 1: Setup → Phase 2: Foundational → Phase 3: US1 (all chapters) → Phase 6: US4 (deploy basic site)
2. This gives: Complete chapter structure + deployed, navigable textbook
3. Then add: Phase 4: US2 (code examples + labs) → Phase 5: US3 (workflow docs) → Phase 7: Polish

**Full Sequential Path** (if single developer):
1. Phase 1 → Phase 2 → Phase 3 (US1) → Phase 4 (US2) → Phase 5 (US3) → Phase 6 (US4) → Phase 7 (Polish)

**Parallel Team Path** (if multiple developers):
1. Team: Phase 1 + Phase 2 together
2. Once Foundational done:
   - Developer A: Phase 3 (US1) - Create all chapter structure
   - Developer B: Phase 5 (US3) - Document AI-native workflow
3. After US1 complete:
   - Developer A: Phase 4 (US2) - Add code examples to chapters
   - Developer B: Phase 6 (US4) - Set up deployment
4. All: Phase 7 (Polish) together

### Within Each User Story

**US1** (Chapter Creation):
- Preface chapters [P] can be created in parallel
- Part overview pages [P] can be created in parallel
- Chapters within a part can be created in parallel [P] (different files)
- Must follow chapter template structure

**US2** (Code Examples + Labs):
- All code example files [P] can be created in parallel (different files)
- Lab additions to chapters are sequential (depends on chapter existing from US1)
- Tutorial enhancements depend on chapters existing

**US3** (AI-Native Workflow):
- ADR files [P] can be created in parallel
- Documentation sections are sequential (same file edits)

**US4** (Deployment):
- Site enhancement tasks mostly sequential (same config files)
- Image optimization [P] can run in parallel
- Testing tasks are sequential (depends on deployment setup)

### Parallel Opportunities

**Maximum Parallelism Examples**:

```bash
# Phase 1 Setup - Can run T003, T004, T005, T007 in parallel (different files)
Task: "Create docusaurus.config.js"
Task: "Create src/css/custom.css"
Task: "Create static/img/ directory"
Task: "Create .gitignore file"

# Phase 2 Foundational - Can run T010, T011, T012, T013, T015 in parallel
Task: "Create sidebars.js"
Task: "Create static/code-examples/ structure"
Task: "Create README.md"
Task: "Create .github/workflows/deploy.yml"
Task: "Install @docusaurus/theme-mermaid"

# Phase 3 (US1) Preface - Can run T018-T022 in parallel (different files)
Task: "Create docs/preface/index.md"
Task: "Create docs/preface/how-created.md"
Task: "Create docs/preface/audience.md"
Task: "Create docs/preface/how-to-use.md"
Task: "Create docs/preface/requirements.md"

# Phase 3 (US1) Part Overviews - Can run T023, T028, T033, T037, T042, T047, T052, T057 in parallel
Task: "Create docs/01-physical-ai/index.md"
Task: "Create docs/02-ros2/index.md"
Task: "Create docs/03-gazebo-unity/index.md"
[... all part overview pages]

# Phase 4 (US2) Code Examples - Can run T063-T079 in parallel (different files)
Task: "Create static/code-examples/ros2/nodes/minimal_publisher.py"
Task: "Create static/code-examples/ros2/nodes/minimal_subscriber.py"
Task: "Create static/code-examples/gazebo/worlds/empty_world.sdf"
[... all code example files]

# Phase 5 (US3) ADRs - Can run T097-T099 in parallel (different files)
Task: "Create history/adr/0001-docusaurus-selection.md"
Task: "Create history/adr/0002-ros2-humble-lts.md"
Task: "Create history/adr/0003-chapter-structure-standard.md"
```

---

## Implementation Strategy

### MVP First (Fastest Path to Working Textbook)

**Goal**: Deployable textbook with all chapter structure

1. ✅ Complete Phase 1: Setup (~1-2 hours)
2. ✅ Complete Phase 2: Foundational (~2-3 hours)
3. ✅ Complete Phase 3: US1 - All 27 chapters + preface + appendices (~40-60 hours for content generation)
4. ✅ Complete Phase 6: US4 - Deploy to GitHub Pages (~3-4 hours)
5. **STOP and VALIDATE**: Full textbook is online and navigable
6. Deploy/share link for initial review

**MVP Deliverable**: Complete textbook structure with all chapters (may have basic content, no code examples yet)

### Incremental Delivery (Add Features Progressively)

1. ✅ MVP (Setup + Foundational + US1 + US4 deployment) → **Textbook is online**
2. ➕ Add Phase 4: US2 - Code examples and lab exercises (~20-30 hours) → **Practical skills enabled**
3. ➕ Add Phase 5: US3 - AI-native workflow documentation (~3-5 hours) → **Methodology showcased**
4. ➕ Add Phase 7: Polish - Quality improvements (~10-15 hours) → **Professional quality**
5. Each increment adds value without breaking previous functionality

### Parallel Team Strategy

If you have multiple contributors:

1. **Everyone**: Complete Setup + Foundational together (~3-5 hours)
2. **Split User Stories**:
   - Team Member A: US1 Chapters 1-9 (Parts I-III)
   - Team Member B: US1 Chapters 10-18 (Parts IV-V)
   - Team Member C: US1 Chapters 19-27 + Preface + Appendices (Parts VI-VIII)
   - Team Member D: US3 AI-Native Workflow (parallel to chapter creation)
3. **After US1 Complete**:
   - Team Members A+B: US2 Code Examples (split by part)
   - Team Member C: US4 Deployment Setup
   - Team Member D: US4 Testing & QA
4. **Final**: All together on Phase 7 Polish

---

## Summary Statistics

**Total Tasks**: 171 tasks

**Tasks by Phase**:
- Phase 1 (Setup): 8 tasks
- Phase 2 (Foundational): 9 tasks
- Phase 3 (US1 - Comprehensive Learning): 45 tasks (5 preface + 4 part I + 4 part II + 3 part III + 4 part IV + 4 part V + 4 part VI + 4 part VII + 6 appendices + 7 part overviews)
- Phase 4 (US2 - Practical Skills): 30 tasks (17 code examples + 6 labs + 7 tutorial enhancements)
- Phase 5 (US3 - AI-Native Workflow): 11 tasks
- Phase 6 (US4 - Deployment): 26 tasks
- Phase 7 (Polish): 42 tasks

**Tasks by User Story**:
- US1: 45 tasks (all chapter creation)
- US2: 30 tasks (code examples, labs, tutorials)
- US3: 11 tasks (AI-native workflow docs)
- US4: 26 tasks (deployment and site enhancements)
- Setup/Foundational/Polish: 59 tasks

**Parallel Opportunities**:
- Phase 1: 4 tasks can run in parallel
- Phase 2: 5 tasks can run in parallel
- Phase 3 (US1): 8 part overviews + 5 preface pages + many chapters [P] = ~30+ tasks can run in parallel
- Phase 4 (US2): 17 code example files [P] can run in parallel
- Phase 5 (US3): 3 ADRs [P] can run in parallel
- Phase 7 (Polish): ~15 tasks [P] can run in parallel

**Estimated Effort**:
- **MVP** (Setup + Foundational + US1 + US4 deployment): ~50-70 hours
- **Full Project** (All phases): ~100-130 hours
- **With Parallel Team** (4 people): ~30-40 hours wall-clock time

**Test Coverage**:
- No automated tests (documentation project)
- Manual validation: content review, link checking, build verification
- Independent test criteria defined for each user story

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable
- Commit after each task or logical group of tasks
- Stop at any checkpoint to validate story independently
- Follow chapter template structure (contracts/chapter-template.md) for all chapters
- Follow constitution.md principles for all content creation
- Verify build succeeds regularly with `npm run build`
- Test deployment locally with `npm run serve` before pushing to production
