<!-- Sync Impact Report:
Version change: 1.0.0 → 1.1.0
Modified principles: None (existing principles preserved)
Added sections:
  - Principle VII: RAG System Isolation & Safety
  - Principle VIII: RAG Technical Stack & Architecture
  - Principle IX: Dual-Mode RAG Interaction
  - Updated Development Tools & Technologies section
  - Updated Project Requirements section
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ compatible (no changes needed)
  - .specify/templates/spec-template.md: ✅ compatible (no changes needed)
  - .specify/templates/tasks-template.md: ✅ compatible (no changes needed)
  - .claude/commands/*.md: ✅ compatible (no changes needed)
  - README.md: ⚠ pending (may need RAG feature documentation)
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

### VII. RAG System Isolation & Safety
The RAG chatbot integration MUST be completely isolated from the existing textbook content. All RAG code MUST reside exclusively in the `/rag/` top-level directory. NO modifications to `/docs/`, `/src/pages/`, or core Docusaurus configuration files are permitted unless explicitly required for minimal integration points (e.g., adding a chat component import). The existing GPT-generated textbook structure MUST be preserved without modification. This principle ensures backward compatibility and prevents accidental content corruption.

**Rationale**: The textbook is a complete, working educational resource. RAG enhancement is additive, not destructive. Isolation protects months of content creation work and maintains system stability.

### VIII. RAG Technical Stack & Architecture
The RAG system MUST use:
- **Gemini API** (free tier) for embeddings and LLM responses
- **Qdrant Cloud** (free tier) for vector storage
- **FastAPI** for backend API server
- **OpenAI ChatKit SDK** for frontend UI ONLY (NOT for LLM calls)
- **Vercel serverless functions** for deployment compatibility

All backend logic MUST connect to Gemini, not OpenAI. ChatKit serves purely as a UI component framework, communicating with the custom FastAPI backend. This architecture maximizes cost efficiency (free Gemini API) while providing a professional chat interface.

**Rationale**: Free-tier services reduce operational costs for educational projects. Decoupling UI (ChatKit) from LLM provider (Gemini) allows flexibility and cost optimization.

### IX. Dual-Mode RAG Interaction
The RAG chatbot MUST support two distinct modes:

1. **Normal RAG Mode**: User query → embed via Gemini → retrieve from Qdrant → augment context → answer via Gemini
2. **Selected-Text Mode**: User sends highlighted text → skip Qdrant retrieval → answer ONLY from provided text via Gemini

Mode selection MUST be automatic based on whether the user provides selected text with their query. This enables both semantic search across the entire textbook and focused explanations of specific passages.

**Rationale**: Students need both broad search capabilities (find relevant chapters) and deep comprehension support (explain this specific paragraph). Dual-mode design serves both use cases without UI complexity.

## Development Tools & Technologies

### Core Textbook Stack
- Claude Code for file operations and coding
- Spec-Kit Plus for full project specification, planning, tasking, and execution
- Docusaurus for static book site
- GitHub Pages for hosting
- Git + GitHub repository integration
- Markdown-based content output
- NPM/terminal MCP for Docusaurus commands

### RAG System Stack
- **Backend**: Python 3.9+, FastAPI, Gemini API SDK, Qdrant client, uvicorn
- **Frontend**: React (via Docusaurus), OpenAI ChatKit SDK, axios/fetch for API calls
- **Vector Database**: Qdrant Cloud (free tier)
- **LLM Provider**: Google Gemini API (free tier)
- **Deployment**: Vercel serverless functions (Python runtime)
- **Development**: Claude Code filesystem MCP, Claude terminal MCP

## Project Requirements

### Textbook Requirements (Existing)
**Purpose**: Create an end-to-end textbook that teaches Physical AI, embodied intelligence, humanoid robot systems, and VLA-driven control pipelines—presented through an AI-native workflow using Claude Code and Spec-Kit Plus.

**Target Audience**: University-level students, robotics instructors, AI engineers, and advanced learners of ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action robotics.

**References & Source Material**: Physical AI & Humanoid Robotics course overview, all modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA), hardware requirements (Jetson kits, RealSense, IMUs, Unitree robots), weekly learning outcomes, capstone project description, lab infrastructure descriptions (Sim Rig, Edge Kit, Robot Lab).

### RAG System Requirements (New)
**Purpose**: Add an intelligent chatbot assistant to the textbook website that helps students search content semantically, get explanations of specific passages, and receive contextual answers to robotics questions based on the textbook corpus.

**Target Users**: Students reading the textbook who need quick answers, concept clarification, or help finding relevant sections.

**Non-Functional Requirements**:
- **Isolation**: Zero modifications to existing `/docs/` content files
- **Performance**: <3s response time for RAG queries (P95)
- **Cost**: Free-tier only (Gemini API, Qdrant Cloud)
- **Scalability**: Support at least 100 concurrent users on Vercel free tier
- **Security**: API keys stored in environment variables, never committed to Git
- **Maintainability**: All RAG code isolated in `/rag/` for easy updates/removal

**Mandatory File Structure**:
```
/rag/
├── ingest.py              # Markdown → Qdrant vectorization script
├── rag_server.py          # FastAPI backend (Gemini + Qdrant)
├── requirements.txt       # Python dependencies
├── vercel.json           # Vercel deployment config
└── config.env.example    # Environment template (API keys)

/src/components/RagChat/
├── index.jsx             # React component wrapper
└── ChatKitWrapper.js     # OpenAI ChatKit SDK integration

/static/
└── ragchat.css          # Chat UI styles
```

**Integration Points** (minimal modifications allowed):
- `/src/components/RagChat/` creation (new directory)
- `/static/ragchat.css` creation (new file)
- Optional: Add `<RagChat />` import to a Docusaurus page/layout (if specified in implementation)

## Governance

This constitution defines the rules governing all subsequent stages. Amendments require:
1. Documentation of the change rationale
2. Version increment following semantic versioning (MAJOR.MINOR.PATCH)
3. Update to the Sync Impact Report at the top of this file
4. Validation that dependent templates remain consistent
5. User approval for MAJOR version changes

All changes to the book project and RAG system MUST comply with these principles. Violations of Principle VII (RAG System Isolation) are considered critical and MUST be rejected.

**Compliance Reviews**:
- Before `/sp.plan`: Verify constitution alignment
- After `/sp.tasks`: Ensure tasks respect isolation boundaries
- Before implementation: Validate file paths match required structure
- After implementation: Confirm no `/docs/` modifications occurred

**Version**: 1.1.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-09
