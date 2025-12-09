---
id: 0002
title: RAG chatbot implementation plan
stage: plan
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: anaslinux
command: /sp.plan
labels: ["planning", "rag-chatbot", "implementation", "architecture", "fastapi", "chatkit", "gemini", "qdrant"]
links:
  spec: /specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/plan.md
tests:
  - Constitution compliance check (all principles passed)
---

## Prompt

Create an end-to-end implementation plan for the RAG chatbot with ChatKit UI.

Sections:

1. Initial Setup Plan
   - Qdrant cluster creation steps
   - Gemini API key creation
   - /rag folder preparation

2. Data Extraction & Chunking Plan
   - How ingest.py will read /docs
   - Chunk sizes & overlap
   - Metadata design

3. Embedding + Ingestion Plan
   - Gemini embedding model ("models/embedding-001")
   - How embeddings are stored in Qdrant

4. Backend Plan
   - FastAPI structure
   - /query endpoint
   - Logic branching (selectedText vs normal)
   - Response formatting

5. Frontend Plan (ChatKit)
   - ChatKit UI component structure
   - How frontend sends selectedText
   - How ChatKit styling will work

6. Deployment Plan
   - Vercel deployment
   - Environment variables on Vercel
   - Local testing steps

7. Testing & QA Plan
   - Validate ingestion
   - Validate Qdrant search
   - Validate selected text mode
   - Validate ChatKit UI

8. Deliverables Checklist
   - All backend files
   - Frontend ChatKit component
   - Manual integration instructions
   - Successful deployment

## Response snapshot

Successfully created comprehensive end-to-end implementation plan (~2,080 lines):

**Plan Structure**:
- Summary: Dual-mode RAG system overview and technical approach
- Technical Context: Python 3.9+, FastAPI, Gemini, Qdrant, ChatKit, Vercel
- Constitution Check: All 3 core principles validated (✅ PASS)
- Project Structure: Backend in /rag/, frontend in /src/components/RagChat/
- 8 detailed implementation sections with code examples

**Section Highlights**:

1. **Initial Setup Plan** (3 subsections):
   - Qdrant Cloud setup (create cluster, get credentials, verify connection)
   - Gemini API key creation (AI Studio, create key, test API)
   - /rag folder preparation (directory structure, config files, .gitignore)

2. **Data Extraction & Chunking Plan** (3 subsections):
   - Document reading with pathlib.Path.rglob()
   - Chunking algorithm (500 tokens, 150 overlap, sliding window)
   - Metadata design (9 fields: source, chapter, section, token positions, etc.)
   - Complete Python code examples for all functions

3. **Embedding + Ingestion Plan** (2 subsections):
   - Gemini embedding-001 integration (768-dim vectors)
   - Batch processing with rate limiting (10 chunks/batch, 1s delay)
   - Qdrant collection creation and vector upload
   - Error handling with exponential backoff retry logic
   - Full ingest.py script with CLI args

4. **Backend Plan (FastAPI)** (3 subsections):
   - Core FastAPI app setup with CORS and Pydantic models
   - POST /query endpoint with dual-mode logic
   - GET /health endpoint for monitoring
   - Complete rag_server.py implementation
   - Error handling (400, 429, 500, 503)

5. **Frontend Plan (ChatKit)** (8 subsections):
   - ChatKit SDK installation and component structure
   - Backend URL configuration (dev/prod environments)
   - ChatKitWrapper.jsx with message state management
   - Text selection handler using window.getSelection()
   - Styling with module CSS and global styles
   - Optional Docusaurus integration (Root.js)

6. **Deployment Plan** (4 subsections):
   - Vercel configuration (vercel.json, Python runtime)
   - Step-by-step Vercel deployment (CLI, env vars, production deploy)
   - Frontend deployment to GitHub Pages
   - Local testing workflow (2 terminals: backend + frontend)

7. **Testing & QA Plan** (6 subsections):
   - Ingestion testing (document loading, chunking, embedding, Qdrant upload)
   - Qdrant search testing (vector search validation)
   - Selected text mode testing (mode detection, context usage)
   - ChatKit UI testing (rendering, message flow, text selection, errors)
   - End-to-end integration tests (3 scenarios: Normal RAG, Selected Text, Error Handling)
   - Performance testing (response time targets, optional load testing)

8. **Deliverables Checklist**:
   - 6 backend files (ingest.py, rag_server.py, requirements.txt, vercel.json, config.env.example, README.md)
   - 6 frontend files (index.jsx, ChatKitWrapper.jsx, TextSelectionHandler.js, config.js, styles.module.css, ragchat.css)
   - 4 documentation files (research.md, data-model.md, quickstart.md, api-schema.yaml)
   - Deployment and testing validation checklists

**Technical Decisions Documented**:
- Tiktoken for tokenization (Gemini-compatible)
- Markdown header-aware chunking (preserve structure)
- HNSW index for Qdrant (cosine similarity)
- Async FastAPI with helper functions
- React hooks for text selection
- Environment-based backend URL configuration

**Constitution Compliance**:
- ✅ Principle VII: All RAG code isolated in /rag/ and /src/components/RagChat/
- ✅ Principle VIII: Gemini (embeddings + chat), Qdrant, FastAPI, ChatKit, Vercel
- ✅ Principle IX: Dual-mode RAG (Normal + Selected-Text) with automatic detection

## Outcome

- ✅ Impact: Complete end-to-end implementation plan created, ready for task generation
- 🧪 Tests: Constitution check passed (3/3 principles), quality validation ready
- 📁 Files: specs/001-rag-chatbot/plan.md (2,080 lines with code examples)
- 🔁 Next prompts: Phase 0: Generate research.md → Phase 1: Generate data-model.md, contracts/, quickstart.md → Run /sp.tasks
- 🧠 Reflection: Plan provides detailed step-by-step guidance for all 8 sections with executable code examples, test scenarios, and validation checklists while maintaining strict constitution compliance

## Evaluation notes (flywheel)

- Failure modes observed: None - all sections completed with comprehensive detail and code examples
- Graders run and results (PASS/FAIL): Constitution check - PASS (all 3 principles validated)
- Prompt variant (if applicable): Standard /sp.plan workflow with detailed 8-section structure
- Next experiment (smallest change to try): Validate plan completeness by generating Phase 0 research.md and Phase 1 artifacts before proceeding to tasks
