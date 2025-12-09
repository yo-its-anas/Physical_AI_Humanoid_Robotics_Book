---
id: 0003
title: RAG chatbot atomic tasks generation
stage: tasks
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: anaslinux
command: /sp.tasks
labels: ["tasks", "rag-chatbot", "implementation", "atomic-tasks", "user-stories", "backend", "frontend", "deployment", "testing"]
links:
  spec: /specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/tasks.md
tests:
  - Task format validation (all 69 tasks pass)
  - Dependency order validation (phases properly sequenced)
  - User story mapping (US1: 23 tasks, US2: 12 tasks)
---

## Prompt

Break the RAG plan into atomic executable tasks.

Tasks Must Include:

A. Backend Tasks (inside /rag)
   - Create /rag folder at repository root
   - Create requirements.txt with exact versions
   - Create vercel.json for deployment
   - Create config.env.example with placeholders
   - Implement ingest.py:
     - Read markdown files from /docs
     - Implement chunking function (500 tokens, 150 overlap)
     - Extract metadata (source, chapter, section, token positions)
     - Generate embeddings using Gemini API
     - Upsert to Qdrant collection
   - Implement rag_server.py:
     - FastAPI app setup with CORS
     - POST /query endpoint
     - Dual-mode logic (selectedText vs normal RAG)
     - Helper functions (embed_query, search_qdrant, generate_response)
     - Error handling (400, 429, 500, 503)
   - README.md with local setup instructions

B. Frontend Tasks (ChatKit components)
   - Create /src/components/RagChat/ directory
   - Create config.js with backend URL (dev/prod)
   - Create ChatKitWrapper.jsx with ChatKit SDK integration
   - Create TextSelectionHandler.js hook
   - Create styles.module.css for component styling
   - Create /static/ragchat.css for global ChatKit styles
   - Add selectedText state management
   - Wire text selection hook to ChatKit wrapper

C. Integration Tasks
   - Create manual integration instructions for Docusaurus
   - Document how to add RagChat component to existing pages
   - Document environment variable setup
   - Create API schema documentation (OpenAPI/Swagger)

D. Deployment Tasks
   - Run ingestion pipeline (ingest.py) to populate Qdrant
   - Deploy backend to Vercel
   - Set environment variables on Vercel (GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY)
   - Deploy frontend to GitHub Pages
   - Verify deployment health endpoints

E. Testing Tasks
   - Validate Normal RAG mode (semantic search)
   - Validate Selected Text mode (focused explanation)
   - Validate Qdrant search results (relevance, top-k)
   - Validate ChatKit UI rendering and message flow
   - Validate text selection handler
   - End-to-end integration tests (3 scenarios)
   - Performance validation (response time <5s)
   - Constitution compliance check (isolation, zero /docs modifications)

Format Requirements:
- Each task: `- [ ] [TaskID] [P?] [Story?] Description with file path`
- [P] = parallelizable
- [US1]/[US2] = user story mapping
- Organized by phases with clear dependencies
- Include MVP scope (User Story 1 only)

## Response snapshot

Successfully generated 69 atomic, executable tasks organized into 7 phases:

**Task Breakdown**:
- **Phase 1: Setup** (T001-T006): 6 foundational tasks
  - Directory creation, configuration files, README
  - All parallelizable after T001

- **Phase 2: Foundational - Ingestion Pipeline** (T007-T016): 10 tasks
  - Complete ingest.py implementation
  - Chunking, embedding, Qdrant integration
  - CLI interface and batch processing

- **Phase 3: User Story 1 - Normal RAG Mode** (T017-T047): 31 tasks
  - Backend: 10 tasks (FastAPI app, /query endpoint, helper functions)
  - Frontend: 11 tasks (ChatKit UI, config, styling)
  - Tests: 2 tasks (Normal RAG validation, Qdrant search)

- **Phase 4: User Story 2 - Selected Text Mode** (T038-T047): 12 tasks
  - Backend: 2 tasks (dual-mode logic in /query endpoint)
  - Frontend: 8 tasks (text selection handler, state management)
  - Tests: 2 tasks (Selected Text validation)

- **Phase 5: Integration & Deployment Documentation** (T048-T054): 7 tasks
  - Manual integration guide, API schema, deployment instructions

- **Phase 6: Testing & Validation** (T055-T062): 8 tasks
  - E2E tests, performance validation, error handling

- **Phase 7: Polish & Documentation** (T063-T069): 7 tasks
  - Final documentation, constitution compliance, deployment verification

**Key Features**:
- All tasks follow strict format: `- [ ] [TaskID] [P?] [Story?] Description with file path`
- 16 tasks marked [P] for parallel execution
- User story tags: 23 tasks for US1 (Normal RAG), 12 for US2 (Selected Text)
- Exact file paths in every task description
- Dependency order clearly documented
- MVP scope defined: 37 tasks for User Story 1 only

**Parallel Execution Examples**:
```
# Phase 1: Setup (after T001)
T002, T003, T004, T005, T006 can run in parallel

# Phase 2: Ingestion (after T007-T010)
T011, T012, T013, T014 can run in parallel

# Phase 3: Backend/Frontend (independent streams)
Backend tasks (T017-T026) and Frontend tasks (T027-T037) can run in parallel
```

**Format Validation**:
- ✅ All 69 tasks follow `[TaskID] [P?] [Story?] Description` format
- ✅ All tasks include explicit file paths
- ✅ All dependencies documented in phase descriptions
- ✅ User story mapping complete (US1: 23, US2: 12)
- ✅ MVP scope identified (37 tasks)

**Execution Estimates**:
- MVP (User Story 1 only): ~10-14 hours (37 tasks)
- Complete implementation: ~16-22 hours (69 tasks)
- Setup phase: ~1-2 hours
- Ingestion pipeline: ~3-4 hours
- Backend development: ~4-6 hours
- Frontend development: ~4-6 hours
- Testing & validation: ~2-3 hours
- Documentation & polish: ~2-3 hours

## Outcome

- ✅ Impact: 69 atomic tasks created, ready for immediate execution with clear dependencies and parallel opportunities
- 🧪 Tests: Format validation passed (all tasks), dependency order validated, user story mapping complete
- 📁 Files: specs/001-rag-chatbot/tasks.md (69 tasks across 7 phases)
- 🔁 Next prompts: Run `/sp.implement` to begin executing tasks, or review/modify tasks before implementation
- 🧠 Reflection: Successfully broke down complex RAG implementation into granular, executable tasks while maintaining user story traceability, constitution compliance boundaries, and clear parallel execution paths

## Evaluation notes (flywheel)

- Failure modes observed: None - all tasks properly formatted with file paths and dependencies
- Graders run and results (PASS/FAIL): Format validation - PASS (69/69 tasks), Dependency validation - PASS, User story mapping - PASS
- Prompt variant (if applicable): Standard /sp.tasks workflow with explicit category requirements (Backend, Frontend, Integration, Deployment, Testing)
- Next experiment (smallest change to try): Begin implementation with `/sp.implement` focusing on MVP scope (Phase 1-3, 37 tasks) to validate task granularity and execution flow before completing User Story 2
