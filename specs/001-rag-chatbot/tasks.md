# Tasks: RAG Chatbot Integration

**Input**: Design documents from `/specs/001-rag-chatbot/`
**Prerequisites**: plan.md (complete), spec.md (complete)

**Tests**: Tests are included as requested in the user requirements (E. Testing Tasks)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1 = Normal RAG, US2 = Selected Text)
- Include exact file paths in descriptions

## Path Conventions

- Backend: `/rag/` (isolated directory for all RAG backend code)
- Frontend: `/src/components/RagChat/` (isolated component directory)
- Styles: `/static/ragchat.css` (global chat styles)
- Documentation: `/rag/README.md`, `/specs/001-rag-chatbot/`

---

## Phase 1: Setup (Backend Infrastructure)

**Purpose**: Initialize RAG backend structure and configuration files

- [ ] T001 Create /rag directory at repository root
- [ ] T002 [P] Create /rag/.gitignore with .env, __pycache__/, *.pyc, venv/, .vercel/
- [ ] T003 [P] Create /rag/requirements.txt with FastAPI==0.104.1, uvicorn[standard]==0.24.0, google-generativeai==0.3.1, qdrant-client==1.7.0, python-dotenv==1.0.0, tiktoken==0.5.2, pydantic==2.5.0, aiohttp==3.9.1, tqdm==4.66.1
- [ ] T004 [P] Create /rag/config.env.example with template for GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION, SERVER_PORT, CORS_ORIGINS, TOP_K_RESULTS, SIMILARITY_THRESHOLD, CHUNK_SIZE, CHUNK_OVERLAP
- [ ] T005 [P] Create /rag/vercel.json with Python runtime config, routes for /api/query and /api/health, environment variable references
- [ ] T006 [P] Create /rag/README.md with setup instructions, API usage, deployment guide

**Checkpoint**: Backend directory structure ready for implementation

---

## Phase 2: Foundational (Document Ingestion & Embedding)

**Purpose**: Core ingestion pipeline that MUST be complete before RAG functionality can work

**⚠️ CRITICAL**: No user story work can begin until document embedding is complete

- [ ] T007 Create /rag/ingest.py with import statements (os, pathlib, typing, argparse, uuid, time, google.generativeai, qdrant_client, tiktoken, tqdm, dotenv)
- [ ] T008 [P] Implement load_documents() function in /rag/ingest.py to recursively read all markdown files from /docs using pathlib.Path.rglob("*.md")
- [ ] T009 [P] Implement extract_chapter() helper function in /rag/ingest.py to extract chapter name from directory structure
- [ ] T010 [P] Implement split_by_headers() function in /rag/ingest.py to split markdown content by ## and ### headers using regex
- [ ] T011 [P] Implement chunk_document() function in /rag/ingest.py with sliding window (500 tokens, 150 overlap) using tiktoken tokenizer
- [ ] T012 [P] Implement generate_embeddings() function in /rag/ingest.py with Gemini API batch calls (10 chunks/batch), rate limiting (1s delay), retry logic (3 attempts with exponential backoff)
- [ ] T013 [P] Implement init_qdrant_collection() function in /rag/ingest.py to create collection with VectorParams(size=768, distance=COSINE)
- [ ] T014 [P] Implement upload_to_qdrant() function in /rag/ingest.py with batch uploads (100 points/batch) using PointStruct with UUID, vector, and payload
- [ ] T015 Implement main() function in /rag/ingest.py with argparse CLI (--docs-dir, --collection, --batch-size, --recreate flags) and orchestrate load → chunk → embed → upload pipeline
- [ ] T016 Add __name__ == "__main__" block in /rag/ingest.py to call main()

**Checkpoint**: Ingestion pipeline ready - can now populate Qdrant with document embeddings

---

## Phase 3: User Story 1 - Ask a Question (Normal RAG Mode) (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions and receive accurate answers based on semantic search across all textbook content

**Independent Test**: Submit a question without selected text → verify response contains accurate information from /docs markdown files with source citations

### Implementation for User Story 1 (Backend)

- [ ] T017 [P] [US1] Create /rag/rag_server.py with FastAPI app initialization, CORS middleware configuration allowing CORS_ORIGINS from env
- [ ] T018 [P] [US1] Add Pydantic models in /rag/rag_server.py: QueryRequest(question: str, selectedText: Optional[str]), Source(text: str, source: str, score: float), QueryResponse(answer: str, mode: str, sources: List[Source], response_time_ms: int)
- [ ] T019 [P] [US1] Initialize Gemini client in /rag/rag_server.py with genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
- [ ] T020 [P] [US1] Initialize Qdrant client in /rag/rag_server.py with QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
- [ ] T021 [P] [US1] Implement embed_query() async function in /rag/rag_server.py to embed user question using Gemini models/embedding-001 with task_type="retrieval_query"
- [ ] T022 [P] [US1] Implement search_qdrant() async function in /rag/rag_server.py to search collection with embedding, top_k=5, score_threshold=0.7, return list of {text, source, score}
- [ ] T023 [P] [US1] Implement generate_response() async function in /rag/rag_server.py using Gemini models/gemini-pro with context and question in prompt
- [ ] T024 [US1] Implement POST /api/query endpoint in /rag/rag_server.py for Normal RAG mode: validate question → embed query → search Qdrant → assemble context from top results → generate response via Gemini → return QueryResponse with sources
- [ ] T025 [P] [US1] Add error handling in POST /api/query endpoint: 400 for empty question, 429 for rate limits, 500 for server errors, 503 for Qdrant/Gemini unavailable
- [ ] T026 [P] [US1] Implement GET /api/health endpoint in /rag/rag_server.py to check Qdrant connection and return {status: "healthy", qdrant_connected: bool}

### Implementation for User Story 1 (Frontend)

- [ ] T027 [P] [US1] Create /src/components/RagChat directory
- [ ] T028 [P] [US1] Create /src/components/RagChat/config.js with getBackendUrl() function returning http://localhost:8000 for dev, REACT_APP_RAG_BACKEND_URL env var for prod
- [ ] T029 [P] [US1] Create /src/components/RagChat/ChatKitWrapper.jsx with imports (React, useState, ChatKit components from @chatscope/chat-ui-kit-react, config)
- [ ] T030 [US1] Implement state management in ChatKitWrapper.jsx: messages array, isLoading bool, error string state using useState hooks
- [ ] T031 [US1] Implement handleSendMessage() function in ChatKitWrapper.jsx to add user message to state, call fetch(`${backendUrl}/api/query`) with POST, handle response/errors, add assistant message with metadata
- [ ] T032 [US1] Implement ChatKit UI in ChatKitWrapper.jsx: MainContainer > ChatContainer > MessageList (with messages mapped to Message components) > MessageInput with onSend={handleSendMessage}
- [ ] T033 [US1] Add loading indicator (TypingIndicator) in ChatKitWrapper.jsx MessageList when isLoading=true
- [ ] T034 [US1] Display source citations in Message.Footer in ChatKitWrapper.jsx when msg.metadata.sources exists and has length > 0
- [ ] T035 [P] [US1] Create /src/components/RagChat/index.jsx to export RagChat component wrapping ChatKitWrapper with props spread
- [ ] T036 [P] [US1] Create /src/components/RagChat/styles.module.css with .chatContainer (fixed, bottom-right, 400px x 600px, z-index 1000), .sources (11px, gray), responsive media query for mobile
- [ ] T037 [P] [US1] Create /static/ragchat.css with ChatKit style overrides: .cs-message-list max-height, .cs-message--incoming/.cs-message--outgoing background colors

**Checkpoint**: User Story 1 complete - Normal RAG mode fully functional and testable independently

---

## Phase 4: User Story 2 - Ask a Question with Selected Text (Priority: P1)

**Goal**: Enable users to select specific text from textbook and ask focused questions on that selection

**Independent Test**: Highlight text on page, submit question with selection → verify response uses only selected text as context, not Qdrant results

### Implementation for User Story 2 (Backend)

- [ ] T038 [US2] Add Selected Text mode logic to POST /api/query endpoint in /rag/rag_server.py: detect if request.selectedText exists and is non-empty → set mode="selected_text", use selectedText as context, skip embed_query() and search_qdrant(), return QueryResponse with empty sources list
- [ ] T039 [US2] Validate selectedText length (max 10000 chars) in POST /api/query endpoint in /rag/rag_server.py

### Implementation for User Story 2 (Frontend)

- [ ] T040 [P] [US2] Add selectedText state (string | null) to ChatKitWrapper.jsx using useState
- [ ] T041 [P] [US2] Create /src/components/RagChat/TextSelectionHandler.js with useTextSelection(onTextSelected) hook using useEffect to listen for mouseup/touchend events
- [ ] T042 [US2] Implement selection detection in TextSelectionHandler.js: get window.getSelection(), filter selections from .chat-container, call onTextSelected(text) for valid selections
- [ ] T043 [US2] Import and call useTextSelection hook in ChatKitWrapper.jsx with callback to setSelectedText(text)
- [ ] T044 [US2] Pass selectedText to backend API call in handleSendMessage() in ChatKitWrapper.jsx
- [ ] T045 [US2] Clear selectedText after message sent in handleSendMessage() in ChatKitWrapper.jsx with setSelectedText(null)
- [ ] T046 [US2] Display selected text indicator UI in ChatKitWrapper.jsx when selectedText exists: show div with label, truncated text (50 chars), clear button (✕)
- [ ] T047 [P] [US2] Add .selectedTextIndicator styles to /src/components/RagChat/styles.module.css: flex layout, blue left border, padding, .label bold, .clearButton styling

**Checkpoint**: User Story 2 complete - Selected Text mode fully functional and testable independently

---

## Phase 5: Integration & Deployment Instructions

**Purpose**: Documentation for manual integration and deployment steps

- [ ] T048 Create /rag/INTEGRATION.md with section "Integrate RagChat Component into Docusaurus": instructions to create /src/theme/Root.js, import RagChat from '@site/src/components/RagChat', add <RagChat enableTextSelection={true} /> after {children}, note this is optional and reversible
- [ ] T049 Add "Install ChatKit Dependencies" section to /rag/INTEGRATION.md: npm install @chatscope/chat-ui-kit-react @chatscope/chat-ui-kit-styles command
- [ ] T050 Add "Load CSS" section to /rag/INTEGRATION.md: import '@chatscope/chat-ui-kit-styles/dist/default/styles.min.css' in ChatKitWrapper.jsx already handles ChatKit styles, /static/ragchat.css loaded automatically by Docusaurus static assets
- [ ] T051 Create "Local Ingestion" section in /rag/README.md: cd /rag → python -m venv venv → source venv/bin/activate → pip install -r requirements.txt → cp config.env.example .env (fill real API keys) → python ingest.py --docs-dir ../docs --collection robotics_textbook_chunks
- [ ] T052 Create "Deploy Backend to Vercel" section in /rag/README.md: npm install -g vercel → vercel login → vercel link → set environment variables in Vercel dashboard (GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION, CORS_ORIGINS) → vercel --prod → test with curl https://your-app.vercel.app/api/health
- [ ] T053 Create "Environment Variables" section in /rag/README.md: document all required variables from config.env.example with descriptions, note to never commit .env to Git, add .env to .gitignore
- [ ] T054 Add "Frontend Deployment" section to /rag/INTEGRATION.md: create .env.production with REACT_APP_RAG_BACKEND_URL=https://your-app.vercel.app → npm run build → npm run deploy (GitHub Pages)

---

## Phase 6: Testing & Validation

**Purpose**: Validate all functionality per user requirements (E. Testing Tasks)

### Test User Story 1 (Normal RAG Mode)

- [ ] T055 [US1] Test Normal RAG mode: run python /rag/rag_server.py locally → send curl POST to localhost:8000/api/query with {"question": "What is ROS 2?"} → verify response has mode="normal_rag", sources array with 3-5 chunks, answer contains ROS 2 information
- [ ] T056 [US1] Test Qdrant search: verify /rag/ingest.py successfully populated Qdrant collection → check Qdrant dashboard shows correct point count → test search returns relevant chunks with scores > 0.7

### Test User Story 2 (Selected Text Mode)

- [ ] T057 [US2] Test Selected Text mode: send curl POST to localhost:8000/api/query with {"question": "Explain this", "selectedText": "ROS 2 uses DDS middleware..."} → verify response has mode="selected_text", sources=[], answer references provided text not Qdrant results
- [ ] T058 [US2] Test text selection in UI: open localhost:3000 → highlight text on page → verify selected text indicator appears in chat widget → submit question → verify selectedText sent in network request

### Test ChatKit Interface

- [ ] T059 Test ChatKit UI: open localhost:3000 → verify chat widget appears bottom-right → send test question → verify user message appears → verify loading indicator shows → verify assistant response appears with sources footer → check browser console for errors
- [ ] T060 Test error handling: stop backend server → send question in UI → verify error message displays "Sorry, I encountered an error" → restart server → verify subsequent queries work

### Integration & Performance Tests

- [ ] T061 Test end-to-end flow: start backend (uvicorn rag_server:app --reload --port 8000) → start frontend (npm start) → submit Normal RAG query → verify <5s response time → submit Selected Text query → verify <3s response time
- [ ] T062 Test deployment: verify Vercel backend accessible at production URL → verify health endpoint returns 200 → test query endpoint from deployed frontend → verify GitHub Pages shows chat widget and responds correctly

---

## Phase 7: Polish & Documentation

**Purpose**: Final cleanup and comprehensive documentation

- [ ] T063 [P] Add comprehensive docstrings to all functions in /rag/ingest.py with param types, return types, and descriptions
- [ ] T064 [P] Add comprehensive docstrings to all functions in /rag/rag_server.py with FastAPI automatic documentation
- [ ] T065 [P] Add code comments explaining dual-mode logic in /rag/rag_server.py POST /api/query endpoint
- [ ] T066 [P] Add JSDoc comments to React components in /src/components/RagChat/ChatKitWrapper.jsx
- [ ] T067 Update /rag/README.md with troubleshooting section: common issues (API key errors, Qdrant connection failures, CORS errors, rate limiting), solutions for each
- [ ] T068 Add "Architecture Overview" section to /rag/README.md: ASCII diagram showing user → ChatKit → FastAPI → Gemini/Qdrant flow for both modes
- [ ] T069 Verify constitution compliance: run git status → confirm no /docs/ files modified → confirm all RAG code in /rag/ and /src/components/RagChat/ only → confirm .gitignore prevents .env commit

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (Phase 1) - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational (Phase 2) - Can proceed independently
- **User Story 2 (Phase 4)**: Depends on User Story 1 (Phase 3) - Extends US1 backend endpoint and frontend component
- **Integration (Phase 5)**: Depends on US1 and US2 completion - Documentation only
- **Testing (Phase 6)**: Depends on US1 and US2 completion - Validates all functionality
- **Polish (Phase 7)**: Depends on testing completion - Final improvements

### User Story Dependencies

- **User Story 1 (P1)**: Independent after Foundational phase - Normal RAG mode standalone
- **User Story 2 (P1)**: Depends on US1 - Adds Selected Text mode to existing backend/frontend

**Note**: US2 extends US1 rather than being fully independent due to shared backend endpoint and frontend component

### Within Each User Story

**User Story 1 Flow**:
1. Backend models (T018) before backend functions (T021-T023)
2. Backend endpoint (T024) depends on all backend functions
3. Frontend config (T028) independent of backend
4. Frontend component (T029-T037) can proceed in parallel with backend
5. Integration happens when both backend and frontend complete

**User Story 2 Flow**:
1. Backend mode logic (T038-T039) extends existing endpoint from US1
2. Frontend selection handler (T041-T042) independent
3. Frontend UI updates (T043-T047) integrate handler with existing component

### Parallel Opportunities

**Phase 1 Setup** (all parallel):
- T002, T003, T004, T005, T006 can all run in parallel (different files)

**Phase 2 Foundational** (many parallel):
- T008, T009, T010, T011, T012, T013, T014 can all run in parallel (different functions, same file but independent)
- T007 and T015-T016 must be sequential (main file structure)

**Phase 3 User Story 1 Backend** (many parallel):
- T017, T018, T019, T020, T021, T022, T023, T026 can all run in parallel (different sections/functions)
- T024-T025 must follow backend functions (depends on T021-T023)

**Phase 3 User Story 1 Frontend** (many parallel):
- T027, T028, T029, T035, T036, T037 can all run in parallel (different files)
- T030-T034 must be sequential within ChatKitWrapper.jsx (depends on T029)

**Phase 4 User Story 2 Frontend** (some parallel):
- T040, T041, T046, T047 can run in parallel (different parts of component/different files)
- T042-T045 must be sequential (builds on previous tasks)

**Phase 7 Polish** (all parallel):
- T063, T064, T065, T066, T067, T068, T069 can all run in parallel (different files/sections)

---

## Parallel Example: User Story 1 Backend

```bash
# Launch backend infrastructure tasks together:
Task: "Add Pydantic models in /rag/rag_server.py" (T018)
Task: "Initialize Gemini client in /rag/rag_server.py" (T019)
Task: "Initialize Qdrant client in /rag/rag_server.py" (T020)
Task: "Implement embed_query() in /rag/rag_server.py" (T021)
Task: "Implement search_qdrant() in /rag/rag_server.py" (T022)
Task: "Implement generate_response() in /rag/rag_server.py" (T023)
```

## Parallel Example: User Story 1 Frontend

```bash
# Launch frontend component files together:
Task: "Create config.js" (T028)
Task: "Create index.jsx" (T035)
Task: "Create styles.module.css" (T036)
Task: "Create /static/ragchat.css" (T037)

# Then launch ChatKitWrapper.jsx implementation:
Task: "Create ChatKitWrapper.jsx with imports" (T029)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T016) - CRITICAL ingestion pipeline
3. Complete Phase 3: User Story 1 (T017-T037) - Normal RAG mode
4. **STOP and VALIDATE**: Run T055-T056 tests
5. Demo Normal RAG mode working end-to-end

**MVP Deliverable**: Functional RAG chatbot with semantic search across textbook content

### Incremental Delivery

1. MVP (US1) → Deploy backend to Vercel → Test locally → Validate
2. Add US2 (T038-T047) → Test Selected Text mode → Validate both modes work
3. Complete Integration docs (T048-T054) → Deploy to production
4. Run all tests (T055-T062) → Fix any issues → Validate
5. Polish (T063-T069) → Final review → Ship

### Sequential Strategy (Single Developer)

1. Setup (1 hour) → Foundational (3-4 hours including ingestion run)
2. User Story 1 Backend (2-3 hours) → User Story 1 Frontend (2-3 hours) → Test US1 (1 hour)
3. User Story 2 Backend (1 hour) → User Story 2 Frontend (1-2 hours) → Test US2 (1 hour)
4. Integration docs (1 hour) → Full testing (1-2 hours) → Polish (1-2 hours)

**Total Estimate**: 16-22 hours for complete implementation

---

## Task Validation

### Format Checklist

✅ All tasks follow format: `- [ ] [ID] [P?] [Story?] Description with file path`
✅ Task IDs sequential (T001-T069)
✅ [P] markers only on parallelizable tasks (different files, no dependencies)
✅ [Story] labels only on user story phases (US1, US2)
✅ All descriptions include specific file paths
✅ No tasks missing required components

### Completeness Checklist

✅ User Story 1 has all components: backend models, functions, endpoint, frontend components, styles
✅ User Story 2 has all components: backend mode logic, frontend selection handler, UI updates
✅ Each user story independently testable per spec.md acceptance criteria
✅ All files from plan.md structure included (/rag/ingest.py, /rag/rag_server.py, /rag/requirements.txt, /rag/vercel.json, /rag/config.env.example, /src/components/RagChat/*.jsx/js/css, /static/ragchat.css)
✅ All testing requirements covered (T055-T062)
✅ Deployment instructions included (T048-T054)
✅ Constitution compliance validated (T069)

### Suggested MVP Scope

**Minimum Viable Product** = Phase 1 (Setup) + Phase 2 (Foundational) + Phase 3 (User Story 1)

**Tasks**: T001-T037 + T055-T056 (testing)

**Deliverable**: Functional RAG chatbot with Normal RAG mode, deployed backend, working frontend

**Value**: Users can ask questions and get accurate answers from textbook content

**Validation**: Submit "What is ROS 2?" → receive accurate response with source citations in <5 seconds

---

## Notes

- **[P] tasks** = different files, no dependencies on incomplete tasks in same phase
- **[Story] labels** map tasks to user stories for traceability (US1 = Normal RAG, US2 = Selected Text)
- Each user story independently completable and testable per spec.md
- Foundational phase (T007-T016) MUST complete before any user story work
- User Story 2 extends User Story 1 (shared endpoint/component) but adds independent functionality
- Stop at any checkpoint to validate story independently
- Integration is manual (T048-T054) - no code modifications to existing Docusaurus files per constitution
- All RAG code isolated in /rag/ and /src/components/RagChat/ per constitution Principle VII
- Tests included per user requirement (E. Testing Tasks)
- Deployment is manual setup (not automated CI/CD) per user requirement (D. Deployment Tasks)

**Total Tasks**: 69
**User Story 1 Tasks**: 21 implementation + 2 tests = 23
**User Story 2 Tasks**: 10 implementation + 2 tests = 12
**Setup/Foundational**: 16
**Integration/Deployment**: 7
**Testing**: 8 (includes US-specific tests)
**Polish**: 7
