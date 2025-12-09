---
id: 0001
title: RAG chatbot detailed specification
stage: spec
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: anaslinux
command: /sp.specify
labels: ["specification", "rag-chatbot", "architecture", "gemini", "qdrant", "fastapi", "chatkit"]
links:
  spec: /specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/spec.md
  - specs/001-rag-chatbot/checklists/requirements.md
tests:
  - Quality validation checklist (all items passed)
---

## Prompt

Convert the RAG constitution into a detailed full specification.

Include:

1. Architecture Overview
   - Gemini embeddings + Gemini chat generation
   - Qdrant vector DB design (collection name, payload schema, embedding dims)
   - FastAPI backend flow (Normal RAG vs Selected-Text mode)
   - Frontend ChatKit SDK architecture

2. Data Flow
   a) Normal RAG mode:
      user question → embed → Qdrant search → context → Gemini → response
   b) Selected text mode:
      user question + selectedText → skip Qdrant → Gemini → response

3. Chunking Strategy
   - Read /docs markdown
   - 500-token chunks, 150-token overlap
   - Metadata for each chunk

4. Detailed File Specifications for /rag
   - ingest.py
   - rag_server.py
   - requirements.txt
   - vercel.json
   - config.env.example

5. FastAPI Endpoint Specification
   POST /query
   Body:
     {
       question: "...",
       selectedText?: "..."
     }
   Logic:
     IF selectedText → skip Qdrant
     ELSE → normal RAG

6. ChatKit Frontend Specification
   - React component calling backend
   - ChatKit SDK for UI
   - Selected-text support in UI
   - How to wire backend URL

7. Deployment Specification
   - Vercel serverless setup
   - Environment setup
   - Build & deploy flow

8. Safety Boundaries
   - Never modify existing textbook files
   - All RAG code isolated
   - ChatKit integration only through a new component

Produce a complete specification suitable for planning and implementation.

## Response snapshot

Successfully created comprehensive specification for RAG chatbot integration:

**Specification Highlights**:
- Expanded existing spec from basic requirements to ~1475 lines of detailed guidance
- Added complete architecture overview with ASCII diagrams showing 4-layer system (Frontend, Backend, Vector Storage, LLM)
- Detailed data flow specifications for both Normal RAG and Selected Text modes with step-by-step processes
- Comprehensive chunking strategy (500 tokens, 150 overlap, metadata extraction)
- Complete file specifications for all /rag directory files (ingest.py, rag_server.py, requirements.txt, vercel.json, config.env.example)
- Full FastAPI endpoint contracts with request/response schemas, error handling, and processing logic
- ChatKit frontend integration with component structure, state management, and example implementation
- Deployment specification covering Vercel setup, environment configuration, and CI/CD pipeline
- Safety boundaries enforcing constitution's isolation principle

**Quality Validation**:
- Created requirements checklist at specs/001-rag-chatbot/checklists/requirements.md
- All 14 checklist items passed validation
- No [NEEDS CLARIFICATION] markers remain
- All requirements testable and unambiguous
- Success criteria measurable and technology-agnostic
- Specification ready for planning phase

**Technical Decisions**:
- Gemini embedding-001 (768-dim vectors)
- Gemini gemini-pro for chat
- Qdrant collection: "robotics_textbook_chunks"
- FastAPI with dual-mode RAG logic
- OpenAI ChatKit SDK for UI only (not LLM calls)
- Vercel serverless deployment

## Outcome

- ✅ Impact: Complete RAG chatbot specification created from constitution, ready for architectural planning
- 🧪 Tests: Quality checklist validation passed (14/14 items)
- 📁 Files: specs/001-rag-chatbot/spec.md (expanded), specs/001-rag-chatbot/checklists/requirements.md (created)
- 🔁 Next prompts: Run `/sp.plan` to create detailed technical architecture and implementation plan
- 🧠 Reflection: Successfully transformed constitutional principles into actionable specification with comprehensive architecture, data flows, file specifications, and deployment guidance while maintaining isolation boundaries

## Evaluation notes (flywheel)

- Failure modes observed: None - all requirements specified with reasonable defaults, no clarifications needed
- Graders run and results (PASS/FAIL): Quality checklist - PASS (all 14 items)
- Prompt variant (if applicable): Standard /sp.specify workflow with detailed constitution input
- Next experiment (smallest change to try): Proceed to planning phase to validate architecture assumptions against actual implementation constraints
