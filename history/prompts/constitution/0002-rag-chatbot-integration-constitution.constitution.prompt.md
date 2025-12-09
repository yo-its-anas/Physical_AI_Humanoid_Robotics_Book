---
id: 0002
title: RAG chatbot integration constitution
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-ai-robotics-textbook
user: anaslinux
command: /sp.constitution
labels: ["constitution", "rag-system", "gemini-api", "qdrant", "fastapi", "chatkit", "isolation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - n/a
---

## Prompt

Create a project constitution for integrating a RAG (Retrieval-Augmented Generation) chatbot into my existing Docusaurus textbook website.

Project Requirements:
- NEVER modify or overwrite any existing textbook files inside /docs or core Docusaurus files.
- All RAG code must be created ONLY inside a new top-level folder: /rag/
- Use Gemini API (FREE) for embeddings and LLM responses.
- Use Qdrant Cloud Free Tier for vector storage.
- Use FastAPI for the backend.
- Deployment must be compatible with Vercel serverless functions.
- The chatbot must have two modes:
    1) Normal RAG mode → embed user query → retrieve from Qdrant → answer via Gemini.
    2) Selected-Text mode → if user sends highlighted text, skip Qdrant and answer ONLY from that selected text.

Frontend Requirement:
- Utilize the **OpenAI ChatKit SDK** for the chat interface UI, but connect it to my custom FastAPI backend (Gemini + Qdrant). ChatKit should only be used for the frontend chat component, not for LLM calls.

Tools Allowed:
- Claude Code filesystem MCP for generating files.
- Claude terminal MCP for commands (python, npm, git).
- NO other MCP integrations.
- MUST preserve my existing GPT-generated textbook structure.

Claude Must Generate:
- /rag/ingest.py
- /rag/rag_server.py
- /rag/requirements.txt
- /rag/vercel.json
- /rag/config.env.example
- Frontend ChatKit UI integration:
    - /src/components/RagChat/index.jsx
    - /src/components/RagChat/ChatKitWrapper.js
    - /static/ragchat.css

Goal:
Create a complete, safe, isolated RAG system connected to the Docusaurus site, fully satisfying the "OpenAI Agents/ChatKit SDKs, FastAPI, Qdrant" requirement without modifying existing book content.

## Response snapshot

Updated constitution to version 1.1.0 (MINOR bump). Added three new principles:

**Principle VII: RAG System Isolation & Safety** - Mandates complete isolation of RAG code in `/rag/` directory, zero modifications to existing `/docs/` content, and preservation of GPT-generated textbook structure.

**Principle VIII: RAG Technical Stack & Architecture** - Specifies Gemini API (free) for LLM/embeddings, Qdrant Cloud (free) for vectors, FastAPI for backend, ChatKit for UI only (not LLM calls), and Vercel serverless deployment.

**Principle IX: Dual-Mode RAG Interaction** - Defines Normal RAG mode (semantic search via Qdrant) and Selected-Text mode (direct explanation without retrieval), with automatic mode detection.

Expanded Development Tools & Technologies section to include RAG System Stack. Added RAG System Requirements subsection with mandatory file structure, non-functional requirements (isolation, performance <3s, free-tier cost, 100 concurrent users), and explicit integration points.

Updated Governance section with compliance review checkpoints to enforce isolation principle at each development stage.

All existing textbook principles (I-VI) preserved unchanged. Templates validated as compatible with no required updates.

## Outcome

- ✅ Impact: Constitution amended from v1.0.0 to v1.1.0 with three new RAG-specific principles while preserving all existing textbook principles
- 🧪 Tests: n/a (constitutional document)
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: `/sp.specify` to create RAG chatbot feature specification, `/sp.plan` to design RAG architecture
- 🧠 Reflection: MINOR version appropriate—new principles added without breaking existing governance. Isolation principle critical for protecting months of textbook content work.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Manual validation PASS—no bracket placeholders, dates ISO format, version incremented correctly, principles declarative and testable
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): Monitor compliance during `/sp.tasks` to validate isolation boundaries hold
