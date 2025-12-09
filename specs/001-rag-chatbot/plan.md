# Implementation Plan: RAG Chatbot Integration

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot/spec.md`

## Summary

Build an intelligent RAG chatbot that enhances the AI & Robotics textbook with semantic search and contextual Q&A capabilities. The system uses Gemini API for embeddings and chat, Qdrant Cloud for vector storage, FastAPI for the backend, and ChatKit SDK for the frontend UI. It supports dual modes: Normal RAG (semantic search across all content) and Selected-Text mode (focused explanations of highlighted passages). All RAG code is isolated in `/rag/` to preserve existing textbook integrity.

**Technical Approach**:
1. Ingest `/docs` markdown files → chunk (500 tokens, 150 overlap) → embed via Gemini → store in Qdrant
2. Backend FastAPI server detects mode (selectedText present or not) and routes accordingly
3. Frontend ChatKit component captures user input + optional selected text → calls backend → displays response
4. Deploy backend as Vercel serverless function, frontend integrated into existing Docusaurus site

## Technical Context

**Language/Version**: Python 3.9+, JavaScript (React via Docusaurus), Node.js 18+
**Primary Dependencies**:
- Backend: FastAPI 0.104+, google-generativeai 0.3+, qdrant-client 1.7+, uvicorn 0.24+
- Frontend: @chatscope/chat-ui-kit-react, React 18+ (Docusaurus), axios/fetch
**Storage**: Qdrant Cloud (vector DB, free tier 1GB), no SQL database required
**Testing**:
- Backend: pytest, manual API testing (curl/Postman)
- Frontend: Manual UI testing, browser console validation
- Integration: End-to-end flow testing (question → response)
**Target Platform**:
- Backend: Vercel serverless (Python runtime)
- Frontend: Browser (Chrome/Firefox/Safari), integrated with Docusaurus static site
**Project Type**: Web application (backend API + frontend React components)
**Performance Goals**:
- P95 response time <5s for Normal RAG mode
- P95 response time <3s for Selected-Text mode
- Support 100 concurrent users on Vercel free tier
- Embedding batch processing: 10 chunks/batch
**Constraints**:
- Free-tier only (Gemini API: 60 req/min, Qdrant: 1GB storage)
- Zero modifications to `/docs/` content files
- All RAG code isolated in `/rag/` directory
- No secrets committed to Git
**Scale/Scope**:
- Estimated 200-300 markdown files in `/docs`
- Estimated 1,500-2,500 chunks total
- Expected usage: 50-100 queries/day (development/educational use)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Principle VII: RAG System Isolation & Safety
- **Requirement**: All RAG code in `/rag/`, zero modifications to `/docs/`, minimal integration points
- **Compliance**:
  - ✅ All backend code in `/rag/` (ingest.py, rag_server.py, requirements.txt, vercel.json, config.env.example)
  - ✅ Frontend components in `/src/components/RagChat/` (new directory, isolated)
  - ✅ Styles in `/static/ragchat.css` (new file)
  - ✅ No `/docs/` modifications planned
  - ✅ Optional integration: `<RagChat />` import in layout (reversible)
- **Status**: PASS

### ✅ Principle VIII: RAG Technical Stack & Architecture
- **Requirement**: Gemini API, Qdrant Cloud, FastAPI, ChatKit SDK (UI only), Vercel
- **Compliance**:
  - ✅ Gemini API for embeddings (models/embedding-001) and chat (models/gemini-pro)
  - ✅ Qdrant Cloud for vector storage (collection: "robotics_textbook_chunks")
  - ✅ FastAPI for backend server
  - ✅ ChatKit SDK for frontend UI (custom backend, not OpenAI)
  - ✅ Vercel serverless deployment
- **Status**: PASS

### ✅ Principle IX: Dual-Mode RAG Interaction
- **Requirement**: Normal RAG mode + Selected-Text mode, automatic mode detection
- **Compliance**:
  - ✅ Normal RAG: question → embed → Qdrant → context → Gemini → response
  - ✅ Selected-Text: question + text → skip Qdrant → Gemini → response
  - ✅ Mode detection via presence of `selectedText` in request
- **Status**: PASS

### Additional Constitution Alignment

- **Principle III (AI-Native Development)**: Using Claude Code + Spec-Kit Plus for implementation ✅
- **Principle IV (Docusaurus)**: Frontend integrates with existing Docusaurus site ✅
- **Principle V (Code Quality)**: Python code will use type hints, error handling, and docstrings ✅

**Overall Constitution Compliance**: ✅ PASS - All principles satisfied, no violations

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot/
├── spec.md              # Feature specification (complete)
├── plan.md              # This file (implementation plan)
├── research.md          # Phase 0 output (technology research)
├── data-model.md        # Phase 1 output (data structures)
├── quickstart.md        # Phase 1 output (setup guide)
├── contracts/           # Phase 1 output (API schemas)
│   └── api-schema.yaml  # OpenAPI spec for FastAPI endpoints
├── checklists/          # Quality validation
│   └── requirements.md  # Requirements checklist (complete)
└── tasks.md             # Phase 2 output (/sp.tasks - not yet created)
```

### Source Code (repository root)

```text
# RAG Backend (isolated)
/rag/
├── ingest.py              # Document ingestion & embedding script
├── rag_server.py          # FastAPI backend (dual-mode RAG)
├── requirements.txt       # Python dependencies (pinned versions)
├── vercel.json           # Vercel serverless deployment config
├── config.env.example    # Environment variable template
├── README.md             # Backend setup and usage docs
└── tests/                # Backend tests (optional, future)
    ├── test_ingest.py
    └── test_rag_server.py

# Frontend Integration (new, isolated)
/src/components/RagChat/
├── index.jsx             # Main export component
├── ChatKitWrapper.jsx    # ChatKit SDK integration
├── TextSelectionHandler.js # Text selection logic
├── config.js             # Backend URL configuration
└── styles.module.css     # Component-specific styles

# Styles (new file)
/static/
└── ragchat.css          # Global chat UI styles

# Existing structure (NO MODIFICATIONS)
/docs/                   # Textbook content (protected)
/src/pages/              # Docusaurus pages (protected)
/docusaurus.config.js    # Main config (minimal changes only if needed)
```

**Structure Decision**: Web application with isolated backend and frontend components. Backend lives entirely in `/rag/` to enforce constitution isolation principle. Frontend components isolated in `/src/components/RagChat/` as new directory. This structure allows complete RAG system to be added/removed without affecting existing textbook infrastructure.

## Complexity Tracking

*No constitutional violations - this section not required.*

---

## Phase 0: Research & Technology Validation

### Objectives
- Validate Gemini API integration approach
- Confirm Qdrant Cloud setup and SDK usage
- Research FastAPI best practices for serverless deployment
- Investigate ChatKit SDK customization for custom backend
- Determine chunking strategy implementation details

### Research Tasks

**Research deliverables will be documented in `research.md`**

1. **Gemini API Research**
   - Confirm `models/embedding-001` produces 768-dim vectors
   - Verify `models/gemini-pro` context window (32k tokens)
   - Test batch embedding API (error handling, rate limits)
   - Document free-tier limits (60 req/min, 1500 req/day)
   - Example code for embedding and chat generation

2. **Qdrant Cloud Research**
   - Document collection creation API
   - Verify HNSW index configuration (cosine similarity)
   - Test vector search with top-k and score threshold
   - Confirm payload schema flexibility
   - Example code for collection init and vector search

3. **FastAPI + Vercel Research**
   - Verify Vercel Python runtime supports FastAPI
   - Document `vercel.json` configuration for serverless functions
   - Research CORS handling for cross-origin requests
   - Investigate cold start optimization (keep-alive patterns)
   - Example minimal FastAPI app deployed to Vercel

4. **ChatKit SDK Research**
   - Confirm ChatKit can use custom backend (not OpenAI)
   - Document message state management in React
   - Research text selection API in browser (window.getSelection)
   - Identify styling customization options
   - Example ChatKit component with custom API

5. **Chunking Strategy Research**
   - Research tiktoken or similar tokenizers for Gemini
   - Investigate markdown-aware chunking (preserve headers)
   - Determine optimal metadata extraction from file paths
   - Explore sliding window implementation (500 tokens, 150 overlap)
   - Example chunking function with metadata

### Research Outcomes (to be documented in research.md)

For each research task, document:
- **Decision**: What approach was chosen
- **Rationale**: Why this approach is optimal
- **Alternatives Considered**: What else was evaluated and why rejected
- **Example Code**: Minimal working example
- **References**: Documentation links, tutorials, GitHub examples

---

## Phase 1: Design & Contracts

### 1.1 Data Model Design

**Deliverable**: `data-model.md`

#### Entities

**Chunk Entity** (stored in Qdrant)
```yaml
Chunk:
  id: UUID (auto-generated)
  vector: List[float] (768 dimensions)
  payload:
    text: string (chunk content, max ~600 tokens)
    source: string (file path, e.g., "/docs/module-1/intro.md")
    chapter: string (extracted from directory/headers)
    section: string (nearest markdown heading)
    start_token: integer (position in original doc)
    end_token: integer (position in original doc)
    token_count: integer (actual tokens in chunk)
    chunk_index: integer (sequential within document)
    total_chunks: integer (total from this document)
    file_modified: datetime (ISO 8601 format)
```

**Query Request Entity** (FastAPI input)
```yaml
QueryRequest:
  question: string (required, 1-2000 chars)
  selectedText: string | null (optional, max 10000 chars)
```

**Query Response Entity** (FastAPI output)
```yaml
QueryResponse:
  answer: string (generated response)
  mode: enum ("normal_rag" | "selected_text")
  sources: List[Source] (empty for selected_text mode)
  response_time_ms: integer

Source:
  text: string (chunk excerpt, first 200 chars)
  source: string (file path)
  score: float (similarity score, 0-1)
```

**Error Response Entity**
```yaml
ErrorResponse:
  error: string (error message)
  detail: string (additional context)
  status_code: integer (400, 429, 500, 503)
```

#### State Transitions

**Ingestion Flow**:
```
Markdown File → Read → Chunk → Embed → Store
                ↓       ↓        ↓       ↓
              Content  Chunks  Vectors Qdrant
```

**Query Flow (Normal RAG)**:
```
Question → Embed → Search → Retrieve → Generate → Response
   ↓         ↓       ↓         ↓          ↓         ↓
 String    Vector  Qdrant   Chunks    Gemini    JSON
```

**Query Flow (Selected Text)**:
```
Question + Text → Generate → Response
      ↓             ↓          ↓
    Strings      Gemini     JSON
```

#### Validation Rules

- **Chunk**:
  - `text`: Non-empty, 100-600 tokens
  - `vector`: Exactly 768 dimensions
  - `source`: Valid file path starting with "/docs/"
  - `token_count`: Matches actual tokenized length

- **QueryRequest**:
  - `question`: Non-empty string, strip whitespace
  - `selectedText`: If provided, non-empty after strip

- **QueryResponse**:
  - `answer`: Non-empty string
  - `mode`: Must be "normal_rag" or "selected_text"
  - `sources`: Empty list if mode is "selected_text"

### 1.2 API Contracts

**Deliverable**: `contracts/api-schema.yaml` (OpenAPI 3.0 spec)

**Endpoints**:

1. **POST /api/query**
   - Summary: Process user question with dual-mode RAG
   - Request Body: `QueryRequest` (JSON)
   - Responses:
     - 200: `QueryResponse` (success)
     - 400: `ErrorResponse` (bad request)
     - 429: `ErrorResponse` (rate limit)
     - 500: `ErrorResponse` (server error)
     - 503: `ErrorResponse` (service unavailable)

2. **GET /api/health**
   - Summary: Health check for monitoring
   - Responses:
     - 200: `{"status": "healthy", "qdrant_connected": bool}`
     - 503: `{"status": "unhealthy", "error": string}`

**Full OpenAPI schema will be generated in `contracts/api-schema.yaml`**

### 1.3 Quickstart Guide

**Deliverable**: `quickstart.md`

**Contents**:
1. Prerequisites (Python 3.9+, Node 18+, API keys)
2. Backend setup (virtualenv, pip install, .env config)
3. Run ingestion script (`python ingest.py --docs-dir ../docs`)
4. Start local FastAPI server (`uvicorn rag_server:app --reload`)
5. Frontend setup (npm install ChatKit, configure backend URL)
6. Test integration (curl examples, browser testing)
7. Deploy to Vercel (vercel CLI, environment variables)
8. Troubleshooting common issues

---

## Implementation Plan: Detailed Steps

### 1. Initial Setup Plan

**Objective**: Prepare external services and local project structure

#### 1.1 Qdrant Cloud Setup

**Steps**:
1. **Create Qdrant Cloud account** (https://cloud.qdrant.io)
   - Sign up with email (free tier)
   - Verify email address

2. **Create cluster**
   - Click "Create Cluster"
   - Name: `robotics-textbook-rag`
   - Region: Closest to your location
   - Tier: Free (1GB storage)
   - Wait for cluster provisioning (~2-3 minutes)

3. **Get API credentials**
   - Navigate to cluster dashboard
   - Copy **Cluster URL** (e.g., `https://xyz.qdrant.io`)
   - Copy **API Key** from settings
   - Store in password manager (never commit to Git)

4. **Verify connection** (local test):
   ```bash
   pip install qdrant-client
   python -c "
   from qdrant_client import QdrantClient
   client = QdrantClient(url='YOUR_URL', api_key='YOUR_KEY')
   print(client.get_collections())
   "
   ```

**Expected Output**: Empty collections list `{"collections": []}`

#### 1.2 Gemini API Key Creation

**Steps**:
1. **Access Google AI Studio** (https://makersuite.google.com/app/apikey)
   - Sign in with Google account

2. **Create API key**
   - Click "Get API Key"
   - Select existing project or create new
   - Click "Create API Key"
   - Copy key (starts with `AIza...`)
   - Store in password manager

3. **Verify API key** (local test):
   ```bash
   pip install google-generativeai
   python -c "
   import google.generativeai as genai
   genai.configure(api_key='YOUR_KEY')
   model = genai.GenerativeModel('gemini-pro')
   response = model.generate_content('Hello')
   print(response.text)
   "
   ```

**Expected Output**: Greeting response from Gemini

#### 1.3 /rag Folder Preparation

**Steps**:
1. **Create directory structure**:
   ```bash
   mkdir -p /rag
   cd /rag
   ```

2. **Create placeholder files**:
   ```bash
   touch ingest.py rag_server.py requirements.txt vercel.json config.env.example README.md
   ```

3. **Initialize .gitignore** (in `/rag/.gitignore`):
   ```
   .env
   __pycache__/
   *.pyc
   .pytest_cache/
   venv/
   .vercel/
   ```

4. **Create config.env.example**:
   ```bash
   # Gemini API Configuration
   GEMINI_API_KEY=your_gemini_api_key_here
   GEMINI_EMBEDDING_MODEL=models/embedding-001
   GEMINI_CHAT_MODEL=models/gemini-pro

   # Qdrant Configuration
   QDRANT_URL=https://your-cluster.qdrant.io
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_COLLECTION=robotics_textbook_chunks

   # Server Configuration
   SERVER_PORT=8000
   CORS_ORIGINS=http://localhost:3000,https://yourdomain.com

   # RAG Configuration
   TOP_K_RESULTS=5
   SIMILARITY_THRESHOLD=0.7
   CHUNK_SIZE=500
   CHUNK_OVERLAP=150
   ```

5. **Copy to .env and fill real values**:
   ```bash
   cp config.env.example .env
   # Edit .env with actual API keys
   ```

**Validation Checklist**:
- [ ] Qdrant cluster active and accessible
- [ ] Gemini API key works for embeddings and chat
- [ ] `/rag/` directory exists with all placeholder files
- [ ] `.env` file created with real credentials (not committed)
- [ ] `.gitignore` configured to protect secrets

---

### 2. Data Extraction & Chunking Plan

**Objective**: Implement `ingest.py` to process markdown files into embeddable chunks

#### 2.1 Document Reading Strategy

**Implementation Approach**:

```python
# ingest.py - Document Loading
import os
from pathlib import Path

def load_documents(docs_dir: str = "../docs") -> List[Dict]:
    """Recursively load all markdown files from docs directory."""
    documents = []
    docs_path = Path(docs_dir)

    for md_file in docs_path.rglob("*.md"):
        # Read file content
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract metadata from path
        relative_path = md_file.relative_to(docs_path)
        metadata = {
            "source": f"/docs/{relative_path}",
            "chapter": extract_chapter(relative_path),
            "file_modified": md_file.stat().st_mtime
        }

        documents.append({
            "content": content,
            "metadata": metadata
        })

    return documents
```

**Key Decisions**:
- Use `pathlib.Path.rglob()` for recursive markdown discovery
- UTF-8 encoding for markdown files
- Extract chapter from directory structure (e.g., `/docs/module-1/` → "Module 1")
- Record file modification time for incremental updates (future)

#### 2.2 Chunking Implementation

**Chunk Sizes & Overlap**:
- **Target size**: 500 tokens
- **Overlap**: 150 tokens
- **Step size**: 350 tokens (500 - 150)
- **Minimum chunk**: 100 tokens (discard smaller)
- **Maximum chunk**: 600 tokens (hard limit)

**Algorithm**:

```python
# ingest.py - Chunking Function
import tiktoken

def chunk_document(content: str, metadata: dict,
                   chunk_size: int = 500,
                   overlap: int = 150) -> List[Dict]:
    """Chunk document with sliding window and overlap."""
    # Initialize tokenizer (Gemini-compatible)
    tokenizer = tiktoken.get_encoding("cl100k_base")

    # Tokenize content
    tokens = tokenizer.encode(content)

    # Split on markdown headers first (preserve structure)
    sections = split_by_headers(content)

    chunks = []
    for section in sections:
        section_tokens = tokenizer.encode(section["text"])
        section_metadata = {**metadata, "section": section["heading"]}

        # If section fits in one chunk, keep as-is
        if len(section_tokens) <= chunk_size:
            if len(section_tokens) >= 100:  # Minimum threshold
                chunks.append({
                    "text": section["text"],
                    "metadata": {
                        **section_metadata,
                        "start_token": section["start"],
                        "end_token": section["start"] + len(section_tokens),
                        "token_count": len(section_tokens),
                        "chunk_index": 0,
                        "total_chunks": 1
                    }
                })
        else:
            # Apply sliding window
            step = chunk_size - overlap
            for i, start in enumerate(range(0, len(section_tokens), step)):
                end = min(start + chunk_size, len(section_tokens))
                chunk_tokens = section_tokens[start:end]

                if len(chunk_tokens) < 100:  # Skip tiny tail chunks
                    break

                chunk_text = tokenizer.decode(chunk_tokens)
                chunks.append({
                    "text": chunk_text,
                    "metadata": {
                        **section_metadata,
                        "start_token": section["start"] + start,
                        "end_token": section["start"] + end,
                        "token_count": len(chunk_tokens),
                        "chunk_index": i,
                        "total_chunks": (len(section_tokens) // step) + 1
                    }
                })

    return chunks
```

**Helper Functions**:

```python
def split_by_headers(content: str) -> List[Dict]:
    """Split markdown content by headers (##, ###)."""
    import re

    lines = content.split('\n')
    sections = []
    current_section = {"heading": "", "text": "", "start": 0}
    token_position = 0

    for line in lines:
        if re.match(r'^#{2,3}\s+', line):  # Header level 2-3
            # Save previous section
            if current_section["text"]:
                sections.append(current_section)

            # Start new section
            heading = line.strip('#').strip()
            current_section = {
                "heading": heading,
                "text": line + '\n',
                "start": token_position
            }
        else:
            current_section["text"] += line + '\n'

        token_position += len(tiktoken.get_encoding("cl100k_base").encode(line))

    # Add last section
    if current_section["text"]:
        sections.append(current_section)

    return sections
```

#### 2.3 Metadata Design

**Metadata Fields** (per chunk):

| Field | Type | Source | Example |
|-------|------|--------|---------|
| `source` | string | File path | `/docs/module-1/intro.md` |
| `chapter` | string | Directory name | `Module 1: ROS 2 Fundamentals` |
| `section` | string | Nearest ## heading | `1.1 Introduction` |
| `start_token` | integer | Token position | `0` |
| `end_token` | integer | Token position | `500` |
| `token_count` | integer | Calculated | `500` |
| `chunk_index` | integer | Sequence number | `0` (first chunk) |
| `total_chunks` | integer | Total from doc | `3` |
| `file_modified` | datetime | File stat | `2025-12-09T00:00:00Z` |

**Extraction Logic**:

```python
def extract_chapter(file_path: Path) -> str:
    """Extract chapter name from directory structure."""
    # Example: /docs/module-1/intro.md → "Module 1"
    parts = file_path.parts
    if len(parts) >= 2:
        module_dir = parts[-2]  # e.g., "module-1"
        # Convert "module-1" to "Module 1"
        return module_dir.replace('-', ' ').title()
    return "Unknown"
```

**Output Format** (per chunk):

```json
{
  "text": "ROS 2 (Robot Operating System 2) is a flexible framework...",
  "metadata": {
    "source": "/docs/module-1/intro.md",
    "chapter": "Module 1",
    "section": "1.1 Introduction",
    "start_token": 0,
    "end_token": 500,
    "token_count": 500,
    "chunk_index": 0,
    "total_chunks": 3,
    "file_modified": "2025-12-09T00:00:00Z"
  }
}
```

**Validation**:
- All chunks have non-empty `text`
- Token counts match actual tokenization
- Source paths are valid (exist in `/docs/`)
- Start/end tokens are sequential within document

---

### 3. Embedding + Ingestion Plan

**Objective**: Generate embeddings with Gemini and store in Qdrant

#### 3.1 Gemini Embedding Integration

**Model**: `models/embedding-001`
**Output**: 768-dimensional vectors

**Implementation**:

```python
# ingest.py - Embedding Generation
import google.generativeai as genai
import os
from typing import List
import time

# Configure Gemini
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

def generate_embeddings(chunks: List[Dict], batch_size: int = 10) -> List[Dict]:
    """Generate embeddings for chunks in batches."""
    model = "models/embedding-001"
    embedded_chunks = []

    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i + batch_size]
        texts = [chunk["text"] for chunk in batch]

        try:
            # Batch embedding call
            result = genai.embed_content(
                model=model,
                content=texts,
                task_type="retrieval_document"
            )

            # Attach embeddings to chunks
            for chunk, embedding in zip(batch, result['embedding']):
                embedded_chunks.append({
                    **chunk,
                    "vector": embedding
                })

            # Rate limiting: respect free tier (60 req/min)
            time.sleep(1)  # 1 second between batches

        except Exception as e:
            print(f"Error embedding batch {i//batch_size}: {e}")
            # Retry logic (max 3 retries)
            for retry in range(3):
                time.sleep(2 ** retry)  # Exponential backoff
                try:
                    result = genai.embed_content(
                        model=model,
                        content=texts,
                        task_type="retrieval_document"
                    )
                    for chunk, embedding in zip(batch, result['embedding']):
                        embedded_chunks.append({**chunk, "vector": embedding})
                    break
                except Exception as retry_error:
                    if retry == 2:  # Last retry
                        raise retry_error

    return embedded_chunks
```

**Error Handling**:
- Catch API errors (rate limits, network issues)
- Retry with exponential backoff (max 3 attempts)
- Log failed batches for manual inspection
- Continue processing remaining batches on partial failure

**Rate Limiting**:
- Batch size: 10 chunks (balance speed vs. rate limits)
- Delay: 1 second between batches (60 batches/min = 600 chunks/min)
- Free tier: 1500 requests/day → ~15,000 chunks/day capacity

#### 3.2 Qdrant Storage Implementation

**Collection Setup**:

```python
# ingest.py - Qdrant Collection Initialization
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import uuid

def init_qdrant_collection(client: QdrantClient, collection_name: str):
    """Create or recreate Qdrant collection."""
    # Check if collection exists
    collections = client.get_collections().collections
    exists = any(c.name == collection_name for c in collections)

    if exists:
        print(f"Collection '{collection_name}' already exists. Skipping creation.")
        # Option: Delete and recreate if --recreate flag passed
        # client.delete_collection(collection_name)
    else:
        # Create collection
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=768,  # Gemini embedding dimensions
                distance=Distance.COSINE  # Cosine similarity
            )
        )
        print(f"Created collection '{collection_name}'")
```

**Upload Chunks to Qdrant**:

```python
# ingest.py - Upload Function
def upload_to_qdrant(client: QdrantClient, collection_name: str,
                     embedded_chunks: List[Dict]):
    """Upload embedded chunks to Qdrant."""
    points = []

    for chunk in embedded_chunks:
        point = PointStruct(
            id=str(uuid.uuid4()),
            vector=chunk["vector"],
            payload={
                "text": chunk["text"],
                **chunk["metadata"]
            }
        )
        points.append(point)

    # Upload in batches (Qdrant recommends 100-1000 per batch)
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        client.upsert(
            collection_name=collection_name,
            points=batch
        )
        print(f"Uploaded batch {i//batch_size + 1}: {len(batch)} points")

    print(f"Total points uploaded: {len(points)}")
```

**Main Ingestion Script**:

```python
# ingest.py - Main Function
import argparse

def main():
    parser = argparse.ArgumentParser(description="Ingest docs into Qdrant")
    parser.add_argument("--docs-dir", default="../docs", help="Path to docs directory")
    parser.add_argument("--collection", default="robotics_textbook_chunks", help="Qdrant collection name")
    parser.add_argument("--batch-size", type=int, default=10, help="Embedding batch size")
    parser.add_argument("--recreate", action="store_true", help="Recreate collection")
    args = parser.parse_args()

    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv()

    # Initialize Qdrant client
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Step 1: Load documents
    print("Loading documents...")
    documents = load_documents(args.docs_dir)
    print(f"Loaded {len(documents)} documents")

    # Step 2: Chunk documents
    print("Chunking documents...")
    all_chunks = []
    for doc in documents:
        chunks = chunk_document(doc["content"], doc["metadata"])
        all_chunks.extend(chunks)
    print(f"Created {len(all_chunks)} chunks")

    # Step 3: Generate embeddings
    print("Generating embeddings...")
    embedded_chunks = generate_embeddings(all_chunks, args.batch_size)
    print(f"Generated {len(embedded_chunks)} embeddings")

    # Step 4: Initialize collection
    print("Initializing Qdrant collection...")
    if args.recreate:
        try:
            client.delete_collection(args.collection)
            print(f"Deleted existing collection '{args.collection}'")
        except:
            pass
    init_qdrant_collection(client, args.collection)

    # Step 5: Upload to Qdrant
    print("Uploading to Qdrant...")
    upload_to_qdrant(client, args.collection, embedded_chunks)

    print("Ingestion complete!")

if __name__ == "__main__":
    main()
```

**Execution**:
```bash
cd /rag
python ingest.py --docs-dir ../docs --collection robotics_textbook_chunks
```

**Expected Output**:
```
Loading documents...
Loaded 250 documents
Chunking documents...
Created 2000 chunks
Generating embeddings...
Generated 2000 embeddings
Initializing Qdrant collection...
Created collection 'robotics_textbook_chunks'
Uploading to Qdrant...
Uploaded batch 1: 100 points
Uploaded batch 2: 100 points
...
Total points uploaded: 2000
Ingestion complete!
```

**Validation**:
- Verify chunk count matches expected
- Check Qdrant dashboard for collection size
- Test sample search query

---

### 4. Backend Plan (FastAPI)

**Objective**: Build dual-mode RAG API server

#### 4.1 FastAPI Project Structure

**File**: `rag_server.py`

**Dependencies** (`requirements.txt`):
```txt
fastapi==0.104.1
uvicorn[standard]==0.24.0
google-generativeai==0.3.1
qdrant-client==1.7.0
python-dotenv==1.0.0
pydantic==2.5.0
aiohttp==3.9.1
```

#### 4.2 Core FastAPI Application

```python
# rag_server.py
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import Optional, List
import os
from dotenv import load_dotenv
import google.generativeai as genai
from qdrant_client import QdrantClient
import time

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Dual-mode RAG system for robotics textbook",
    version="1.0.0"
)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=os.getenv("CORS_ORIGINS", "*").split(","),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize clients
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Pydantic models
class QueryRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=2000)
    selectedText: Optional[str] = Field(None, max_length=10000)

class Source(BaseModel):
    text: str
    source: str
    score: float

class QueryResponse(BaseModel):
    answer: str
    mode: str
    sources: List[Source] = []
    response_time_ms: int

# Helper functions (see sections below)
async def embed_query(question: str) -> List[float]:
    """Embed user question using Gemini."""
    result = genai.embed_content(
        model=os.getenv("GEMINI_EMBEDDING_MODEL", "models/embedding-001"),
        content=question,
        task_type="retrieval_query"
    )
    return result['embedding']

async def search_qdrant(embedding: List[float], top_k: int = 5) -> List[dict]:
    """Search Qdrant for similar chunks."""
    collection = os.getenv("QDRANT_COLLECTION", "robotics_textbook_chunks")
    results = qdrant_client.search(
        collection_name=collection,
        query_vector=embedding,
        limit=top_k,
        score_threshold=float(os.getenv("SIMILARITY_THRESHOLD", "0.7"))
    )
    return [
        {
            "text": hit.payload["text"],
            "source": hit.payload["source"],
            "score": hit.score
        }
        for hit in results
    ]

async def generate_response(question: str, context: str) -> str:
    """Generate response using Gemini chat."""
    model = genai.GenerativeModel(
        os.getenv("GEMINI_CHAT_MODEL", "models/gemini-pro")
    )

    prompt = f"""You are a helpful AI assistant for a robotics textbook.
Answer the user's question based on the provided context.
If the context doesn't contain relevant information, say so politely.

Context:
{context}

Question: {question}

Answer:"""

    response = model.generate_content(prompt)
    return response.text

# Endpoint implementations (see next section)
```

#### 4.3 Endpoint Implementation

**POST /query** (dual-mode logic):

```python
# rag_server.py - /query endpoint
@app.post("/api/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Process user question with dual-mode RAG.

    Mode selection:
    - If selectedText is provided: Selected-Text mode (skip Qdrant)
    - Otherwise: Normal RAG mode (embed → search → generate)
    """
    start_time = time.time()

    # Validate request
    if not request.question.strip():
        raise HTTPException(status_code=400, detail="Question cannot be empty")

    try:
        # Mode detection
        if request.selectedText and request.selectedText.strip():
            # Selected-Text Mode
            mode = "selected_text"
            context = request.selectedText
            sources = []  # No Qdrant search

        else:
            # Normal RAG Mode
            mode = "normal_rag"

            # Step 1: Embed question
            embedding = await embed_query(request.question)

            # Step 2: Search Qdrant
            search_results = await search_qdrant(
                embedding,
                top_k=int(os.getenv("TOP_K_RESULTS", "5"))
            )

            # Step 3: Assemble context
            if not search_results:
                context = "No relevant information found in the textbook."
                sources = []
            else:
                context = "\n\n".join([
                    f"[Source: {r['source']}]\n{r['text']}"
                    for r in search_results
                ])
                sources = [
                    Source(
                        text=r["text"][:200] + "..." if len(r["text"]) > 200 else r["text"],
                        source=r["source"],
                        score=r["score"]
                    )
                    for r in search_results
                ]

        # Generate response (both modes)
        answer = await generate_response(request.question, context)

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        return QueryResponse(
            answer=answer,
            mode=mode,
            sources=sources,
            response_time_ms=response_time_ms
        )

    except Exception as e:
        # Error handling
        if "rate_limit" in str(e).lower():
            raise HTTPException(status_code=429, detail="Rate limit exceeded. Please try again later.")
        elif "qdrant" in str(e).lower():
            raise HTTPException(status_code=503, detail="Vector database unavailable")
        else:
            raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")
```

**GET /health** (monitoring):

```python
# rag_server.py - /health endpoint
@app.get("/api/health")
async def health_check():
    """Health check endpoint for monitoring."""
    try:
        # Check Qdrant connection
        collections = qdrant_client.get_collections()
        qdrant_connected = True
    except:
        qdrant_connected = False

    if qdrant_connected:
        return {"status": "healthy", "qdrant_connected": True}
    else:
        raise HTTPException(status_code=503, detail="Qdrant unavailable")
```

**Run locally**:
```bash
cd /rag
uvicorn rag_server:app --reload --port 8000
```

**Test endpoints**:
```bash
# Health check
curl http://localhost:8000/api/health

# Normal RAG query
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'

# Selected text query
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"question": "Explain this", "selectedText": "ROS 2 uses DDS middleware..."}'
```

---

### 5. Frontend Plan (ChatKit)

**Objective**: Build React chat UI with text selection support

#### 5.1 ChatKit Installation

```bash
# From project root
npm install @chatscope/chat-ui-kit-react @chatscope/chat-ui-kit-styles
```

#### 5.2 Component Structure

**Directory**: `/src/components/RagChat/`

**Files**:
1. `index.jsx` - Main export
2. `ChatKitWrapper.jsx` - Chat UI component
3. `TextSelectionHandler.js` - Text selection logic
4. `config.js` - Backend URL config
5. `styles.module.css` - Component styles

#### 5.3 Backend URL Configuration

**File**: `/src/components/RagChat/config.js`

```javascript
// config.js
const getBackendUrl = () => {
  // Development: local FastAPI server
  if (process.env.NODE_ENV === 'development') {
    return 'http://localhost:8000';
  }

  // Production: Vercel serverless function
  return process.env.REACT_APP_RAG_BACKEND_URL || 'https://your-app.vercel.app';
};

export default getBackendUrl;
```

**Environment variables** (`.env` in project root):
```bash
# Development
REACT_APP_RAG_BACKEND_URL=http://localhost:8000

# Production (set in Vercel dashboard)
REACT_APP_RAG_BACKEND_URL=https://your-rag-backend.vercel.app
```

#### 5.4 ChatKit Component Implementation

**File**: `/src/components/RagChat/ChatKitWrapper.jsx`

```jsx
import React, { useState } from 'react';
import {
  MainContainer,
  ChatContainer,
  MessageList,
  Message,
  MessageInput,
  TypingIndicator
} from '@chatscope/chat-ui-kit-react';
import '@chatscope/chat-ui-kit-styles/dist/default/styles.min.css';
import getBackendUrl from './config';
import styles from './styles.module.css';

const ChatKitWrapper = ({ enableTextSelection = true }) => {
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState(null);
  const [error, setError] = useState(null);

  const backendUrl = getBackendUrl();

  const handleSendMessage = async (messageText) => {
    // Add user message to UI
    const userMessage = {
      message: messageText,
      sentTime: new Date().toISOString(),
      sender: "user",
      direction: "outgoing",
      position: "single"
    };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      // Call backend API
      const response = await fetch(`${backendUrl}/api/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: messageText,
          selectedText: selectedText
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Add assistant response
      const assistantMessage = {
        message: data.answer,
        sentTime: new Date().toISOString(),
        sender: "assistant",
        direction: "incoming",
        position: "single",
        metadata: {
          mode: data.mode,
          sources: data.sources,
          responseTime: data.response_time_ms
        }
      };
      setMessages(prev => [...prev, assistantMessage]);

      // Clear selected text after use
      setSelectedText(null);

    } catch (error) {
      console.error('Error calling backend:', error);
      setError('Sorry, I encountered an error. Please try again.');

      const errorMessage = {
        message: error.message || "Sorry, I encountered an error. Please try again.",
        sentTime: new Date().toISOString(),
        sender: "system",
        direction: "incoming",
        position: "single"
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.chatContainer}>
      <MainContainer>
        <ChatContainer>
          <MessageList
            scrollBehavior="smooth"
            typingIndicator={isLoading ? <TypingIndicator content="Assistant is thinking..." /> : null}
          >
            {messages.map((msg, i) => (
              <Message key={i} model={msg}>
                {msg.metadata?.sources && msg.metadata.sources.length > 0 && (
                  <Message.Footer>
                    <div className={styles.sources}>
                      Sources: {msg.metadata.sources.map(s => s.source).join(', ')}
                    </div>
                  </Message.Footer>
                )}
              </Message>
            ))}
          </MessageList>
          <MessageInput
            placeholder="Ask a question about the textbook..."
            onSend={handleSendMessage}
            disabled={isLoading}
            attachButton={false}
          />
          {selectedText && (
            <div className={styles.selectedTextIndicator}>
              <span className={styles.label}>Selected text:</span>
              <span className={styles.text}>
                {selectedText.substring(0, 50)}...
              </span>
              <button
                className={styles.clearButton}
                onClick={() => setSelectedText(null)}
              >
                ✕
              </button>
            </div>
          )}
        </ChatContainer>
      </MainContainer>
    </div>
  );
};

export default ChatKitWrapper;
```

#### 5.5 Text Selection Handler

**File**: `/src/components/RagChat/TextSelectionHandler.js`

```javascript
// TextSelectionHandler.js
import { useEffect } from 'react';

export const useTextSelection = (onTextSelected) => {
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const selectedText = selection.toString().trim();

      if (selectedText.length > 0) {
        // Only capture selections from main content area
        const range = selection.getRangeAt(0);
        const container = range.commonAncestorContainer;

        // Check if selection is from article content (not from chat UI)
        if (container.closest && !container.closest('.chat-container')) {
          onTextSelected(selectedText);
        }
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
    };
  }, [onTextSelected]);
};
```

**Integration in main component**:

```jsx
// ChatKitWrapper.jsx - add text selection hook
import { useTextSelection } from './TextSelectionHandler';

const ChatKitWrapper = ({ enableTextSelection = true }) => {
  // ... existing state ...

  // Text selection hook
  useTextSelection((text) => {
    if (enableTextSelection) {
      setSelectedText(text);
      // Optional: Show floating "Ask about this" button
    }
  });

  // ... rest of component ...
};
```

#### 5.6 ChatKit Styling

**File**: `/src/components/RagChat/styles.module.css`

```css
/* styles.module.css */
.chatContainer {
  position: fixed;
  bottom: 20px;
  right: 20px;
  width: 400px;
  height: 600px;
  border-radius: 10px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  z-index: 1000;
  background-color: white;
}

.selectedTextIndicator {
  display: flex;
  align-items: center;
  padding: 8px 12px;
  background-color: #f0f8ff;
  border-left: 3px solid #007bff;
  font-size: 12px;
  margin: 5px 10px;
  border-radius: 4px;
}

.selectedTextIndicator .label {
  font-weight: bold;
  margin-right: 8px;
}

.selectedTextIndicator .text {
  flex-grow: 1;
  color: #555;
}

.selectedTextIndicator .clearButton {
  background: none;
  border: none;
  font-size: 16px;
  cursor: pointer;
  color: #999;
  margin-left: 8px;
}

.selectedTextIndicator .clearButton:hover {
  color: #333;
}

.sources {
  font-size: 11px;
  color: #666;
  margin-top: 5px;
  padding-top: 5px;
  border-top: 1px solid #eee;
}

@media (max-width: 768px) {
  .chatContainer {
    width: 100%;
    height: 100%;
    bottom: 0;
    right: 0;
    border-radius: 0;
  }
}
```

**Global styles** (`/static/ragchat.css`):

```css
/* /static/ragchat.css */
.cs-message-list {
  max-height: 450px;
  overflow-y: auto;
}

.cs-message__content {
  word-wrap: break-word;
  white-space: pre-wrap;
}

.cs-message--incoming .cs-message__content {
  background-color: #f1f3f4;
  color: #202124;
}

.cs-message--outgoing .cs-message__content {
  background-color: #007bff;
  color: white;
}
```

#### 5.7 Main Export

**File**: `/src/components/RagChat/index.jsx`

```jsx
// index.jsx
import React from 'react';
import ChatKitWrapper from './ChatKitWrapper';

const RagChat = (props) => {
  return <ChatKitWrapper {...props} />;
};

export default RagChat;
```

#### 5.8 Integration with Docusaurus

**Optional**: Add to Docusaurus layout (e.g., `/src/theme/Root.js`):

```jsx
// /src/theme/Root.js
import React from 'react';
import RagChat from '@site/src/components/RagChat';

export default function Root({ children }) {
  return (
    <>
      {children}
      <RagChat enableTextSelection={true} />
    </>
  );
}
```

**Note**: This is the ONLY modification to existing Docusaurus structure, and it's optional/reversible.

---

### 6. Deployment Plan

**Objective**: Deploy backend to Vercel, frontend to GitHub Pages

#### 6.1 Vercel Configuration

**File**: `/rag/vercel.json`

```json
{
  "version": 2,
  "builds": [
    {
      "src": "rag_server.py",
      "use": "@vercel/python",
      "config": {
        "maxDuration": 30,
        "runtime": "python3.9"
      }
    }
  ],
  "routes": [
    {
      "src": "/api/query",
      "dest": "rag_server.py"
    },
    {
      "src": "/api/health",
      "dest": "rag_server.py"
    }
  ],
  "env": {
    "GEMINI_API_KEY": "@gemini-api-key",
    "QDRANT_URL": "@qdrant-url",
    "QDRANT_API_KEY": "@qdrant-api-key",
    "QDRANT_COLLECTION": "@qdrant-collection",
    "CORS_ORIGINS": "@cors-origins",
    "TOP_K_RESULTS": "5",
    "SIMILARITY_THRESHOLD": "0.7"
  }
}
```

#### 6.2 Vercel Deployment Steps

**Prerequisites**:
- Vercel account (free tier)
- Vercel CLI installed (`npm install -g vercel`)
- Git repository pushed to GitHub

**Steps**:

1. **Install Vercel CLI**:
   ```bash
   npm install -g vercel
   ```

2. **Login to Vercel**:
   ```bash
   vercel login
   ```

3. **Link project** (from project root):
   ```bash
   vercel link
   ```
   - Select team (or personal account)
   - Link to existing project or create new
   - Name: `rag-chatbot-backend`

4. **Set environment variables** (Vercel dashboard):
   - Go to Project Settings → Environment Variables
   - Add secrets:
     ```
     GEMINI_API_KEY=<your-gemini-key>
     QDRANT_URL=https://<your-cluster>.qdrant.io
     QDRANT_API_KEY=<your-qdrant-key>
     QDRANT_COLLECTION=robotics_textbook_chunks
     CORS_ORIGINS=https://yourusername.github.io,http://localhost:3000
     ```

5. **Deploy to production**:
   ```bash
   vercel --prod
   ```

6. **Verify deployment**:
   ```bash
   # Health check
   curl https://your-app.vercel.app/api/health

   # Test query
   curl -X POST https://your-app.vercel.app/api/query \
     -H "Content-Type: application/json" \
     -d '{"question": "What is ROS 2?"}'
   ```

**Expected Output**: JSON response with `answer` field

#### 6.3 Frontend Deployment (GitHub Pages)

**Prerequisites**:
- GitHub repository with Docusaurus site
- GitHub Pages enabled

**Steps**:

1. **Update frontend config** (`.env.production`):
   ```bash
   REACT_APP_RAG_BACKEND_URL=https://your-app.vercel.app
   ```

2. **Build Docusaurus**:
   ```bash
   npm run build
   ```

3. **Deploy to GitHub Pages**:
   ```bash
   npm run deploy
   ```
   - Or via GitHub Actions (if configured)

4. **Verify integration**:
   - Open `https://yourusername.github.io/ai_robotics_book`
   - Look for chat widget (bottom-right corner)
   - Submit test query
   - Verify response appears

#### 6.4 Local Testing Steps

**Before deploying, test locally**:

1. **Start backend** (Terminal 1):
   ```bash
   cd /rag
   python -m venv venv
   source venv/bin/activate  # Windows: venv\Scripts\activate
   pip install -r requirements.txt
   uvicorn rag_server:app --reload --port 8000
   ```

2. **Start frontend** (Terminal 2):
   ```bash
   # From project root
   npm start
   ```

3. **Test integration**:
   - Open `http://localhost:3000`
   - Open browser DevTools (F12) → Console
   - Submit question in chat
   - Verify:
     - Network request to `http://localhost:8000/api/query`
     - Response received (200 OK)
     - Answer displayed in chat UI

4. **Test selected text mode**:
   - Highlight text on page
   - Submit question
   - Verify `selectedText` sent in request (check Network tab)
   - Verify response uses selected text

---

### 7. Testing & QA Plan

**Objective**: Validate all system components

#### 7.1 Ingestion Testing

**Checklist**:

- [ ] **Document Loading**
  - Run: `python ingest.py --docs-dir ../docs`
  - Verify: All `.md` files loaded (check console output)
  - Expected: ~200-300 files

- [ ] **Chunking**
  - Verify: Chunk count matches expected (~1500-2500)
  - Check: No chunks < 100 tokens
  - Check: No chunks > 600 tokens
  - Check: Overlap working (inspect adjacent chunks)

- [ ] **Embedding Generation**
  - Verify: All chunks have embeddings
  - Check: Vector dimension = 768
  - Monitor: API rate limits respected (1s delay between batches)

- [ ] **Qdrant Upload**
  - Verify: Collection created in Qdrant dashboard
  - Check: Point count matches chunk count
  - Test: Random point retrieval (verify payload structure)

**Manual Test**:
```python
# test_ingestion.py
from qdrant_client import QdrantClient
import os

client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))

# Get collection info
collection_info = client.get_collection("robotics_textbook_chunks")
print(f"Total points: {collection_info.points_count}")

# Retrieve random point
points = client.scroll(
    collection_name="robotics_textbook_chunks",
    limit=1
)[0]
print(f"Sample point: {points[0].payload}")
```

#### 7.2 Qdrant Search Testing

**Checklist**:

- [ ] **Vector Search**
  - Test: Search with sample question embedding
  - Verify: Top-k results returned (k=5)
  - Check: Similarity scores > threshold (0.7)
  - Inspect: Returned payloads contain expected fields

**Manual Test**:
```python
# test_search.py
import google.generativeai as genai
from qdrant_client import QdrantClient
import os

genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))

# Embed test question
question = "What is ROS 2?"
result = genai.embed_content(
    model="models/embedding-001",
    content=question,
    task_type="retrieval_query"
)
embedding = result['embedding']

# Search Qdrant
results = client.search(
    collection_name="robotics_textbook_chunks",
    query_vector=embedding,
    limit=5,
    score_threshold=0.7
)

print(f"Found {len(results)} results")
for i, hit in enumerate(results):
    print(f"\n{i+1}. Score: {hit.score:.3f}")
    print(f"   Source: {hit.payload['source']}")
    print(f"   Text: {hit.payload['text'][:100]}...")
```

**Expected Output**: 3-5 relevant chunks about ROS 2

#### 7.3 Selected Text Mode Testing

**Checklist**:

- [ ] **Mode Detection**
  - Test: Request with `selectedText` → mode = "selected_text"
  - Test: Request without `selectedText` → mode = "normal_rag"

- [ ] **Context Usage**
  - Verify: Response uses only provided text (not Qdrant results)
  - Test: Provide unrelated text → response should reference provided text

**Manual Test**:
```bash
# Test selected text mode
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Explain this concept",
    "selectedText": "ROS 2 uses DDS (Data Distribution Service) as its middleware, enabling real-time communication between nodes."
  }' | jq .

# Expected: response.mode = "selected_text", no sources
```

#### 7.4 ChatKit UI Testing

**Checklist**:

- [ ] **UI Rendering**
  - Verify: Chat widget appears (bottom-right)
  - Check: Input field accepts text
  - Check: Send button enabled

- [ ] **Message Flow**
  - Test: Send question → user message appears
  - Verify: Loading indicator shows
  - Verify: Response appears after delay
  - Check: Timestamp displayed

- [ ] **Text Selection**
  - Test: Highlight text on page → indicator appears
  - Verify: Next question includes selected text
  - Check: Indicator clears after sending

- [ ] **Error Handling**
  - Test: Empty question → error message
  - Test: Backend down → error message
  - Test: Rate limit → retry message

**Manual Browser Test**:
1. Open DevTools → Network tab
2. Send question
3. Inspect request payload (question, selectedText)
4. Inspect response (answer, mode, sources)
5. Check for JavaScript errors in Console

#### 7.5 End-to-End Integration Test

**Scenario 1: Normal RAG Query**

Steps:
1. Open textbook in browser
2. Type question: "What is ROS 2?"
3. Send message

Expected:
- Backend receives request with `selectedText: null`
- Mode: "normal_rag"
- Qdrant search executed
- Sources returned (3-5 chunks)
- Relevant answer displayed
- Response time < 5s

**Scenario 2: Selected Text Query**

Steps:
1. Highlight paragraph about ROS 2
2. Type question: "Explain this in simple terms"
3. Send message

Expected:
- Backend receives request with `selectedText: "..."`
- Mode: "selected_text"
- No Qdrant search (sources empty)
- Answer references highlighted text
- Response time < 3s

**Scenario 3: Error Handling**

Steps:
1. Stop backend server
2. Send question

Expected:
- Network error caught
- Error message displayed in chat
- User can retry after backend restart

#### 7.6 Performance Testing

**Metrics**:

| Metric | Target | Test Method |
|--------|--------|-------------|
| Normal RAG response time | <5s (P95) | Send 20 queries, measure time |
| Selected text response time | <3s (P95) | Send 20 queries with text |
| Embedding batch processing | 10 chunks/batch | Monitor ingestion logs |
| Concurrent users | 100 | Load testing (optional) |

**Load Test** (optional, using `locust` or similar):
```python
# locustfile.py
from locust import HttpUser, task, between

class RagChatbotUser(HttpUser):
    wait_time = between(1, 5)

    @task
    def query_normal_rag(self):
        self.client.post("/api/query", json={
            "question": "What is ROS 2?"
        })

    @task
    def query_selected_text(self):
        self.client.post("/api/query", json={
            "question": "Explain this",
            "selectedText": "ROS 2 uses DDS middleware..."
        })
```

Run: `locust -f locustfile.py --host http://localhost:8000`

---

### 8. Deliverables Checklist

**Backend Files**:

- [x] `/rag/ingest.py` - Document ingestion script
  - [ ] Load documents from `/docs`
  - [ ] Chunk with sliding window (500/150)
  - [ ] Generate embeddings via Gemini
  - [ ] Upload to Qdrant

- [x] `/rag/rag_server.py` - FastAPI backend
  - [ ] POST /api/query endpoint
  - [ ] GET /api/health endpoint
  - [ ] Dual-mode logic (selectedText detection)
  - [ ] Error handling

- [x] `/rag/requirements.txt` - Python dependencies
  - [ ] FastAPI, uvicorn
  - [ ] google-generativeai
  - [ ] qdrant-client
  - [ ] pydantic, python-dotenv

- [x] `/rag/vercel.json` - Vercel config
  - [ ] Python runtime specified
  - [ ] Routes configured (/api/query, /api/health)
  - [ ] Environment variables referenced

- [x] `/rag/config.env.example` - Environment template
  - [ ] All required variables documented
  - [ ] Example values provided

- [x] `/rag/README.md` - Backend documentation
  - [ ] Setup instructions
  - [ ] Usage examples
  - [ ] Deployment guide

**Frontend Files**:

- [x] `/src/components/RagChat/index.jsx` - Main export
- [x] `/src/components/RagChat/ChatKitWrapper.jsx` - Chat UI
  - [ ] Message state management
  - [ ] Backend API calls
  - [ ] Error handling

- [x] `/src/components/RagChat/TextSelectionHandler.js`
  - [ ] Text selection detection
  - [ ] State updates

- [x] `/src/components/RagChat/config.js` - Backend URL
  - [ ] Environment-based configuration
  - [ ] Dev/prod URLs

- [x] `/src/components/RagChat/styles.module.css`
  - [ ] Component styles
  - [ ] Responsive design

- [x] `/static/ragchat.css` - Global styles
  - [ ] ChatKit customization

**Integration**:

- [x] Optional: `/src/theme/Root.js` modification
  - [ ] Import RagChat component
  - [ ] Render in layout

**Documentation**:

- [x] `/specs/001-rag-chatbot/research.md` - Technology research
- [x] `/specs/001-rag-chatbot/data-model.md` - Data structures
- [x] `/specs/001-rag-chatbot/quickstart.md` - Setup guide
- [x] `/specs/001-rag-chatbot/contracts/api-schema.yaml` - API spec

**Deployment**:

- [ ] Vercel deployment successful
  - [ ] Backend accessible at production URL
  - [ ] Environment variables configured
  - [ ] Health check passing

- [ ] Frontend deployed to GitHub Pages
  - [ ] Chat widget visible
  - [ ] Backend URL configured
  - [ ] Integration working

**Testing**:

- [ ] Ingestion validated (chunks in Qdrant)
- [ ] Normal RAG mode tested
- [ ] Selected text mode tested
- [ ] ChatKit UI tested (all scenarios)
- [ ] End-to-end integration verified
- [ ] Performance targets met (<5s response time)

---

## Architecture Decision Records (ADR)

**No ADRs required at this stage** - all technical decisions align with constitution requirements (Gemini, Qdrant, FastAPI, ChatKit, Vercel). If architectural changes are needed during implementation, create ADRs via `/sp.adr` command.

---

## Next Steps

After plan approval:

1. **Phase 0**: Generate `research.md` with technology validation
2. **Phase 1**: Generate `data-model.md`, `contracts/`, `quickstart.md`
3. **Phase 2**: Run `/sp.tasks` to create granular implementation tasks
4. **Implementation**: Execute tasks via `/sp.implement`

**Branch**: `001-rag-chatbot`
**Plan File**: `/specs/001-rag-chatbot/plan.md`

---

## Summary

This implementation plan provides end-to-end guidance for building the RAG chatbot:

1. **Initial Setup**: Qdrant cluster, Gemini API, `/rag` folder structure
2. **Data Extraction**: Markdown reading, 500/150 chunking, metadata extraction
3. **Embedding**: Gemini embedding-001 (768-dim), batch processing
4. **Ingestion**: Qdrant collection creation, vector upload
5. **Backend**: FastAPI dual-mode server, /query endpoint, error handling
6. **Frontend**: ChatKit UI, text selection, backend integration
7. **Deployment**: Vercel (backend), GitHub Pages (frontend)
8. **Testing**: Comprehensive validation across all components

All code is isolated in `/rag/` and `/src/components/RagChat/` to maintain constitution compliance. The system supports both semantic search (Normal RAG) and focused explanations (Selected-Text mode) using free-tier services (Gemini, Qdrant, Vercel).

**Ready for `/sp.tasks` generation.**
