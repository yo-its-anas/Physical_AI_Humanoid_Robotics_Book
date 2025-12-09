# Feature Specification: RAG Chatbot Integration

**Feature Branch**: `001-rag-chatbot`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "Convert the RAG constitution into a detailed full specification.

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

Produce a complete specification suitable for planning and implementation."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask a Question (Normal RAG Mode) (Priority: P1)

As a user, I want to ask questions about the textbook content and receive relevant answers based on the most accurate information available in the documentation.

**Why this priority**: This is the primary function of a RAG chatbot, providing immediate value by enabling users to query the textbook content dynamically.

**Independent Test**: Can be fully tested by submitting a question and verifying the response contains accurate information derived from the textbook, without requiring any prior context or selected text.

**Acceptance Scenarios**:

1.  **Given** I am on the chatbot interface, **When** I submit a question without any selected text, **Then** I receive a concise and accurate answer based on the textbook content.
2.  **Given** I ask a question about a topic present in the `/docs` markdown files, **When** the system processes my question, **Then** the answer incorporates relevant information from those documents.
3.  **Given** I ask a question that is out of scope for the textbook, **When** the system processes my question, **Then** the system gracefully indicates it cannot answer or provides a general response, without hallucinating.

---

### User Story 2 - Ask a Question with Selected Text (Priority: P1)

As a user, I want to select specific text from the textbook and ask a question relevant to that selection, ensuring the chatbot's response is directly focused on my highlighted context.

**Why this priority**: This enhances user control and provides a more focused interaction, allowing for precise queries within specific sections of the textbook, crucial for in-depth study.

**Independent Test**: Can be fully tested by selecting text, submitting a question related to that text, and verifying the response is strictly confined to the provided selected text and question, ignoring broader document context.

**Acceptance Scenarios**:

1.  **Given** I have selected a portion of text from the textbook, **When** I submit a question along with the selected text, **Then** I receive an answer that primarily uses the selected text as context.
2.  **Given** I select text and ask a question, **When** the system processes my request, **Then** it bypasses the standard document retrieval and directly uses the selected text for response generation.

---

### Edge Cases

-   What happens when an empty question or selected text is submitted? The system should handle gracefully, prompting the user for valid input.
-   How does the system handle extremely long selected text? The system should process it efficiently, potentially summarizing or focusing on key parts if there are token limits.
-   What happens if no relevant context is found in Normal RAG mode? The system should indicate it cannot answer or provide a general fallback response.
-   What happens if the selected text is irrelevant to the question in selected text mode? The system should still attempt to answer based on the provided text, potentially indicating ambiguity if necessary.

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: The system MUST accept user questions for RAG-based answers.
-   **FR-002**: The system MUST support two modes of interaction: Normal RAG (question only) and Selected Text (question + selected text).
-   **FR-003**: The system MUST use Gemini embeddings for document chunk embedding and user question embedding.
-   **FR-004**: The system MUST use Gemini for chat generation to produce responses.
-   **FR-005**: The system MUST store document chunks and their embeddings in a Qdrant vector database.
-   **FR-006**: The Qdrant vector database MUST have a collection with a defined payload schema including `text`, `source`, `start_token`, `end_token`, and appropriate embedding dimensions.
-   **FR-007**: In Normal RAG mode, the system MUST embed the user question, search Qdrant for relevant context, and pass both the question and context to Gemini for response generation.
-   **FR-008**: In Selected Text mode, the system MUST skip Qdrant search and pass the user question along with the provided selected text directly to Gemini for response generation.
-   **FR-009**: The system MUST ingest markdown files from the `/docs` directory.
-   **FR-010**: The chunking strategy MUST divide documents into approximately 500-token chunks with a 150-token overlap.
-   **FR-011**: Each chunk MUST include metadata such as `source` (file path), `start_token`, and `end_token`.
-   **FR-012**: The backend MUST expose a POST `/query` endpoint.
-   **FR-013**: The POST `/query` endpoint body MUST accept `question` (string, mandatory) and `selectedText` (string, optional).
-   **FR-014**: The FastAPI backend MUST implement the logic for switching between Normal RAG and Selected Text modes based on the presence of `selectedText` in the request body.
-   **FR-015**: The ChatKit frontend MUST be a React component that calls the backend `/query` endpoint.
-   **FR-016**: The ChatKit frontend MUST support sending selected text from the UI to the backend.
-   **FR-017**: The ChatKit frontend MUST allow configuration of the backend URL.
-   **FR-018**: The deployment MUST be set up as a Vercel serverless function.
-   **FR-019**: The deployment MUST include environment variable configuration (e.g., `config.env.example`).
-   **FR-020**: The deployment MUST have a defined build and deploy flow.
-   **FR-021**: The RAG codebase MUST be isolated and NOT modify existing textbook files.
-   **FR-022**: The ChatKit integration MUST be done through a new component and NOT directly modify existing components.

### Key Entities *(include if feature involves data)*

-   **Chunk**: A segment of document text (approx. 500 tokens) with associated metadata (source, start_token, end_token).
-   **Embedding**: A numerical vector representation of a text chunk or user question, generated by Gemini embeddings.
-   **Question**: The natural language query submitted by the user.
-   **Context**: Relevant document chunks retrieved from Qdrant, used to inform Gemini's response.
-   **Response**: The generated answer from Gemini.
-   **SelectedText**: A specific portion of text highlighted by the user from the textbook, used as direct context.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: 90% of user questions in Normal RAG mode receive accurate and relevant answers based on textbook content.
-   **SC-002**: 95% of user questions in Selected Text mode receive answers that are directly informed by the provided selected text.
-   **SC-003**: The RAG chatbot responds to 95% of queries within 5 seconds.
-   **SC-004**: Document ingestion and embedding process successfully processes all `/docs` markdown files without errors.
-   **SC-005**: The deployed RAG service on Vercel is accessible and functional 99.9% of the time.
-   **SC-006**: No unintended modifications are made to existing textbook files or core components during integration.

---

## Architecture Overview

### System Components

The RAG chatbot system consists of four primary architectural layers:

1. **Frontend Layer**: React-based chat interface using OpenAI ChatKit SDK for UI rendering
2. **Backend Layer**: FastAPI Python server handling dual-mode RAG logic
3. **Vector Storage Layer**: Qdrant Cloud vector database storing document embeddings
4. **LLM Layer**: Google Gemini API for embeddings and response generation

### Component Interactions

```
┌──────────────────────────────────────────────────────────────────┐
│                         FRONTEND LAYER                            │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │ ChatKit UI Component (React)                                │ │
│  │  - User input field                                         │ │
│  │  - Text selection handler                                   │ │
│  │  - Message display                                          │ │
│  └─────────────────────────────────────────────────────────────┘ │
└────────────────────────────┬─────────────────────────────────────┘
                             │ HTTP POST /query
                             │ {question, selectedText?}
                             ▼
┌──────────────────────────────────────────────────────────────────┐
│                         BACKEND LAYER                             │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │ FastAPI Server (rag_server.py)                              │ │
│  │                                                             │ │
│  │  ┌─────────────┐    ┌──────────────────────────────────┐   │ │
│  │  │ /query POST │───▶│ Mode Detection Logic             │   │ │
│  │  └─────────────┘    │  - Check if selectedText exists  │   │ │
│  │                     └──────────────┬───────────────────┘   │ │
│  │                                    │                        │ │
│  │         ┌──────────────────────────┴────────────────┐       │ │
│  │         ▼                                           ▼       │ │
│  │  ┌────────────────┐                      ┌────────────────┐│ │
│  │  │ Normal RAG Mode│                      │Selected Text   ││ │
│  │  │                │                      │Mode            ││ │
│  │  │ 1. Embed query │                      │                ││ │
│  │  │ 2. Search DB   │                      │ 1. Use selected││ │
│  │  │ 3. Retrieve    │                      │    text as     ││ │
│  │  │    context     │                      │    context     ││ │
│  │  └────────┬───────┘                      └────────┬───────┘│ │
│  │           └──────────────────┬───────────────────┘         │ │
│  │                              ▼                              │ │
│  │                   ┌──────────────────────┐                  │ │
│  │                   │ Gemini Chat API Call │                  │ │
│  │                   │ (question + context) │                  │ │
│  │                   └──────────┬───────────┘                  │ │
│  │                              │                              │ │
│  │                              ▼                              │ │
│  │                   ┌──────────────────────┐                  │ │
│  │                   │ Format & Return JSON │                  │ │
│  │                   └──────────────────────┘                  │ │
│  └─────────────────────────────────────────────────────────────┘ │
└────────────┬─────────────────────────────────────────────────────┘
             │
             │ Embedding queries & context retrieval
             ▼
┌──────────────────────────────────────────────────────────────────┐
│                      VECTOR STORAGE LAYER                         │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │ Qdrant Cloud Vector Database                                │ │
│  │                                                             │ │
│  │  Collection: "robotics_textbook_chunks"                     │ │
│  │                                                             │ │
│  │  Schema:                                                    │ │
│  │  {                                                          │ │
│  │    "id": "uuid",                                            │ │
│  │    "vector": [768 dimensions],  // Gemini embedding size    │ │
│  │    "payload": {                                             │ │
│  │      "text": "chunk content",                               │ │
│  │      "source": "/docs/path/to/file.md",                     │ │
│  │      "start_token": 0,                                      │ │
│  │      "end_token": 500,                                      │ │
│  │      "chapter": "Module 1",                                 │ │
│  │      "metadata": {...}                                      │ │
│  │    }                                                        │ │
│  │  }                                                          │ │
│  │                                                             │ │
│  │  Operations:                                                │ │
│  │  - Vector search (cosine similarity)                        │ │
│  │  - Top-k retrieval (default k=5)                            │ │
│  └─────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
             ▲
             │ Document ingestion (one-time/periodic)
             │
┌──────────────────────────────────────────────────────────────────┐
│                      INGESTION PIPELINE                           │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │ ingest.py Script                                            │ │
│  │                                                             │ │
│  │  1. Read /docs/**/*.md files                                │ │
│  │  2. Chunk text (500 tokens, 150 overlap)                    │ │
│  │  3. Generate embeddings (Gemini API)                        │ │
│  │  4. Upload to Qdrant with metadata                          │ │
│  └─────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
             │
             │ Calls Gemini API
             ▼
┌──────────────────────────────────────────────────────────────────┐
│                           LLM LAYER                               │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │ Google Gemini API (Free Tier)                               │ │
│  │                                                             │ │
│  │  Services Used:                                             │ │
│  │  - models/embedding-001 (768-dim embeddings)                │ │
│  │  - models/gemini-pro (chat completion)                      │ │
│  │                                                             │ │
│  │  Rate Limits (Free Tier):                                   │ │
│  │  - 60 requests/minute                                       │ │
│  │  - 1500 requests/day                                        │ │
│  └─────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

### Gemini Integration Details

**Embedding Model**: `models/embedding-001`
- Output dimensions: 768
- Context window: 2048 tokens
- Use case: Convert text chunks and user queries to vector representations

**Chat Model**: `models/gemini-pro`
- Context window: 32,000 tokens
- Temperature: 0.7 (configurable)
- Use case: Generate responses based on question + retrieved context

### Qdrant Vector Database Design

**Collection Name**: `robotics_textbook_chunks`

**Vector Configuration**:
- Size: 768 (matching Gemini embedding dimensions)
- Distance metric: Cosine similarity
- Index type: HNSW (Hierarchical Navigable Small World)

**Payload Schema**:
```json
{
  "text": "The actual chunk content...",
  "source": "/docs/module-1/introduction.md",
  "start_token": 0,
  "end_token": 500,
  "chapter": "Module 1: ROS 2 Fundamentals",
  "section": "1.1 Introduction",
  "token_count": 500,
  "chunk_index": 0,
  "total_chunks": 12,
  "file_modified": "2025-12-09T00:00:00Z"
}
```

**Search Configuration**:
- Top-k results: 5 (configurable)
- Score threshold: 0.7 (minimum similarity)
- Reranking: Optional post-processing based on metadata relevance

---

## Data Flow Specifications

### Normal RAG Mode Flow

**Trigger**: User submits question WITHOUT selected text

```
Step 1: User Input
├─ Input: {"question": "What is ROS 2?"}
└─ Frontend: ChatKit captures input, sends POST to /query

Step 2: Backend Receives Request
├─ FastAPI /query endpoint activated
├─ Validates question is non-empty
└─ Detects selectedText is null/undefined → Normal RAG mode

Step 3: Question Embedding
├─ Call Gemini Embedding API
├─ Input: "What is ROS 2?"
├─ Output: [768-dimensional vector]
└─ Latency: ~200-500ms

Step 4: Vector Search
├─ Query Qdrant with question embedding
├─ Parameters: top_k=5, score_threshold=0.7
├─ Retrieve: Top 5 most similar chunks
└─ Example results:
    {
      "chunks": [
        {"text": "ROS 2 is...", "score": 0.92, "source": "..."},
        {"text": "Key features...", "score": 0.88, "source": "..."},
        ...
      ]
    }

Step 5: Context Assembly
├─ Combine retrieved chunks into context string
├─ Format: "Context 1: [text]\n\nContext 2: [text]..."
├─ Add source citations
└─ Total context length: ~2000-2500 tokens

Step 6: Prompt Construction
├─ System prompt: "You are a helpful AI assistant..."
├─ Context injection: "[Retrieved Context]"
├─ User question: "What is ROS 2?"
└─ Full prompt: ~2500-3000 tokens

Step 7: Gemini Chat API Call
├─ Model: gemini-pro
├─ Input: Constructed prompt
├─ Output: Generated response text
└─ Latency: ~1-3 seconds

Step 8: Response Formatting
├─ Extract answer text
├─ Add source citations (optional)
├─ Format as JSON: {"answer": "...", "sources": [...]}
└─ Return to frontend

Step 9: Frontend Display
├─ ChatKit receives response
├─ Renders answer in chat UI
└─ Total time: <5 seconds (P95)
```

### Selected Text Mode Flow

**Trigger**: User submits question WITH selected text

```
Step 1: User Input
├─ User highlights text in textbook
├─ Input: {
│   "question": "Explain this concept",
│   "selectedText": "ROS 2 provides a distributed..."
│ }
└─ Frontend: ChatKit captures both, sends POST to /query

Step 2: Backend Receives Request
├─ FastAPI /query endpoint activated
├─ Validates both question and selectedText are non-empty
└─ Detects selectedText is present → Selected Text mode

Step 3: Skip Vector Search
├─ Bypass embedding generation
├─ Bypass Qdrant query
└─ Use selectedText directly as context

Step 4: Prompt Construction
├─ System prompt: "You are a helpful AI assistant..."
├─ Context: [selectedText provided by user]
├─ User question: "Explain this concept"
└─ Full prompt: variable length based on selection

Step 5: Gemini Chat API Call
├─ Model: gemini-pro
├─ Input: Constructed prompt with selectedText
├─ Output: Generated response focused on selected text
└─ Latency: ~1-2 seconds (faster than Normal RAG)

Step 6: Response Formatting
├─ Extract answer text
├─ Format as JSON: {"answer": "...", "mode": "selected_text"}
└─ Return to frontend

Step 7: Frontend Display
├─ ChatKit receives response
├─ Renders answer in chat UI
└─ Total time: <3 seconds (P95)
```

---

## Chunking Strategy Specification

### Document Processing Pipeline

**Input**: All markdown files in `/docs` directory (recursive)

**Output**: Individual text chunks with metadata

### Chunking Parameters

- **Target chunk size**: 500 tokens
- **Overlap size**: 150 tokens
- **Tokenizer**: Gemini-compatible tokenizer (tiktoken or similar)
- **Minimum chunk size**: 100 tokens (discard smaller chunks)
- **Maximum chunk size**: 600 tokens (hard limit)

### Chunking Algorithm

```
For each markdown file:
  1. Read file content
  2. Extract metadata (file path, chapter, section)
  3. Split on markdown headers (##, ###) to respect logical boundaries
  4. For each section:
     a. Tokenize text
     b. If tokens <= 600: Keep as single chunk
     c. If tokens > 600: Apply sliding window
        - Window size: 500 tokens
        - Step size: 350 tokens (500 - 150 overlap)
        - Create chunks until section is fully covered
  5. For each chunk:
     a. Record start_token and end_token positions
     b. Store original text
     c. Attach metadata (source, chapter, section)
     d. Assign unique chunk ID
  6. Return list of chunks
```

### Metadata Extraction

Each chunk includes:
- **source**: Full file path (e.g., `/docs/module-1/ros2-intro.md`)
- **chapter**: Extracted from directory structure or file header
- **section**: Extracted from nearest markdown heading
- **start_token**: Token position in original document
- **end_token**: Token position in original document
- **token_count**: Actual number of tokens in chunk
- **chunk_index**: Sequential index within document
- **total_chunks**: Total chunks from this document
- **file_modified**: Last modification timestamp

### Example Chunking Output

**Input Document** (`/docs/module-1/ros2-intro.md`):
```markdown
# Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework...
[1000 tokens of content]
```

**Output Chunks**:
```json
[
  {
    "id": "uuid-1",
    "text": "ROS 2 (Robot Operating System 2) is a flexible framework...",
    "source": "/docs/module-1/ros2-intro.md",
    "chapter": "Module 1: ROS 2 Fundamentals",
    "section": "Introduction to ROS 2",
    "start_token": 0,
    "end_token": 500,
    "token_count": 500,
    "chunk_index": 0,
    "total_chunks": 3
  },
  {
    "id": "uuid-2",
    "text": "[overlapping 150 tokens]... ROS 2 provides...",
    "source": "/docs/module-1/ros2-intro.md",
    "chapter": "Module 1: ROS 2 Fundamentals",
    "section": "Introduction to ROS 2",
    "start_token": 350,
    "end_token": 850,
    "token_count": 500,
    "chunk_index": 1,
    "total_chunks": 3
  }
]
```

---

## File Specifications (/rag Directory)

### File Structure

```
/rag/
├── ingest.py              # Document ingestion and embedding script
├── rag_server.py          # FastAPI backend server
├── requirements.txt       # Python dependencies
├── vercel.json           # Vercel deployment configuration
├── config.env.example    # Environment variable template
└── README.md             # Setup and usage documentation
```

### ingest.py Specification

**Purpose**: One-time (or periodic) script to process `/docs` markdown files, generate embeddings, and upload to Qdrant

**Key Functions**:

1. **load_documents()**: Recursively read all .md files from `/docs`
2. **chunk_document(content, metadata)**: Apply chunking strategy
3. **generate_embeddings(chunks)**: Batch call Gemini Embedding API
4. **upload_to_qdrant(chunks, embeddings)**: Create/update Qdrant collection

**CLI Usage**:
```bash
python ingest.py --docs-dir ../docs --collection robotics_textbook_chunks
```

**Expected Inputs**:
- `--docs-dir`: Path to documentation directory (default: `../docs`)
- `--collection`: Qdrant collection name (default: `robotics_textbook_chunks`)
- `--batch-size`: Embedding batch size (default: 10)
- `--recreate`: Flag to drop and recreate collection (default: False)

**Expected Outputs**:
- Console logs showing progress
- Summary statistics (files processed, chunks created, embeddings generated)
- Qdrant collection populated with vectors

**Error Handling**:
- Handle missing API keys gracefully
- Retry failed embedding calls (max 3 retries)
- Skip files with read errors (log and continue)
- Validate chunk count before upload

**Dependencies**:
- `google-generativeai`: Gemini API client
- `qdrant-client`: Qdrant Python SDK
- `tiktoken`: Tokenization
- `python-dotenv`: Environment variable loading
- `tqdm`: Progress bars

### rag_server.py Specification

**Purpose**: FastAPI server exposing `/query` endpoint for dual-mode RAG

**Endpoints**:

1. **POST /query**
   - Input: `{"question": str, "selectedText": Optional[str]}`
   - Output: `{"answer": str, "sources": Optional[List], "mode": str}`
   - Logic: Detect mode, execute RAG flow, return response

2. **GET /health**
   - Output: `{"status": "healthy", "qdrant_connected": bool}`
   - Logic: Health check for monitoring

**Key Functions**:

1. **embed_query(question: str) -> List[float]**: Call Gemini to embed question
2. **search_qdrant(embedding: List[float], top_k: int) -> List[Dict]**: Retrieve chunks
3. **generate_response(question: str, context: str) -> str**: Call Gemini chat
4. **normal_rag_mode(question: str) -> Dict**: Full RAG pipeline
5. **selected_text_mode(question: str, selected_text: str) -> Dict**: Direct generation

**Configuration**:
- Port: 8000 (configurable via env)
- CORS: Allow all origins (or configurable whitelist)
- Timeout: 30 seconds per request
- Max request size: 10KB

**Error Handling**:
- 400: Invalid request (missing question, empty fields)
- 500: Internal server error (API failures, DB errors)
- 503: Service unavailable (Gemini/Qdrant down)

**Logging**:
- Request/response logging
- Error tracing
- Performance metrics (response time)

**Dependencies**:
- `fastapi`: Web framework
- `uvicorn`: ASGI server
- `google-generativeai`: Gemini client
- `qdrant-client`: Vector DB client
- `pydantic`: Request/response validation

### requirements.txt Specification

**Content**:
```txt
fastapi==0.104.1
uvicorn[standard]==0.24.0
google-generativeai==0.3.1
qdrant-client==1.7.0
python-dotenv==1.0.0
tiktoken==0.5.2
pydantic==2.5.0
aiohttp==3.9.1
tqdm==4.66.1
```

**Version Constraints**:
- Pin exact versions for reproducibility
- Use latest stable versions as of 2025-12
- Ensure compatibility with Vercel Python runtime (3.9+)

### vercel.json Specification

**Purpose**: Configure Vercel serverless deployment

**Content**:
```json
{
  "version": 2,
  "builds": [
    {
      "src": "rag/rag_server.py",
      "use": "@vercel/python",
      "config": {
        "maxDuration": 30
      }
    }
  ],
  "routes": [
    {
      "src": "/api/query",
      "dest": "rag/rag_server.py"
    },
    {
      "src": "/api/health",
      "dest": "rag/rag_server.py"
    }
  ],
  "env": {
    "GEMINI_API_KEY": "@gemini-api-key",
    "QDRANT_URL": "@qdrant-url",
    "QDRANT_API_KEY": "@qdrant-api-key"
  }
}
```

**Key Configurations**:
- `maxDuration`: 30 seconds (max for free tier)
- Routes: Map `/api/*` to FastAPI app
- Environment variables: Securely reference Vercel secrets

### config.env.example Specification

**Purpose**: Template for local development environment variables

**Content**:
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

**Usage**:
1. Copy to `.env`: `cp config.env.example .env`
2. Fill in actual API keys
3. Load via `python-dotenv`

**Security Notes**:
- Never commit `.env` to Git
- Add `.env` to `.gitignore`
- Use Vercel secrets for production

---

## FastAPI Endpoint Specification

### POST /query

**Description**: Primary endpoint for processing user questions in dual-mode RAG

**URL**: `/api/query`

**Method**: POST

**Request Headers**:
```
Content-Type: application/json
```

**Request Body Schema**:
```json
{
  "question": "string (required, min length: 1, max length: 2000)",
  "selectedText": "string (optional, max length: 10000)"
}
```

**Request Examples**:

*Normal RAG Mode*:
```json
{
  "question": "What are the key differences between ROS 1 and ROS 2?"
}
```

*Selected Text Mode*:
```json
{
  "question": "Explain this in simpler terms",
  "selectedText": "ROS 2 uses DDS (Data Distribution Service) as its middleware, providing real-time communication..."
}
```

**Response Schema (Success)**:
```json
{
  "answer": "string (generated response)",
  "mode": "string (enum: 'normal_rag' | 'selected_text')",
  "sources": [
    {
      "text": "string (chunk excerpt)",
      "source": "string (file path)",
      "score": "float (similarity score, 0-1)"
    }
  ],
  "response_time_ms": "integer"
}
```

**Response Example**:
```json
{
  "answer": "ROS 2 differs from ROS 1 in several key ways: 1) It uses DDS middleware for real-time communication, 2) It supports multiple platforms including Windows, 3) It has improved security features...",
  "mode": "normal_rag",
  "sources": [
    {
      "text": "ROS 2 uses DDS (Data Distribution Service)...",
      "source": "/docs/module-1/ros2-intro.md",
      "score": 0.92
    },
    {
      "text": "Unlike ROS 1, ROS 2 supports...",
      "source": "/docs/module-1/comparison.md",
      "score": 0.88
    }
  ],
  "response_time_ms": 2340
}
```

**Response Schema (Error)**:
```json
{
  "error": "string (error message)",
  "detail": "string (additional error context)",
  "status_code": "integer"
}
```

**Error Responses**:

| Status Code | Error Type | Description |
|-------------|------------|-------------|
| 400 | Bad Request | Missing or invalid question, empty fields |
| 429 | Rate Limit | Gemini API rate limit exceeded |
| 500 | Internal Server Error | Unexpected server error |
| 503 | Service Unavailable | Qdrant or Gemini API unreachable |

**Processing Logic**:

```python
async def query_endpoint(request: QueryRequest):
    # Step 1: Validate request
    if not request.question or len(request.question.strip()) == 0:
        raise HTTPException(400, "Question cannot be empty")

    start_time = time.time()

    # Step 2: Mode detection
    if request.selectedText and len(request.selectedText.strip()) > 0:
        mode = "selected_text"
        response = await selected_text_mode(
            question=request.question,
            selected_text=request.selectedText
        )
    else:
        mode = "normal_rag"
        response = await normal_rag_mode(
            question=request.question
        )

    # Step 3: Format response
    response_time = int((time.time() - start_time) * 1000)

    return {
        "answer": response["answer"],
        "mode": mode,
        "sources": response.get("sources", []),
        "response_time_ms": response_time
    }
```

**Rate Limiting**:
- Client-side: 10 requests per minute per IP (configurable)
- Gemini API: 60 requests per minute (enforced by API)
- Handle 429 errors gracefully with retry logic

**Timeout Handling**:
- Request timeout: 30 seconds
- Gemini API call timeout: 20 seconds
- Qdrant search timeout: 5 seconds

---

## ChatKit Frontend Specification

### Component Structure

```
/src/components/RagChat/
├── index.jsx               # Main component export
├── ChatKitWrapper.jsx      # ChatKit SDK integration
├── TextSelectionHandler.js # Text selection logic
└── styles.module.css       # Component-specific styles
```

### ChatKit SDK Integration

**Purpose**: Use OpenAI ChatKit SDK for UI rendering while connecting to custom FastAPI backend

**Key Implementation Points**:

1. **SDK Installation**:
   ```bash
   npm install @chatscope/chat-ui-kit-react
   ```

2. **Backend Configuration**:
   - Override default OpenAI endpoints
   - Point to custom FastAPI `/api/query` endpoint
   - Handle custom response format

3. **Message Handling**:
   - User messages: Capture question + optional selected text
   - Assistant messages: Display Gemini-generated responses
   - System messages: Show mode indicators, errors

### ChatKitWrapper.jsx Specification

**Purpose**: React component wrapping ChatKit SDK with custom backend logic

**Props**:
```typescript
interface ChatKitWrapperProps {
  backendUrl: string;          // FastAPI server URL
  enableTextSelection: boolean; // Enable/disable selected text mode
  maxMessages: number;          // Message history limit (default: 50)
  theme: 'light' | 'dark';     // UI theme
}
```

**State Management**:
```typescript
interface ChatState {
  messages: Message[];
  isLoading: boolean;
  selectedText: string | null;
  error: string | null;
}
```

**Key Functions**:

1. **handleSendMessage(message: string)**:
   - Capture user input
   - Check if selectedText exists
   - Call backend API
   - Update message history

2. **handleTextSelection()**:
   - Listen for text selection events
   - Store selected text in state
   - Show visual indicator of selected text
   - Clear selection after query

3. **callBackendAPI(question: string, selectedText?: string)**:
   - Construct request body
   - POST to `/api/query`
   - Handle response/errors
   - Update UI

**Example Implementation**:
```jsx
import { ChatContainer, MessageList, Message, MessageInput } from '@chatscope/chat-ui-kit-react';

const ChatKitWrapper = ({ backendUrl, enableTextSelection = true }) => {
  const [messages, setMessages] = useState([]);
  const [selectedText, setSelectedText] = useState(null);
  const [isLoading, setIsLoading] = useState(false);

  const handleSendMessage = async (messageText) => {
    // Add user message to UI
    const userMessage = {
      message: messageText,
      sender: "user",
      direction: "outgoing"
    };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Call backend
      const response = await fetch(`${backendUrl}/api/query`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          question: messageText,
          selectedText: selectedText
        })
      });

      const data = await response.json();

      // Add assistant response
      const assistantMessage = {
        message: data.answer,
        sender: "assistant",
        direction: "incoming",
        metadata: {
          mode: data.mode,
          sources: data.sources
        }
      };
      setMessages(prev => [...prev, assistantMessage]);

      // Clear selected text after use
      setSelectedText(null);
    } catch (error) {
      // Handle errors
      const errorMessage = {
        message: "Sorry, I encountered an error. Please try again.",
        sender: "system",
        direction: "incoming"
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <ChatContainer>
      <MessageList>
        {messages.map((msg, i) => (
          <Message key={i} model={msg} />
        ))}
        {isLoading && <Message model={{ message: "Thinking...", sender: "assistant" }} />}
      </MessageList>
      <MessageInput
        placeholder="Ask a question..."
        onSend={handleSendMessage}
        disabled={isLoading}
      />
      {selectedText && (
        <div className="selected-text-indicator">
          Selected text: {selectedText.substring(0, 50)}...
        </div>
      )}
    </ChatContainer>
  );
};
```

### Text Selection Handler

**Purpose**: Capture text selections from the Docusaurus page and pass to chat component

**Implementation Strategy**:

1. **Selection Detection**:
   ```javascript
   document.addEventListener('mouseup', () => {
     const selection = window.getSelection();
     const selectedText = selection.toString().trim();
     if (selectedText.length > 0) {
       // Store in component state or global context
       setSelectedText(selectedText);
     }
   });
   ```

2. **Visual Feedback**:
   - Show floating button near selection: "Ask about this"
   - Highlight selected text in chat input area
   - Clear indicator after query submission

3. **Integration with ChatKit**:
   - Pass selectedText as prop to ChatKitWrapper
   - Auto-populate context indicator
   - Send with next user message

### Backend URL Configuration

**Environment-based Configuration**:

```javascript
// config.js
const getBackendUrl = () => {
  if (process.env.NODE_ENV === 'development') {
    return 'http://localhost:8000';
  }
  return process.env.REACT_APP_RAG_BACKEND_URL || 'https://your-vercel-app.vercel.app';
};

export default getBackendUrl;
```

**Usage in Component**:
```jsx
import getBackendUrl from './config';

const RagChat = () => {
  const backendUrl = getBackendUrl();
  return <ChatKitWrapper backendUrl={backendUrl} />;
};
```

### Styling Considerations

**Custom CSS** (`/static/ragchat.css`):
```css
.rag-chat-container {
  position: fixed;
  bottom: 20px;
  right: 20px;
  width: 400px;
  height: 600px;
  border-radius: 10px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  z-index: 1000;
}

.selected-text-indicator {
  padding: 8px;
  background-color: #f0f8ff;
  border-left: 3px solid #007bff;
  font-size: 12px;
  margin: 5px 0;
}

.message-sources {
  font-size: 11px;
  color: #666;
  margin-top: 5px;
}
```

---

## Deployment Specification

### Vercel Serverless Setup

**Prerequisites**:
- Vercel account (free tier)
- GitHub repository connected to Vercel
- Gemini API key (free tier)
- Qdrant Cloud account (free tier)

### Deployment Steps

**Step 1: Prepare Repository**

1. Ensure `/rag` directory structure is correct:
   ```
   /rag/
   ├── ingest.py
   ├── rag_server.py
   ├── requirements.txt
   └── vercel.json
   ```

2. Add `.vercelignore`:
   ```
   .env
   __pycache__/
   *.pyc
   .pytest_cache/
   ```

3. Commit changes to Git

**Step 2: Configure Vercel Project**

1. **Link Repository**:
   - Go to Vercel Dashboard
   - Click "New Project"
   - Import GitHub repository
   - Select root directory

2. **Set Environment Variables**:
   ```
   GEMINI_API_KEY=<your-gemini-key>
   QDRANT_URL=https://<your-cluster>.qdrant.io
   QDRANT_API_KEY=<your-qdrant-key>
   QDRANT_COLLECTION=robotics_textbook_chunks
   ```

3. **Configure Build Settings**:
   - Framework Preset: Other
   - Root Directory: `.` (project root)
   - Build Command: (leave empty for serverless functions)
   - Output Directory: (leave empty)

**Step 3: Deploy Functions**

1. **Push to GitHub**:
   ```bash
   git add .
   git commit -m "Add RAG backend"
   git push origin 001-rag-chatbot
   ```

2. **Automatic Deployment**:
   - Vercel auto-deploys on push
   - Monitor deployment in Vercel dashboard
   - Check function logs for errors

3. **Test Endpoints**:
   ```bash
   # Health check
   curl https://your-app.vercel.app/api/health

   # Query test
   curl -X POST https://your-app.vercel.app/api/query \
     -H "Content-Type: application/json" \
     -d '{"question": "What is ROS 2?"}'
   ```

**Step 4: Run Ingestion Script**

1. **Local Execution** (one-time):
   ```bash
   cd /rag
   pip install -r requirements.txt
   cp config.env.example .env
   # Fill in .env with API keys
   python ingest.py --docs-dir ../docs
   ```

2. **Verify Qdrant**:
   - Log into Qdrant Cloud dashboard
   - Check collection `robotics_textbook_chunks` exists
   - Verify vector count matches expected chunks

**Step 5: Update Frontend Configuration**

1. **Set Backend URL** in frontend `.env`:
   ```
   REACT_APP_RAG_BACKEND_URL=https://your-app.vercel.app
   ```

2. **Rebuild Docusaurus**:
   ```bash
   npm run build
   ```

3. **Deploy Frontend** (GitHub Pages):
   ```bash
   npm run deploy
   ```

### Environment Setup Details

**Local Development**:

1. **Backend (FastAPI)**:
   ```bash
   cd /rag
   python -m venv venv
   source venv/bin/activate  # Windows: venv\Scripts\activate
   pip install -r requirements.txt
   cp config.env.example .env
   # Edit .env with real keys
   uvicorn rag_server:app --reload --port 8000
   ```

2. **Frontend (Docusaurus)**:
   ```bash
   npm install
   npm start  # Dev server at localhost:3000
   ```

3. **Test Integration**:
   - Open `http://localhost:3000`
   - Interact with chat component
   - Verify backend calls go to `localhost:8000`

**Production Environment**:

1. **Vercel (Backend)**:
   - Serverless functions auto-scale
   - Cold start latency: ~1-2 seconds
   - Warm requests: <500ms overhead

2. **GitHub Pages (Frontend)**:
   - Static site hosting
   - Global CDN
   - SSL/HTTPS enabled

3. **External Services**:
   - Gemini API: Free tier limits (60 req/min)
   - Qdrant Cloud: Free tier (1GB storage)

### Build & Deploy Flow

**Continuous Deployment Pipeline**:

```
┌─────────────────────────────────────────────────────────────┐
│ Developer pushes to 001-rag-chatbot branch                  │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ GitHub Actions Triggered                                    │
│  - Run tests (if any)                                       │
│  - Lint Python code                                         │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ Vercel Deployment (Backend)                                 │
│  1. Detect changes in /rag                                  │
│  2. Build serverless functions                              │
│  3. Deploy to Vercel edge network                           │
│  4. Run health checks                                       │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ Docusaurus Build (Frontend)                                 │
│  1. npm run build                                           │
│  2. Bundle React components                                 │
│  3. Generate static pages                                   │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ GitHub Pages Deployment                                     │
│  1. Push build output to gh-pages branch                    │
│  2. Deploy to GitHub Pages                                  │
│  3. Propagate to CDN                                        │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ Production Live                                             │
│  - Frontend: https://username.github.io/ai_robotics_book    │
│  - Backend: https://your-app.vercel.app/api                 │
└─────────────────────────────────────────────────────────────┘
```

**Rollback Strategy**:
- Vercel: Instant rollback to previous deployment via dashboard
- GitHub Pages: Revert commit and redeploy
- Qdrant: No rollback needed (data persists independently)

---

## Safety Boundaries

### Isolation Guarantees

**Principle**: The RAG system is an additive enhancement and MUST NOT modify existing textbook content or core infrastructure.

### Protected Directories (NO MODIFICATIONS ALLOWED)

1. **`/docs/`**: All markdown content files
   - No edits to existing chapters
   - No deletion of files
   - No restructuring of directories

2. **`/src/pages/`**: Docusaurus core pages
   - No changes to existing page components
   - No modifications to routing logic

3. **`/docusaurus.config.js`**: Main configuration
   - No changes unless explicitly required for minimal integration
   - Any changes must be documented and reversible

4. **`/src/theme/`**: Docusaurus theme customizations
   - No modifications to existing theme components
   - New components only in designated directories

### Permitted Modifications (Minimal Integration)

1. **New Directories**:
   - `/rag/`: All RAG backend code (isolated)
   - `/src/components/RagChat/`: Chat UI components (new, isolated)

2. **New Files**:
   - `/static/ragchat.css`: Chat styling (new file only)
   - `/rag/*`: All backend files (new directory)

3. **Optional Integration Points**:
   - IF needed: Import `<RagChat />` in a layout file
   - IF needed: Add single line to `docusaurus.config.js` for script loading
   - All integration points must be clearly documented

### Validation Checklist

**Before Deployment**:

- [ ] No files in `/docs/` have been modified (check git diff)
- [ ] No files in `/src/pages/` have been modified
- [ ] All RAG code resides in `/rag/` directory
- [ ] Chat components reside in `/src/components/RagChat/` only
- [ ] No changes to core Docusaurus configuration (or documented minimal changes)
- [ ] `.gitignore` includes `.env` and sensitive files
- [ ] No API keys committed to repository

**After Deployment**:

- [ ] Original textbook content displays correctly
- [ ] No broken links or missing pages
- [ ] RAG chat component functions independently
- [ ] Textbook works without RAG if backend is down (graceful degradation)
- [ ] No console errors related to integration

### Rollback Plan

**If RAG integration causes issues**:

1. **Immediate Mitigation**:
   - Remove `<RagChat />` import (if added to layout)
   - Comment out RAG-related scripts in config

2. **Full Rollback**:
   - Delete `/rag/` directory
   - Delete `/src/components/RagChat/` directory
   - Delete `/static/ragchat.css`
   - Revert any configuration changes
   - Redeploy frontend

3. **Backend Independence**:
   - RAG backend on Vercel can be disabled independently
   - Frontend remains fully functional without backend

### Testing Isolation

**Test Scenarios**:

1. **Backend Down**: Frontend should display error message in chat, but textbook remains accessible
2. **Frontend Without Chat**: Textbook functions completely without chat component
3. **Partial Integration**: Chat can be added/removed without affecting textbook content

---

## Assumptions & Dependencies

### Assumptions

1. All textbook content is in Markdown format in `/docs` directory
2. Gemini API free tier limits are sufficient for expected usage (~100 users/day)
3. Qdrant Cloud free tier (1GB) is sufficient for textbook embeddings
4. Users have modern browsers supporting JavaScript ES6+
5. Text selections work in standard Docusaurus theme
6. Vercel free tier supports expected traffic (100 concurrent users)

### External Dependencies

1. **Google Gemini API**:
   - Service availability: 99.9% uptime SLA
   - Rate limits: 60 req/min, 1500 req/day (free tier)
   - Pricing: Free tier sufficient, paid tiers available

2. **Qdrant Cloud**:
   - Service availability: 99.9% uptime SLA
   - Storage limit: 1GB (free tier)
   - Pricing: Free tier sufficient, paid tiers available

3. **Vercel**:
   - Serverless function limits: 10s execution (hobby), 100GB bandwidth/month
   - Deployment: Automatic on Git push
   - Pricing: Free tier sufficient, paid tiers available

4. **OpenAI ChatKit SDK**:
   - UI library (not API usage)
   - No costs, MIT license
   - Frontend-only dependency

### Technical Dependencies

1. **Python 3.9+**: Required for FastAPI and Gemini SDK
2. **Node.js 18+**: Required for Docusaurus build
3. **Git**: Version control and deployment
4. **NPM/Yarn**: Frontend package management

### Risk Mitigation

1. **API Rate Limits**: Implement client-side rate limiting and caching
2. **Service Downtime**: Graceful degradation (show error, textbook still works)
3. **Cost Overruns**: Monitor usage, set up alerts for free tier limits
4. **Data Privacy**: No PII stored, all queries ephemeral (no logging of user questions)

---

## Open Questions & Clarifications

*This section intentionally left empty. All requirements have been specified with reasonable defaults based on the constitution and industry standards.*

---

## Next Steps

After spec approval, proceed to:

1. **Run `/sp.plan`**: Create detailed technical architecture and implementation plan
2. **Run `/sp.tasks`**: Generate granular, testable tasks from the plan
3. **Run `/sp.implement`**: Execute tasks to build the RAG system

**Estimated Scope**:
- Backend development: 15-20 tasks
- Frontend integration: 10-12 tasks
- Deployment & testing: 8-10 tasks
- Total: ~40 tasks