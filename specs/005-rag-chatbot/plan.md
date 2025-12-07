# Implementation Plan: RAG Chatbot for Documentation

**Branch**: `005-rag-chatbot` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-rag-chatbot/spec.md`

## Summary

Build a RAG (Retrieval-Augmented Generation) chatbot that enables readers to ask questions about the Physical AI textbook content. The system uses Cohere embeddings (free tier), Qdrant vector database (free tier), and OpenAI Chat API for response generation. A React chat widget embedded in the Docusaurus site connects to a Python FastAPI backend.

## Technical Context

**Language/Version**: Python 3.11 (Backend), TypeScript 5.x (Frontend Widget)
**Primary Dependencies**: FastAPI, Cohere SDK, Qdrant Client, OpenAI SDK, React 19
**Storage**: Qdrant Cloud (vector store), Browser sessionStorage (conversation state)
**Testing**: pytest (backend), Vitest (frontend)
**Target Platform**: Docusaurus static site + Cloud-hosted Python API
**Project Type**: Web application (frontend widget + backend API)
**Performance Goals**: <5s response time, 10 concurrent users
**Constraints**: Cohere free tier (1000 calls/month), Qdrant free tier (1GB), OpenAI pay-per-use
**Scale/Scope**: ~1MB documentation, ~100 document chunks, single collection

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| **Section 4.01 - Python 3.10+** | PASS | Using Python 3.11 |
| **Section 4.01 - OpenAI GPT-4o** | PASS | Using OpenAI Chat API (gpt-4o-mini) |
| **Section 7.01 - RAG Chatbot** | PASS | Implementing RAG with Qdrant as specified |
| **Section 7.01 - Docusaurus** | PASS | Widget integrates with existing Docusaurus site |
| **Article VI - Runnable Code** | PASS | All code will be complete and runnable |

**Deviation from Constitution**: Section 7.01 mentions "Neon Postgres" but we're using Qdrant only (no relational DB needed for this feature). This is simpler and aligns with the spec's vector-only storage requirement.

## Project Structure

### Documentation (this feature)

```text
specs/005-rag-chatbot/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   ├── chat-api.yaml    # OpenAPI spec for chat endpoint
│   └── schemas.json     # JSON schemas for entities
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── app/
│   ├── __init__.py
│   ├── main.py              # FastAPI application entry
│   ├── config.py            # Environment configuration
│   ├── rag/
│   │   ├── __init__.py
│   │   ├── embeddings.py    # Cohere embedding service
│   │   ├── vectorstore.py   # Qdrant operations
│   │   └── chain.py         # RAG pipeline orchestration
│   ├── api/
│   │   ├── __init__.py
│   │   └── chat.py          # Chat endpoint
│   └── models/
│       ├── __init__.py
│       └── schemas.py       # Pydantic models
├── scripts/
│   └── ingest.py            # Document ingestion script
├── tests/
│   ├── conftest.py
│   ├── test_embeddings.py
│   ├── test_vectorstore.py
│   └── test_chat.py
├── requirements.txt
├── Dockerfile
└── .env.example

book/src/components/
└── ChatWidget/
    ├── index.tsx            # Main widget component
    ├── ChatBubble.tsx       # Floating bubble button
    ├── ChatPanel.tsx        # Expanded chat interface
    ├── MessageList.tsx      # Conversation display
    ├── InputArea.tsx        # User input component
    ├── hooks/
    │   └── useChat.ts       # Chat state management
    ├── services/
    │   └── chatApi.ts       # Backend API client
    └── styles.module.css    # Widget styles
```

**Structure Decision**: Web application pattern with separate backend (Python FastAPI) and frontend (React component in Docusaurus). The backend handles RAG processing while the frontend provides the chat UI.

## Complexity Tracking

| Deviation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Separate backend service | RAG requires server-side processing (API keys, vector search) | Client-side would expose API keys and lack vector DB access |
| Three external services | Each serves distinct purpose (embed, store, generate) | All-in-one solutions don't have free tiers for all components |

## Architecture Decisions

### AD-001: Embedding Model Choice
**Decision**: Cohere `embed-english-v3.0`
**Rationale**: Free tier (1000 calls/month), high quality embeddings, sufficient for book content size
**Alternatives**: OpenAI embeddings (paid), local models (requires GPU)

### AD-002: Vector Database Choice
**Decision**: Qdrant Cloud
**Rationale**: 1GB free tier forever, excellent Python SDK, simple REST API
**Alternatives**: Pinecone (limited free tier), Chroma (harder to host)

### AD-003: LLM Choice
**Decision**: OpenAI `gpt-4o-mini`
**Rationale**: Best cost/quality ratio for RAG responses, strong instruction following
**Alternatives**: gpt-4o (more expensive), local models (requires GPU)

### AD-004: Chunking Strategy
**Decision**: Markdown-aware chunking with ~500 token chunks, 50 token overlap
**Rationale**: Preserves document structure (headers, code blocks), balances context vs retrieval precision
**Alternatives**: Fixed character split (loses structure), sentence splitting (too granular)

### AD-005: Frontend Integration
**Decision**: Custom React component injected via Docusaurus swizzling
**Rationale**: Full control over UX, no external dependencies, matches Docusaurus patterns
**Alternatives**: iframe embed (limited styling), third-party widget (less control)

## API Design Overview

### POST /api/chat
Request body:
- `message`: User's question (string, required, max 1000 chars)
- `conversation_id`: Session identifier (string, optional)

Response body:
- `response`: Generated answer (string)
- `sources`: Array of source references (chapter, section, relevance score)
- `conversation_id`: Session identifier for follow-up

### GET /api/health
Response: `{ "status": "healthy", "version": "1.0.0" }`

## Ingestion Pipeline

1. **Scan**: Find all `.md` files in `book/docs/`
2. **Parse**: Extract frontmatter (title, sidebar_position) and content
3. **Chunk**: Split into ~500 token segments with markdown awareness
4. **Enrich**: Add metadata (file path, chapter, section heading)
5. **Embed**: Generate vectors via Cohere API
6. **Store**: Upsert to Qdrant collection with metadata

## Deployment Considerations

- **Backend**: Deploy to Railway/Render/Fly.io (free tier options)
- **Frontend**: Bundled with Docusaurus, no separate deployment
- **Qdrant**: Use Qdrant Cloud free tier
- **Secrets**: Store API keys in environment variables, never in code
