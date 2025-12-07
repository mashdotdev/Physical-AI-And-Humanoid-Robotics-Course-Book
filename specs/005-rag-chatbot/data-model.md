# Data Model: RAG Chatbot for Documentation

**Feature**: 005-rag-chatbot
**Date**: 2025-12-07

## Entity Overview

```
┌─────────────────┐       ┌─────────────────┐
│  DocumentChunk  │       │   Conversation  │
├─────────────────┤       ├─────────────────┤
│ id: string      │       │ id: string      │
│ text: string    │       │ messages: []    │
│ embedding: []   │       │ created_at: dt  │
│ metadata: {}    │       └────────┬────────┘
└─────────────────┘                │
                                   │ contains
                                   ▼
┌─────────────────┐       ┌─────────────────┐
│     Source      │◄──────│    Message      │
├─────────────────┤       ├─────────────────┤
│ chapter: string │       │ role: enum      │
│ section: string │       │ content: string │
│ file_path: str  │       │ sources: []     │
│ relevance: float│       │ timestamp: dt   │
└─────────────────┘       └─────────────────┘
```

## Entities

### DocumentChunk

A segment of documentation content stored in the vector database.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | string (UUID) | Yes | Unique identifier for the chunk |
| `text` | string | Yes | The actual text content (max 2000 chars) |
| `embedding` | float[1024] | Yes | Cohere embedding vector |
| `metadata` | ChunkMetadata | Yes | Source information |

**ChunkMetadata**:
| Field | Type | Description |
|-------|------|-------------|
| `file_path` | string | Relative path from book/docs/ |
| `chapter` | string | Chapter name (e.g., "Module 1: The Robotic Nervous System") |
| `section` | string | Section heading (e.g., "Understanding Robot Nervous System") |
| `position` | integer | Chunk position within the document |

**Validation Rules**:
- `text` must be non-empty, max 2000 characters
- `embedding` must have exactly 1024 dimensions
- `file_path` must match pattern `*.md`
- `position` must be >= 0

**Storage**: Qdrant collection `book_docs`

---

### Message

A single exchange in a conversation.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `role` | enum | Yes | "user" or "assistant" |
| `content` | string | Yes | Message text |
| `sources` | Source[] | No | Referenced sources (assistant only) |
| `timestamp` | datetime | Yes | When message was created |

**Validation Rules**:
- `role` must be "user" or "assistant"
- `content` must be non-empty
- `content` max length: 1000 (user), 2000 (assistant)
- `sources` only present when role is "assistant"

**Storage**: Browser sessionStorage (client-side only)

---

### Source

A reference to documentation content used in a response.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `chapter` | string | Yes | Chapter name |
| `section` | string | Yes | Section heading |
| `file_path` | string | Yes | Document path |
| `relevance` | float | Yes | Similarity score (0-1) |

**Validation Rules**:
- `relevance` must be between 0.0 and 1.0
- `file_path` must exist in indexed documents

---

### Conversation

A session-scoped collection of messages.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `id` | string (UUID) | Yes | Session identifier |
| `messages` | Message[] | Yes | Ordered list of messages |
| `created_at` | datetime | Yes | Session start time |

**Validation Rules**:
- `messages` must alternate user/assistant (starting with user)
- Maximum 50 messages per conversation

**Storage**: Browser sessionStorage (client-side only)

---

## Pydantic Models (Backend)

```python
from pydantic import BaseModel, Field
from datetime import datetime
from enum import Enum
from typing import Optional

class Role(str, Enum):
    USER = "user"
    ASSISTANT = "assistant"

class Source(BaseModel):
    chapter: str
    section: str
    file_path: str
    relevance: float = Field(ge=0.0, le=1.0)

class ChatRequest(BaseModel):
    message: str = Field(min_length=1, max_length=1000)
    conversation_id: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    sources: list[Source]
    conversation_id: str

class HealthResponse(BaseModel):
    status: str
    version: str
```

## TypeScript Types (Frontend)

```typescript
type Role = 'user' | 'assistant';

interface Source {
  chapter: string;
  section: string;
  filePath: string;
  relevance: number;
}

interface Message {
  role: Role;
  content: string;
  sources?: Source[];
  timestamp: Date;
}

interface Conversation {
  id: string;
  messages: Message[];
  createdAt: Date;
}

interface ChatRequest {
  message: string;
  conversationId?: string;
}

interface ChatResponse {
  response: string;
  sources: Source[];
  conversationId: string;
}
```

## Qdrant Collection Schema

```json
{
  "collection_name": "book_docs",
  "vectors_config": {
    "size": 1024,
    "distance": "Cosine"
  },
  "payload_schema": {
    "text": { "type": "text" },
    "file_path": { "type": "keyword" },
    "chapter": { "type": "keyword" },
    "section": { "type": "keyword" },
    "position": { "type": "integer" }
  }
}
```

## State Transitions

### Conversation State Machine

```
[Empty] ──user message──> [Active] ──assistant response──> [Active]
                              │
                              │──close widget──> [Persisted in sessionStorage]
                              │
                              │──browser close──> [Destroyed]
```

### Document Indexing States

```
[Raw MD] ──parse──> [Parsed] ──chunk──> [Chunked] ──embed──> [Embedded] ──store──> [Indexed]
```

## Indexes and Search

### Qdrant Index
- **Vector Index**: HNSW for approximate nearest neighbor search
- **Payload Index**: Keyword indexes on `chapter`, `section` for filtering

### Search Parameters
- **Top-K**: 5 (retrieve 5 most relevant chunks)
- **Score Threshold**: 0.3 (filter low relevance results)
- **Distance Metric**: Cosine similarity
