---
id: "0002"
title: "RAG Chatbot Architecture Plan"
stage: plan
date: 2025-12-07
surface: agent
model: claude-opus-4-5-20251101
feature: 005-rag-chatbot
branch: 005-rag-chatbot
user: developer
command: /sp.plan
labels: ["rag", "chatbot", "architecture", "cohere", "qdrant", "openai", "fastapi", "react"]
links:
  spec: specs/005-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-rag-chatbot/plan.md
  - specs/005-rag-chatbot/research.md
  - specs/005-rag-chatbot/data-model.md
  - specs/005-rag-chatbot/quickstart.md
  - specs/005-rag-chatbot/contracts/chat-api.yaml
  - specs/005-rag-chatbot/contracts/schemas.json
tests:
  - null
---

## Prompt

/sp.plan

(Executed after /sp.specify for RAG chatbot feature with user requirements:
- OpenAI Chat API for LLM responses
- Qdrant for vector storage
- Cohere embeddings (free tier)
- React frontend widget (bottom-right corner)
- Python FastAPI backend)

## Response snapshot

Created comprehensive architecture plan for RAG Chatbot with:

**Phase 0 (Research)**:
- Cohere embed-english-v3.0 integration patterns
- Qdrant Cloud setup and Python client usage
- OpenAI Chat API with RAG system prompts
- Markdown-aware chunking strategy (~500 tokens)
- FastAPI async patterns with dependency injection
- Docusaurus swizzling for widget injection

**Phase 1 (Design)**:
- Data model with 4 entities: DocumentChunk, Message, Source, Conversation
- OpenAPI 3.1 specification for /api/chat and /api/health endpoints
- JSON schemas for all entities
- Complete quickstart guide with code samples

**Architecture Decisions**:
- AD-001: Cohere embeddings (free tier, 1024 dimensions)
- AD-002: Qdrant Cloud (1GB free tier)
- AD-003: gpt-4o-mini (cost/quality balance)
- AD-004: 500-token chunks with 50-token overlap
- AD-005: Docusaurus Root swizzling for global widget

**Project Structure**:
- backend/ - Python FastAPI with rag/, api/, models/ modules
- book/src/components/ChatWidget/ - React component with hooks and services

## Outcome

- Impact: Complete architecture plan ready for task generation
- Tests: N/A (planning phase)
- Files: 6 files created (plan.md, research.md, data-model.md, quickstart.md, chat-api.yaml, schemas.json)
- Next prompts: /sp.tasks to generate implementation tasks
- Reflection: All technology choices align with user's free-tier constraint; constitution check passed

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Constitution check PASS, all gates cleared
- Prompt variant (if applicable): N/A
- Next experiment: Proceed to task generation with /sp.tasks
