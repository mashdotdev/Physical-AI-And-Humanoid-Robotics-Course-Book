# Tasks: RAG Chatbot for Documentation

**Input**: Design documents from `/specs/005-rag-chatbot/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Not explicitly requested in spec - omitting test-first tasks

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/` at repository root (Python FastAPI)
- **Frontend**: `book/src/components/ChatWidget/` (React in Docusaurus)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for both backend and frontend

- [x] T001 Create backend directory structure per plan: `backend/app/{rag,api,models}/`, `backend/scripts/`, `backend/tests/`
- [x] T002 [P] Initialize Python project with requirements.txt in `backend/requirements.txt` (fastapi, uvicorn, cohere, qdrant-client, openai, python-dotenv, pydantic, httpx)
- [x] T003 [P] Create environment template in `backend/.env.example` with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, OPENAI_API_KEY, CORS_ORIGINS
- [x] T004 [P] Create frontend directory structure per plan: `book/src/components/ChatWidget/`, `book/src/components/ChatWidget/hooks/`, `book/src/components/ChatWidget/services/`
- [x] T005 [P] Create swizzled Root component in `book/src/theme/Root.tsx` for global widget injection

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Backend Core

- [x] T006 Implement config module in `backend/app/config.py` with environment variable loading (Pydantic Settings)
- [x] T007 [P] Create Pydantic schemas in `backend/app/models/schemas.py` (ChatRequest, ChatResponse, Source, HealthResponse, ErrorResponse)
- [x] T008 [P] Create package __init__.py files in `backend/app/__init__.py`, `backend/app/rag/__init__.py`, `backend/app/api/__init__.py`, `backend/app/models/__init__.py`
- [x] T009 Implement FastAPI app skeleton in `backend/app/main.py` with CORS middleware and health endpoint
- [x] T010 Implement Cohere embeddings service in `backend/app/rag/embeddings.py` with embed_query() and embed_documents() methods
- [x] T011 Implement Qdrant vectorstore service in `backend/app/rag/vectorstore.py` with search() and upsert() methods
- [x] T012 Implement RAG chain orchestration in `backend/app/rag/chain.py` combining embeddings, retrieval, and OpenAI generation

### Content Ingestion (Required for RAG to work)

- [x] T013 Implement markdown chunking utility in `backend/scripts/ingest.py` with chunk_markdown() function (~500 tokens, 50 overlap)
- [x] T014 Implement metadata extraction in `backend/scripts/ingest.py` with extract_metadata() function (chapter, section, file_path)
- [x] T015 Implement full ingestion pipeline in `backend/scripts/ingest.py` with ingest_docs() function (scan → parse → chunk → embed → store)
- [x] T016 Create Qdrant collection setup script in `backend/scripts/create_collection.py`

### Frontend Core

- [x] T017 [P] Create API client service in `book/src/components/ChatWidget/services/chatApi.ts` with sendMessage() function
- [x] T018 [P] Create TypeScript types in `book/src/components/ChatWidget/types.ts` (Message, Source, ChatRequest, ChatResponse)

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Ask a Question About Book Content (Priority: P1) 🎯 MVP

**Goal**: Users can click the chat bubble, type a question, and receive an accurate answer with source references

**Independent Test**: Ask "What is ROS 2?" and verify the response cites chapter 1 content about the Robotic Nervous System

### Implementation for User Story 1

- [x] T019 [US1] Implement chat endpoint in `backend/app/api/chat.py` with POST /api/chat handler using RAG chain
- [x] T020 [US1] Register chat router in `backend/app/main.py`
- [x] T021 [P] [US1] Create ChatBubble component in `book/src/components/ChatWidget/ChatBubble.tsx` (floating button, bottom-right)
- [x] T022 [P] [US1] Create InputArea component in `book/src/components/ChatWidget/InputArea.tsx` (text input, send button, loading state)
- [x] T023 [P] [US1] Create MessageList component in `book/src/components/ChatWidget/MessageList.tsx` (display user/assistant messages with sources)
- [x] T024 [US1] Create ChatPanel component in `book/src/components/ChatWidget/ChatPanel.tsx` (combines header, MessageList, InputArea)
- [x] T025 [US1] Create useChat hook in `book/src/components/ChatWidget/hooks/useChat.ts` (message state, sendMessage, loading)
- [x] T026 [US1] Create main ChatWidget component in `book/src/components/ChatWidget/index.tsx` (orchestrates bubble/panel, calls useChat)
- [x] T027 [US1] Create widget styles in `book/src/components/ChatWidget/styles.module.css` (fixed positioning, panel layout, message bubbles)
- [x] T028 [US1] Add source reference rendering in MessageList showing chapter/section citations

**Checkpoint**: User Story 1 complete - users can ask questions and get answers with sources

---

## Phase 4: User Story 2 - View Conversation History (Priority: P2)

**Goal**: Users can scroll through previous questions and answers within the chat session

**Independent Test**: Ask 3 questions in sequence and scroll up to verify all previous exchanges are visible

### Implementation for User Story 2

- [x] T029 [US2] Extend useChat hook in `book/src/components/ChatWidget/hooks/useChat.ts` to persist messages in sessionStorage
- [x] T030 [US2] Update MessageList component in `book/src/components/ChatWidget/MessageList.tsx` with scrollable container and auto-scroll to bottom
- [x] T031 [US2] Add conversation restoration on widget open in `book/src/components/ChatWidget/index.tsx`
- [x] T032 [US2] Add conversation_id tracking to useChat hook for session continuity

**Checkpoint**: User Story 2 complete - conversation history persists within browser session

---

## Phase 5: User Story 3 - Minimize and Restore Chat (Priority: P2)

**Goal**: Users can minimize the chat to focus on reading, then restore it with conversation intact

**Independent Test**: Minimize chat, scroll the page, restore chat and verify conversation is preserved

### Implementation for User Story 3

- [x] T033 [US3] Add minimize button to ChatPanel header in `book/src/components/ChatWidget/ChatPanel.tsx`
- [x] T034 [US3] Implement isOpen state toggle in `book/src/components/ChatWidget/index.tsx` for minimize/restore
- [x] T035 [US3] Update styles in `book/src/components/ChatWidget/styles.module.css` for smooth minimize/expand animation
- [x] T036 [US3] Preserve scroll position when restoring panel in MessageList

**Checkpoint**: User Story 3 complete - chat can be minimized and restored seamlessly

---

## Phase 6: User Story 4 - Handle Out-of-Scope Questions (Priority: P3)

**Goal**: Chatbot gracefully handles questions not related to the book content

**Independent Test**: Ask "What's the weather today?" and verify a polite redirect response

### Implementation for User Story 4

- [x] T037 [US4] Update system prompt in `backend/app/rag/chain.py` to detect out-of-scope questions
- [x] T038 [US4] Add relevance score threshold check in `backend/app/rag/chain.py` (if score < 0.3, return scope message)
- [x] T039 [US4] Create out-of-scope response template listing available topics (ROS 2, Digital Twin, AI Brain, VLA)
- [x] T040 [US4] Add visual indicator in MessageList for out-of-scope responses (different styling)

**Checkpoint**: User Story 4 complete - out-of-scope questions handled gracefully

---

## Phase 7: Edge Cases & Error Handling

**Purpose**: Handle edge cases defined in spec

- [x] T041 [P] Add empty message prevention in InputArea (disable send button, show hint)
- [x] T042 [P] Add character limit (1000) validation in InputArea with truncation notice
- [x] T043 [P] Add connection error handling in chatApi.ts with retry button
- [x] T044 [P] Add loading state to prevent rapid successive submissions in useChat hook
- [x] T045 Update ErrorResponse handling in ChatPanel to display user-friendly messages

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements affecting multiple user stories

- [x] T046 [P] Add Dockerfile in `backend/Dockerfile` for containerized deployment
- [x] T047 [P] Update backend README in `backend/README.md` with setup and deployment instructions
- [ ] T048 Run document ingestion with `python backend/scripts/ingest.py` to index book content
- [ ] T049 End-to-end validation: follow `specs/005-rag-chatbot/quickstart.md` to verify complete flow
- [x] T050 [P] Add responsive styles for mobile viewports in `book/src/components/ChatWidget/styles.module.css`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - User stories can proceed in priority order (P1 → P2 → P3)
  - US2 and US3 are both P2, can run in parallel if desired
- **Edge Cases (Phase 7)**: Can run after US1 is complete
- **Polish (Phase 8)**: Depends on all user stories being complete

### User Story Dependencies

| Story | Priority | Depends On | Can Start After |
|-------|----------|------------|-----------------|
| US1 | P1 | Phase 2 (Foundational) | T018 complete |
| US2 | P2 | US1 (extends useChat) | T028 complete |
| US3 | P2 | US1 (extends ChatPanel) | T028 complete |
| US4 | P3 | US1 (extends chain.py) | T028 complete |

### Within Each User Story

- Backend before frontend (API must exist for frontend to call)
- Components before hooks that use them
- Individual components before orchestrating component
- Styles can be added incrementally

### Parallel Opportunities

**Phase 1 (Setup)**:
```
T002, T003, T004, T005 can all run in parallel after T001
```

**Phase 2 (Foundational)**:
```
T007, T008 can run in parallel
T010, T011 can run in parallel (both depend on T006)
T017, T018 can run in parallel (frontend core)
```

**Phase 3 (US1)**:
```
T021, T022, T023 can run in parallel (independent components)
```

**Phase 7 (Edge Cases)**:
```
T041, T042, T043, T044 can all run in parallel
```

---

## Parallel Example: Phase 2 Foundational

```bash
# After T006 (config) completes, launch these in parallel:
Task: "T007 Create Pydantic schemas in backend/app/models/schemas.py"
Task: "T008 Create package __init__.py files"
Task: "T010 Implement Cohere embeddings service in backend/app/rag/embeddings.py"
Task: "T011 Implement Qdrant vectorstore service in backend/app/rag/vectorstore.py"

# Frontend tasks can run in parallel with backend:
Task: "T017 Create API client service in book/src/components/ChatWidget/services/chatApi.ts"
Task: "T018 Create TypeScript types in book/src/components/ChatWidget/types.ts"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T018) - **CRITICAL: blocks all stories**
3. Complete Phase 3: User Story 1 (T019-T028)
4. **STOP and VALIDATE**: Test asking "What is ROS 2?"
5. Run `python backend/scripts/ingest.py` to index docs
6. Deploy/demo if ready - **MVP complete!**

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy (MVP!)
3. Add User Story 2 → Test independently → Deploy
4. Add User Story 3 → Test independently → Deploy
5. Add User Story 4 → Test independently → Deploy
6. Polish phase → Final deployment

### Suggested MVP Scope

**Minimum for demo**: Phases 1-3 (T001-T028) = 28 tasks
- Users can ask questions about the book
- Responses include source references
- Basic but complete chat experience

---

## Task Summary

| Phase | Tasks | Description |
|-------|-------|-------------|
| Phase 1: Setup | T001-T005 (5) | Project structure and config |
| Phase 2: Foundational | T006-T018 (13) | Core services and ingestion |
| Phase 3: US1 (P1) | T019-T028 (10) | Ask questions - MVP |
| Phase 4: US2 (P2) | T029-T032 (4) | Conversation history |
| Phase 5: US3 (P2) | T033-T036 (4) | Minimize/restore |
| Phase 6: US4 (P3) | T037-T040 (4) | Out-of-scope handling |
| Phase 7: Edge Cases | T041-T045 (5) | Error handling |
| Phase 8: Polish | T046-T050 (5) | Deployment and validation |
| **Total** | **50 tasks** | |

### Tasks per User Story

| User Story | Task Count | Priority |
|------------|------------|----------|
| US1 | 10 | P1 (MVP) |
| US2 | 4 | P2 |
| US3 | 4 | P2 |
| US4 | 4 | P3 |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Backend API must be running for frontend to work
- Run ingestion script (T048) before testing - vectors must be in Qdrant
