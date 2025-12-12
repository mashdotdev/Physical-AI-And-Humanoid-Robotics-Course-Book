# Tasks: Google OAuth Authentication

**Input**: Design documents from `/specs/006-google-oauth-auth/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Tests**: Not explicitly requested - tests are omitted per task generation rules.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md, this is a web application with three services:
- **Backend (FastAPI)**: `backend/app/`
- **Auth Service (Better Auth)**: `auth/src/`
- **Frontend (Docusaurus)**: `book/src/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize the auth service project and add dependencies to existing projects

- [x] T001 Create auth service directory structure with auth/src/, auth/package.json, auth/tsconfig.json
- [x] T002 [P] Initialize auth service Node.js project with better-auth, pg, dotenv dependencies in auth/package.json
- [x] T003 [P] Create auth service TypeScript configuration in auth/tsconfig.json
- [x] T004 [P] Add environment variables template in auth/.env.example
- [x] T005 [P] Add asyncpg and python-jose[cryptography] to backend/requirements.txt
- [x] T006 [P] Add better-auth client dependency to book/package.json
- [x] T007 Create Vercel deployment configuration in auth/vercel.json

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

### Database & Auth Service Core

- [x] T008 Implement Better Auth server configuration with Google OAuth provider in auth/src/auth.ts
- [x] T009 Create Better Auth server entry point with HTTP handler in auth/src/index.ts
- [ ] T010 Run Better Auth CLI migration to create user, session, account tables in Neon (npx @better-auth/cli migrate)

### Backend Database Connection

- [x] T011 Create Neon database connection pool module in backend/app/db/__init__.py
- [x] T012 Implement async database pool with startup/shutdown lifecycle in backend/app/db/session.py
- [x] T013 Add DATABASE_URL to backend/app/config.py settings

### Backend Auth Infrastructure

- [x] T014 Create auth module structure in backend/app/auth/__init__.py
- [x] T015 Implement session validation dependency with Neon query in backend/app/auth/dependencies.py
- [x] T016 Create Pydantic models for User, Session, CurrentUser in backend/app/models/auth.py
- [x] T017 Update CORS middleware to include auth subdomain and allow_credentials in backend/app/main.py
- [x] T018 Add database pool initialization to app lifespan events in backend/app/main.py

### Frontend Auth Client

- [x] T019 Create Better Auth client configuration in book/src/lib/auth-client.ts
- [x] T020 Create useAuth hook with session state, signIn, signOut in book/src/components/ChatWidget/hooks/useAuth.ts

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Sign In with Google (Priority: P1) MVP

**Goal**: Enable visitors to authenticate with Google before accessing the chatbot

**Independent Test**: Open chat widget, click "Sign in with Google", complete OAuth flow, verify chat interface appears

### Implementation for User Story 1

- [x] T021 [US1] Create SignInButton component with Google sign-in trigger in book/src/components/Auth/SignInButton.tsx
- [x] T022 [US1] Create AuthGate wrapper component that shows sign-in prompt for unauthenticated users in book/src/components/Auth/AuthGate.tsx
- [x] T023 [US1] Add loading state component for auth check in book/src/components/Auth/AuthLoading.tsx
- [x] T024 [US1] Integrate AuthGate into ChatWidget to gate chat access in book/src/components/ChatWidget/index.tsx
- [x] T025 [US1] Add error handling for OAuth errors (access_denied, invalid_request) in book/src/components/Auth/AuthError.tsx
- [x] T026 [US1] Add auth component styles (sign-in button, error states) in book/src/css/custom.css

**Checkpoint**: User Story 1 complete - users can sign in with Google and see chat interface

---

## Phase 4: User Story 2 - Authenticated Chat Session (Priority: P1)

**Goal**: Enable authenticated users to chat with the RAG assistant with backend session validation

**Independent Test**: Sign in, send chat message, verify response is received; send request without auth, verify 401

### Implementation for User Story 2

- [x] T027 [US2] Apply get_current_user dependency to /api/chat endpoint in backend/app/api/chat.py
- [x] T028 [US2] Update chatApi.ts to include credentials: 'include' for cookie sending in book/src/components/ChatWidget/services/chatApi.ts
- [x] T029 [US2] Add 401 error handling in chat service to trigger re-authentication in book/src/components/ChatWidget/services/chatApi.ts
- [x] T030 [US2] Update useChat hook to handle authentication errors in book/src/components/ChatWidget/hooks/useChat.ts

**Checkpoint**: User Story 2 complete - authenticated chat works, unauthenticated requests are rejected

---

## Phase 5: User Story 3 - Sign Out (Priority: P2)

**Goal**: Enable authenticated users to sign out and terminate their session

**Independent Test**: Sign in, click sign out, verify sign-in prompt appears, verify chat requests now fail

### Implementation for User Story 3

- [x] T031 [US3] Create SignOutButton component with session termination in book/src/components/Auth/SignOutButton.tsx
- [x] T032 [US3] Create UserMenu component with avatar and sign-out option in book/src/components/Auth/UserMenu.tsx
- [x] T033 [US3] Add UserMenu to ChatHeader for authenticated users in book/src/components/ChatWidget/ChatHeader.tsx
- [x] T034 [US3] Add user menu and sign-out button styles in book/src/css/custom.css

**Checkpoint**: User Story 3 complete - users can sign out and are returned to sign-in state

---

## Phase 6: User Story 4 - Session Persistence (Priority: P2)

**Goal**: Maintain user sessions across page reloads and browser tabs for 7 days

**Independent Test**: Sign in, close tab, reopen site, verify still authenticated

### Implementation for User Story 4

- [x] T035 [US4] Configure session cookie with 7-day expiry and same-domain settings in auth/src/auth.ts
- [x] T036 [US4] Implement session refresh on activity (updateAge: 1 day) in auth/src/auth.ts
- [x] T037 [US4] Add session expiration check with silent re-auth on useAuth mount in book/src/components/ChatWidget/hooks/useAuth.ts

**Checkpoint**: User Story 4 complete - sessions persist across reloads for 7 days

---

## Phase 7: User Story 5 - User Profile Display (Priority: P3)

**Goal**: Display authenticated user Google profile (name, avatar) in chat interface

**Independent Test**: Sign in with Google, verify profile picture and name appear in chat header

### Implementation for User Story 5

- [x] T038 [P] [US5] Create UserAvatar component with Google image or fallback initials in book/src/components/Auth/UserAvatar.tsx
- [x] T039 [P] [US5] Create UserProfile component showing name and email in book/src/components/Auth/UserProfile.tsx
- [x] T040 [US5] Integrate UserAvatar into ChatHeader in book/src/components/ChatWidget/ChatHeader.tsx
- [x] T041 [US5] Add avatar and profile styles (rounded avatar, fallback initials) in book/src/css/custom.css

**Checkpoint**: User Story 5 complete - user profile info displays in chat UI

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T042 [P] Add environment variable documentation to README or quickstart.md
- [x] T043 [P] Validate all CORS origins for production deployment
- [x] T044 [P] Add error logging for auth failures in backend/app/auth/dependencies.py
- [x] T045 Run quickstart.md validation - test full local development flow
- [x] T046 Verify Vercel deployment configuration for all three services

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - US1 and US2 are both P1 priority but can be done sequentially
  - US3 and US4 are P2 and can start after P1 stories
  - US5 is P3 and can start after P2 stories
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Sign In with Google - No dependencies on other stories
- **User Story 2 (P1)**: Authenticated Chat - Depends on US1 (need user to be signed in to test)
- **User Story 3 (P2)**: Sign Out - Depends on US1 (need user to be signed in to sign out)
- **User Story 4 (P2)**: Session Persistence - Depends on US1 (need sessions to exist)
- **User Story 5 (P3)**: User Profile Display - Depends on US1 (need user data)

### Within Each User Story

- Core implementation before integration
- Frontend components before integration into parent components
- Backend changes before frontend integration

### Parallel Opportunities

**Phase 1 (Setup)** - Run in parallel:
```bash
T002, T003, T004, T005, T006 (all [P] marked)
```

**Phase 5 (US3)** - Run in parallel:
```bash
T031 (SignOutButton) + T032 (UserMenu) can be created in parallel
```

**Phase 7 (US5)** - Run in parallel:
```bash
T038 (UserAvatar) + T039 (UserProfile) are both [P]
```

**Phase 8 (Polish)** - Run in parallel:
```bash
T042, T043, T044 (all [P] marked)
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Sign In)
4. Complete Phase 4: User Story 2 (Authenticated Chat)
5. **STOP and VALIDATE**: Test sign-in flow and authenticated chat independently
6. Deploy/demo MVP

### Incremental Delivery

1. Setup + Foundational  Foundation ready
2. Add US1 (Sign In)  Test  Deploy (Can show login flow)
3. Add US2 (Auth Chat)  Test  Deploy (MVP: Full auth + chat)
4. Add US3 (Sign Out)  Test  Deploy (Complete session management)
5. Add US4 (Persistence)  Test  Deploy (Improved UX)
6. Add US5 (Profile)  Test  Deploy (Polish)
7. Add Polish phase  Final deployment

### Critical Path

```
Setup  Foundational  US1  US2  [MVP READY]
                           US3, US4 (P2 - parallel capable)
                                    US5 (P3)
                                          Polish
```

---

## Summary

| Phase | Task Count | Parallel Tasks |
|-------|------------|----------------|
| Phase 1: Setup | 7 | 5 |
| Phase 2: Foundational | 13 | 0 (sequential dependencies) |
| Phase 3: US1 - Sign In | 6 | 0 |
| Phase 4: US2 - Auth Chat | 4 | 0 |
| Phase 5: US3 - Sign Out | 4 | 2 |
| Phase 6: US4 - Persistence | 3 | 0 |
| Phase 7: US5 - Profile | 4 | 2 |
| Phase 8: Polish | 5 | 3 |
| **Total** | **46** | **12** |

### MVP Scope (Recommended)

Complete through Phase 4 (User Stories 1 + 2) for MVP:
- **Tasks**: T001-T030 (30 tasks)
- **Deliverables**: Google sign-in, authenticated chat access, 401 rejection of unauthenticated requests

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
