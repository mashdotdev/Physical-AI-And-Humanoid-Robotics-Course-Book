# Implementation Plan: Google OAuth Authentication

**Branch**: `006-google-oauth-auth` | **Date**: 2025-12-12 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/006-google-oauth-auth/spec.md`

## Summary

Add Google OAuth 2.0 authentication to the RAG chatbot using Better Auth as a separate Node.js authentication service with Neon PostgreSQL for session storage. The FastAPI backend validates sessions by querying the shared database. All services deploy on same domain with subdomains for cookie sharing. Silent token refresh maintains sessions without user interruption.

## Technical Context

**Language/Version**:
- Backend (FastAPI): Python 3.11
- Auth Service (Better Auth): Node.js 20+, TypeScript
- Frontend (Docusaurus): TypeScript 5.6.2, React 19.0.0

**Primary Dependencies**:
- Backend: FastAPI 0.109.0+, psycopg2/asyncpg (Neon), python-jose (JWT)
- Auth Service: Better Auth, @better-auth/pg (Postgres adapter), Google OAuth provider
- Frontend: Docusaurus 3.9.2, better-auth/react client

**Storage**: Neon PostgreSQL (users, sessions, accounts tables managed by Better Auth)

**Testing**:
- Backend: pytest with pytest-asyncio
- Auth Service: vitest or jest
- Frontend: React Testing Library

**Target Platform**:
- Backend: Vercel (Python serverless)
- Auth Service: Vercel (Node.js serverless)
- Frontend: Vercel (static)

**Project Type**: Web application (frontend + backend + auth service)

**Performance Goals**:
- Sign-in flow: <30 seconds (excluding Google consent screen)
- Sign-in prompt render: <500ms
- Sign-out response: <2 seconds
- OAuth callback success rate: 95%+

**Constraints**:
- Sessions persist 7 days
- 100% rejection of unauthenticated chat requests (401)
- HTTPS required for OAuth
- Same-domain subdomain deployment for cookie sharing

**Scale/Scope**: Documentation site users (initially <1000 concurrent users)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| **Simulation First, Reality Second** | N/A | Authentication is infrastructure, not robotics simulation |
| **Strict Hardware Alignment** | N/A | Web service, not hardware-specific |
| **Modern Stack Only** | PASS | Using modern auth (Better Auth), modern database (Neon PostgreSQL), latest React/TypeScript |
| **Agentic Workflow** | N/A | Auth is prerequisite infrastructure for chatbot AI |
| **Book Artifact Format** | PASS | Per Section 7.01: "Integrated RAG Chatbot... User sign-in (Better-Auth)" - this feature directly implements constitution mandate |
| **Code Runnable** | PASS | All code will be complete and runnable |
| **PHR Required** | PASS | PHRs will be created for all prompts |

**Gate Status**: PASSED - No violations. Better Auth integration is explicitly mandated by Constitution Section 7.01.

## Project Structure

### Documentation (this feature)

```text
specs/006-google-oauth-auth/
├── plan.md              # This file
├── research.md          # Phase 0: Better Auth + Neon research
├── data-model.md        # Phase 1: User/Session schema
├── quickstart.md        # Phase 1: Setup guide
├── contracts/           # Phase 1: API contracts
│   ├── auth-service.yaml    # Better Auth endpoints
│   └── backend-auth.yaml    # FastAPI auth middleware
└── tasks.md             # Phase 2 output (created by /sp.tasks)
```

### Source Code (repository root)

```text
# Existing Backend (FastAPI) - Add auth middleware
backend/
├── app/
│   ├── __init__.py
│   ├── main.py              # Add CORS for auth subdomain
│   ├── config.py            # Add Neon DATABASE_URL
│   ├── api/
│   │   ├── __init__.py
│   │   ├── chat.py          # Add auth middleware to /api/chat
│   │   └── health.py        # Existing health endpoint
│   ├── models/
│   │   ├── __init__.py
│   │   └── schemas.py       # Add User model for validation
│   ├── auth/                # NEW: Auth middleware
│   │   ├── __init__.py
│   │   ├── middleware.py    # Session validation middleware
│   │   └── dependencies.py  # FastAPI Depends for auth
│   └── db/                  # NEW: Database connection
│       ├── __init__.py
│       └── session.py       # Neon connection pool
├── tests/
│   ├── test_auth.py         # NEW: Auth middleware tests
│   └── test_chat.py         # Update with auth mocks
└── requirements.txt         # Add psycopg2-binary, python-jose

# NEW: Auth Service (Better Auth)
auth/
├── src/
│   ├── index.ts             # Better Auth server entry
│   ├── auth.ts              # Better Auth configuration
│   └── db.ts                # Neon database adapter
├── package.json
├── tsconfig.json
├── vercel.json              # Deployment config
└── .env.example

# Existing Frontend (Docusaurus) - Add auth components
book/
├── src/
│   ├── components/
│   │   ├── ChatWidget/
│   │   │   ├── index.tsx        # Add auth gate
│   │   │   ├── services/
│   │   │   │   ├── chatApi.ts   # Add auth headers
│   │   │   │   └── authApi.ts   # NEW: Auth client
│   │   │   └── hooks/
│   │   │       ├── useChat.ts   # Existing
│   │   │       └── useAuth.ts   # NEW: Auth state hook
│   │   └── Auth/                # NEW: Auth components
│   │       ├── SignInButton.tsx
│   │       ├── SignOutButton.tsx
│   │       ├── UserProfile.tsx
│   │       └── AuthProvider.tsx
│   └── css/
│       └── custom.css           # Auth component styles
└── package.json                 # Add better-auth client
```

**Structure Decision**: Web application with 3 services (frontend, backend, auth). The auth service is a new Node.js project at `/auth`. This is justified because Better Auth requires Node.js runtime and cannot run in Python. The frontend and backend are extended in-place.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| 3rd service (auth) | Better Auth requires Node.js runtime; cannot run in Python FastAPI | Python auth libraries (Authlib, etc.) do not provide Better Auth session management, Google OAuth integration, and TypeScript client SDK that the frontend needs |

---

*Next: Phase 0 Research (research.md)*
