---
id: "0002"
title: "Better Auth Neon Clarifications"
stage: spec
date: 2025-12-12
surface: agent
model: claude-opus-4-5-20251101
feature: 006-google-oauth-auth
branch: 006-google-oauth-auth
user: user
command: /sp.clarify
labels: ["better-auth", "neon", "oauth", "clarification", "architecture"]
links:
  spec: specs/006-google-oauth-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-google-oauth-auth/spec.md
tests:
  - null
---

## Prompt

clarify that we will use better auth with neon database for user

## Response snapshot

Conducted clarification session for Google OAuth Authentication feature. Asked 4 targeted questions to resolve architectural ambiguities:

1. **Session storage strategy** - Database sessions (Better Auth manages sessions in Neon PostgreSQL tables)
2. **Auth server architecture** - Separate Node.js auth service (handles OAuth, FastAPI validates sessions via shared Neon DB)
3. **Session expiry handling** - Silent refresh (auto-refresh tokens in background, no user interruption)
4. **Cross-origin cookies** - Same domain with subdomains (shared cookies across services)

Updated spec with:
- New Clarifications section with all 4 Q&A pairs
- 5 new Assumptions about Neon, Better Auth service, and deployment
- 3 new Functional Requirements (FR-013, FR-014, FR-015)
- Updated Key Entities with storage details
- Resolved 1 edge case inline

## Outcome

- Impact: High - resolved critical architecture decisions for auth implementation
- Tests: N/A - clarification phase
- Files: 1 modified (spec.md)
- Next prompts: /sp.plan
- Reflection: Efficient clarification session; user provided clear answers allowing quick resolution of key architectural decisions

## Evaluation notes (flywheel)

- Failure modes observed: File edit conflicts due to external modifications; resolved with sed commands
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
