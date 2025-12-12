# Research: Google OAuth Authentication

**Feature**: 006-google-oauth-auth | **Date**: 2025-12-12

## Overview

This document captures research findings for implementing Google OAuth authentication using Better Auth with Neon PostgreSQL.

---

## 1. Better Auth Configuration

### Decision: Use Better Auth with PostgreSQL Adapter

**Rationale**: Better Auth provides a complete OAuth solution with built-in session management, TypeScript support, and React hooks. The PostgreSQL adapter works directly with Neon.

**Alternatives Considered**:
- **NextAuth/Auth.js**: More Next.js-focused, less suited for standalone Docusaurus + FastAPI architecture
- **Passport.js**: Lower-level, requires more manual session management
- **Custom OAuth**: Higher maintenance burden, reinventing solved problems

### Better Auth Server Configuration

```typescript
// auth/src/auth.ts
import { betterAuth } from "better-auth";
import { Pool } from "pg";

export const auth = betterAuth({
    database: new Pool({
        connectionString: process.env.DATABASE_URL, // Neon connection string
    }),
    socialProviders: {
        google: {
            clientId: process.env.GOOGLE_CLIENT_ID as string,
            clientSecret: process.env.GOOGLE_CLIENT_SECRET as string,
            accessType: "offline", // For refresh tokens
            prompt: "select_account", // Always show account picker
        },
    },
    session: {
        expiresIn: 60 * 60 * 24 * 7, // 7 days in seconds
        updateAge: 60 * 60 * 24, // Update session every day
    },
});
```

### Better Auth React Client

```typescript
// book/src/lib/auth-client.ts
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
    baseURL: process.env.AUTH_URL || "https://auth.example.com",
});

export const { useSession, signIn, signOut } = authClient;
```

---

## 2. Neon PostgreSQL Integration

### Decision: Use Neon Serverless Driver

**Rationale**: Neon serverless driver is optimized for edge/serverless deployments, supports HTTP connections, and works seamlessly with Vercel.

**Alternatives Considered**:
- **Standard pg driver**: Works but less optimized for serverless cold starts
- **Prisma**: Adds ORM complexity not needed for Better Auth built-in adapter

### Connection Configuration

```typescript
// For Better Auth (Node.js)
import { Pool } from "pg";
const pool = new Pool({
    connectionString: process.env.DATABASE_URL,
    ssl: { rejectUnauthorized: false }, // Required for Neon
});
```

```python
# For FastAPI (Python) session validation
import asyncpg
pool = await asyncpg.create_pool(
    dsn=os.environ["DATABASE_URL"],
    ssl="require"
)
```

### Environment Variables

```env
# Neon Database
DATABASE_URL=postgres://user:password@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require

# Google OAuth
GOOGLE_CLIENT_ID=xxx.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=GOCSPX-xxx

# Better Auth
BETTER_AUTH_SECRET=your-32-char-secret-key
BETTER_AUTH_URL=https://auth.yourdomain.com
```

---

## 3. Database Schema

### Decision: Use Better Auth Auto-Managed Schema

**Rationale**: Better Auth CLI generates and migrates tables automatically, reducing manual schema maintenance.

**Tables Created by Better Auth**:

| Table | Purpose |
|-------|---------|
| user | User profiles (id, name, email, image, emailVerified, createdAt, updatedAt) |
| session | Active sessions (id, userId, token, expiresAt, ipAddress, userAgent) |
| account | OAuth accounts (id, userId, provider, providerAccountId, accessToken, refreshToken) |
| verification | Email verification tokens |

### Migration Command

```bash
npx @better-auth/cli migrate
```

---

## 4. FastAPI Session Validation

### Decision: Query Neon Directly for Session Validation

**Rationale**: FastAPI can validate sessions by querying the shared Neon database. This avoids network hops to the auth service for each request.

**Alternatives Considered**:
- **JWT-only validation**: Faster but loses server-side session revocation capability
- **Auth service API call**: Adds latency and single point of failure

### Implementation Pattern

```python
# backend/app/auth/dependencies.py
from fastapi import Depends, HTTPException, Request

async def get_current_user(request: Request) -> dict:
    session_token = request.cookies.get("better-auth.session_token")
    if not session_token:
        raise HTTPException(status_code=401, detail="Not authenticated")

    # Query session from Neon
    session = await request.app.state.db.fetchrow(
        """
        SELECT s.*, u.id as user_id, u.name, u.email, u.image
        FROM session s
        JOIN "user" u ON s."userId" = u.id
        WHERE s.token = $1 AND s."expiresAt" > NOW()
        """,
        session_token
    )

    if not session:
        raise HTTPException(status_code=401, detail="Invalid or expired session")

    return dict(session)
```

---

## 5. Frontend Authentication Flow

### Decision: Cookie-Based Sessions with Same-Domain Subdomains

**Rationale**: Cookies shared across subdomains provide seamless authentication without manual token management.

**Flow**:
1. User clicks "Sign in with Google"
2. Frontend calls `authClient.signIn.social({ provider: "google" })`
3. Better Auth redirects to Google consent screen
4. Google redirects back to auth service callback
5. Better Auth creates session, sets cookie (domain=.example.com)
6. Frontend reads session via `useSession()` hook
7. Chat API requests include cookie automatically

### React Hook Usage

```tsx
// book/src/components/ChatWidget/hooks/useAuth.ts
import { authClient } from "@/lib/auth-client";

export function useAuth() {
    const { data: session, isPending, error, refetch } = authClient.useSession();

    const signInWithGoogle = async () => {
        await authClient.signIn.social({ provider: "google" });
    };

    const signOutUser = async () => {
        await authClient.signOut();
        refetch();
    };

    return {
        user: session?.user,
        session: session?.session,
        isAuthenticated: !!session,
        isLoading: isPending,
        error,
        signInWithGoogle,
        signOut: signOutUser,
    };
}
```

---

## 6. Silent Token Refresh

### Decision: Better Auth Built-in Session Refresh

**Rationale**: Better Auth automatically extends sessions on activity. The `updateAge` config controls refresh frequency.

**Implementation**:
- Session cookie is HttpOnly, Secure, SameSite=Lax
- Better Auth middleware checks session age on each request
- If session is older than `updateAge`, it updates `expiresAt`
- No explicit refresh token flow needed for web cookies

---

## 7. Deployment Architecture

### Decision: Three Vercel Projects with Subdomain Routing

**Rationale**: Vercel supports multiple projects under one domain with subdomain routing.

**Structure**:
- `example.com` - Docusaurus frontend (book/)
- `auth.example.com` - Better Auth service (auth/)
- `api.example.com` - FastAPI backend (backend/)

**Cookie Domain**: `.example.com` (shared across all subdomains)

**Vercel Configuration** (auth/vercel.json):
```json
{
  "builds": [{ "src": "src/index.ts", "use": "@vercel/node" }],
  "routes": [{ "src": "/(.*)", "dest": "src/index.ts" }]
}
```

---

## 8. Security Considerations

### CORS Configuration

```python
# backend/app/main.py
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://example.com",
        "https://auth.example.com",
    ],
    allow_credentials=True,  # Required for cookies
    allow_methods=["*"],
    allow_headers=["*"],
)
```

---

## 9. Error Handling

### OAuth Errors

| Error | User Message | Recovery |
|-------|--------------|----------|
| access_denied | You cancelled the sign-in. Please try again. | Show retry button |
| invalid_request | Something went wrong. Please try again. | Log error, show retry |
| server_error | Google is temporarily unavailable. Please try later. | Retry with backoff |

### Session Errors

| Error | HTTP Status | Action |
|-------|-------------|--------|
| Missing cookie | 401 | Redirect to sign-in |
| Expired session | 401 | Clear cookie, show sign-in |
| Invalid token | 401 | Clear cookie, show sign-in |
| Database error | 500 | Log error, generic message |

---

## 10. Testing Strategy

### Unit Tests
- Mock Better Auth client for React component tests
- Mock database for FastAPI middleware tests

### Integration Tests
- Test OAuth flow with Google test credentials
- Test session persistence across page reloads
- Test session expiration and cleanup

### E2E Tests
- Full sign-in/sign-out flow with Playwright
- Chat access control verification

---

## Summary

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Auth Library | Better Auth | TypeScript-native, React hooks, PostgreSQL adapter |
| Database | Neon PostgreSQL | Serverless, Vercel-optimized, shared sessions |
| Session Storage | Database sessions | Server-side revocation, 7-day persistence |
| Session Validation | Direct DB query | Lower latency than auth service call |
| Cookie Strategy | Same-domain subdomains | Seamless cross-service auth |
| Token Refresh | Built-in session update | Automatic, no manual refresh flow |
| Deployment | 3 Vercel projects | Separation of concerns, independent scaling |
