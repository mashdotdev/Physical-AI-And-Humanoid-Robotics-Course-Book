# Data Model: Google OAuth Authentication

**Feature**: 006-google-oauth-auth | **Date**: 2025-12-12

## Overview

This document defines the database schema for user authentication. Better Auth manages these tables automatically via its CLI migration tool.

---

## Entity Relationship Diagram

```
┌─────────────┐       ┌─────────────┐       ┌─────────────┐
│    user     │───────│   session   │       │   account   │
├─────────────┤  1:N  ├─────────────┤       ├─────────────┤
│ id (PK)     │       │ id (PK)     │       │ id (PK)     │
│ name        │       │ userId (FK) │       │ userId (FK) │
│ email       │       │ token       │       │ provider    │
│ image       │       │ expiresAt   │       │ providerAcct│
│ emailVerif. │       │ ipAddress   │       │ accessToken │
│ createdAt   │       │ userAgent   │       │ refreshToken│
│ updatedAt   │       │ createdAt   │       │ createdAt   │
└─────────────┘       │ updatedAt   │       │ updatedAt   │
       │              └─────────────┘       └─────────────┘
       │                                           │
       └───────────────────────────────────────────┘
                           1:N
```

---

## Tables

### 1. user

Stores authenticated user profiles from Google OAuth.

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | VARCHAR(36) | PK | UUID identifier |
| name | VARCHAR(255) | NOT NULL | Display name from Google |
| email | VARCHAR(255) | UNIQUE, NOT NULL | Email address from Google |
| emailVerified | BOOLEAN | DEFAULT false | Whether email is verified |
| image | TEXT | NULL | Profile image URL from Google |
| createdAt | TIMESTAMP | NOT NULL | Account creation timestamp |
| updatedAt | TIMESTAMP | NOT NULL | Last update timestamp |

**Indexes**:
- `idx_user_email` on `email` (UNIQUE)

**Validation Rules**:
- `email` must be valid email format
- `name` must be 1-255 characters

---

### 2. session

Stores active user sessions for cookie-based authentication.

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | VARCHAR(36) | PK | UUID identifier |
| userId | VARCHAR(36) | FK -> user.id, NOT NULL | Associated user |
| token | VARCHAR(255) | UNIQUE, NOT NULL | Session token (stored in cookie) |
| expiresAt | TIMESTAMP | NOT NULL | Session expiration time |
| ipAddress | VARCHAR(45) | NULL | Client IP address |
| userAgent | TEXT | NULL | Client user agent string |
| createdAt | TIMESTAMP | NOT NULL | Session creation timestamp |
| updatedAt | TIMESTAMP | NOT NULL | Last activity timestamp |

**Indexes**:
- `idx_session_token` on `token` (UNIQUE)
- `idx_session_userId` on `userId`
- `idx_session_expiresAt` on `expiresAt`

**Lifecycle**:
- Created on successful OAuth callback
- Updated on each authenticated request (if older than `updateAge`)
- Deleted on sign-out or expiration cleanup

**TTL**: 7 days from creation/last update

---

### 3. account

Stores OAuth provider account links (supports future multi-provider).

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | VARCHAR(36) | PK | UUID identifier |
| userId | VARCHAR(36) | FK -> user.id, NOT NULL | Associated user |
| provider | VARCHAR(50) | NOT NULL | OAuth provider name ("google") |
| providerAccountId | VARCHAR(255) | NOT NULL | Google user ID |
| accessToken | TEXT | NULL | OAuth access token |
| refreshToken | TEXT | NULL | OAuth refresh token |
| accessTokenExpiresAt | TIMESTAMP | NULL | Access token expiration |
| scope | TEXT | NULL | Granted OAuth scopes |
| idToken | TEXT | NULL | Google ID token |
| createdAt | TIMESTAMP | NOT NULL | Account link creation |
| updatedAt | TIMESTAMP | NOT NULL | Last update timestamp |

**Indexes**:
- `idx_account_userId` on `userId`
- `idx_account_provider_providerAccountId` on `(provider, providerAccountId)` (UNIQUE)

**Validation Rules**:
- `provider` must be "google" (only provider in scope)
- `providerAccountId` must be non-empty

---

### 4. verification (Optional - for email verification flows)

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | VARCHAR(36) | PK | UUID identifier |
| identifier | VARCHAR(255) | NOT NULL | Email or phone to verify |
| value | VARCHAR(255) | NOT NULL | Verification token |
| expiresAt | TIMESTAMP | NOT NULL | Token expiration |
| createdAt | TIMESTAMP | NOT NULL | Creation timestamp |
| updatedAt | TIMESTAMP | NOT NULL | Update timestamp |

**Note**: Not actively used for Google OAuth (Google handles email verification), but created by Better Auth for potential future email/password flows.

---

## SQL Schema (PostgreSQL)

```sql
-- User table
CREATE TABLE "user" (
    id VARCHAR(36) PRIMARY KEY,
    name VARCHAR(255) NOT NULL,
    email VARCHAR(255) UNIQUE NOT NULL,
    "emailVerified" BOOLEAN DEFAULT false,
    image TEXT,
    "createdAt" TIMESTAMP NOT NULL DEFAULT NOW(),
    "updatedAt" TIMESTAMP NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_user_email ON "user"(email);

-- Session table
CREATE TABLE session (
    id VARCHAR(36) PRIMARY KEY,
    "userId" VARCHAR(36) NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    token VARCHAR(255) UNIQUE NOT NULL,
    "expiresAt" TIMESTAMP NOT NULL,
    "ipAddress" VARCHAR(45),
    "userAgent" TEXT,
    "createdAt" TIMESTAMP NOT NULL DEFAULT NOW(),
    "updatedAt" TIMESTAMP NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_session_token ON session(token);
CREATE INDEX idx_session_userId ON session("userId");
CREATE INDEX idx_session_expiresAt ON session("expiresAt");

-- Account table
CREATE TABLE account (
    id VARCHAR(36) PRIMARY KEY,
    "userId" VARCHAR(36) NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    provider VARCHAR(50) NOT NULL,
    "providerAccountId" VARCHAR(255) NOT NULL,
    "accessToken" TEXT,
    "refreshToken" TEXT,
    "accessTokenExpiresAt" TIMESTAMP,
    scope TEXT,
    "idToken" TEXT,
    "createdAt" TIMESTAMP NOT NULL DEFAULT NOW(),
    "updatedAt" TIMESTAMP NOT NULL DEFAULT NOW(),
    UNIQUE(provider, "providerAccountId")
);

CREATE INDEX idx_account_userId ON account("userId");

-- Verification table (optional)
CREATE TABLE verification (
    id VARCHAR(36) PRIMARY KEY,
    identifier VARCHAR(255) NOT NULL,
    value VARCHAR(255) NOT NULL,
    "expiresAt" TIMESTAMP NOT NULL,
    "createdAt" TIMESTAMP NOT NULL DEFAULT NOW(),
    "updatedAt" TIMESTAMP NOT NULL DEFAULT NOW()
);
```

---

## TypeScript Types

```typescript
// Generated from Better Auth schema
interface User {
    id: string;
    name: string;
    email: string;
    emailVerified: boolean;
    image: string | null;
    createdAt: Date;
    updatedAt: Date;
}

interface Session {
    id: string;
    userId: string;
    token: string;
    expiresAt: Date;
    ipAddress: string | null;
    userAgent: string | null;
    createdAt: Date;
    updatedAt: Date;
}

interface Account {
    id: string;
    userId: string;
    provider: "google";
    providerAccountId: string;
    accessToken: string | null;
    refreshToken: string | null;
    accessTokenExpiresAt: Date | null;
    scope: string | null;
    idToken: string | null;
    createdAt: Date;
    updatedAt: Date;
}

// Session with user data (for API responses)
interface SessionWithUser {
    session: Session;
    user: User;
}
```

---

## Python Pydantic Models

```python
# backend/app/models/auth.py
from datetime import datetime
from pydantic import BaseModel, EmailStr
from typing import Optional

class User(BaseModel):
    id: str
    name: str
    email: EmailStr
    email_verified: bool
    image: Optional[str] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class Session(BaseModel):
    id: str
    user_id: str
    token: str
    expires_at: datetime
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class CurrentUser(BaseModel):
    """Authenticated user context for request handlers"""
    id: str
    name: str
    email: EmailStr
    image: Optional[str] = None
    session_id: str
```

---

## State Transitions

### User Lifecycle

```
[New OAuth Login] → Created
     ↓
[Active] ←→ [Profile Updated]
     ↓
[Account Deleted] → Removed (CASCADE deletes sessions/accounts)
```

### Session Lifecycle

```
[OAuth Callback Success] → Created (expiresAt = now + 7 days)
     ↓
[Request with valid token] → Updated (if age > updateAge)
     ↓
[Sign Out] → Deleted
     |
     ↓
[Expiration] → Deleted (by cleanup job)
```

---

## Migration Strategy

### Initial Setup

```bash
# Generate schema from Better Auth config
npx @better-auth/cli generate

# Apply to Neon database
npx @better-auth/cli migrate
```

### Schema Updates

Better Auth handles migrations automatically when config changes. Run:

```bash
npx @better-auth/cli migrate
```

---

## Data Retention

| Data | Retention | Cleanup |
|------|-----------|---------|
| User | Indefinite | Manual deletion only |
| Session | 7 days after expiration | Automatic cleanup job |
| Account | While user exists | CASCADE delete |
| Verification | 24 hours | Automatic cleanup |

---

## Security Considerations

1. **Session tokens**: Cryptographically random, 256-bit minimum
2. **Access tokens**: Encrypted at rest (Better Auth default)
3. **Refresh tokens**: Stored encrypted, rotated on use
4. **PII (email, name)**: Stored in Neon with encryption at rest
5. **IP addresses**: Stored for security audit, consider GDPR retention policy
