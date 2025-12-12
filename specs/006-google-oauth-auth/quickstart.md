# Quickstart: Google OAuth Authentication

**Feature**: 006-google-oauth-auth | **Date**: 2025-12-12

## Prerequisites

- Node.js 20+
- Python 3.11+
- Google Cloud Console account
- Neon PostgreSQL account
- Vercel account (for deployment)

---

## 1. Set Up Google OAuth Credentials

### Google Cloud Console

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select existing
3. Navigate to **APIs & Services > Credentials**
4. Click **Create Credentials > OAuth client ID**
5. Select **Web application**
6. Configure:
   - **Name**: Physical AI Book Auth
   - **Authorized JavaScript origins**:
     - `http://localhost:3000` (dev)
     - `https://auth.yourdomain.com` (prod)
   - **Authorized redirect URIs**:
     - `http://localhost:3000/api/auth/callback/google` (dev)
     - `https://auth.yourdomain.com/api/auth/callback/google` (prod)
7. Copy **Client ID** and **Client Secret**

---

## 2. Set Up Neon Database

### Create Database

1. Go to [Neon Console](https://console.neon.tech/)
2. Create a new project
3. Copy the connection string:
   ```
   postgres://user:password@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
   ```

### Run Migrations

Better Auth will create tables automatically on first run, or manually:

```bash
cd auth
npx @better-auth/cli migrate
```

---

## 3. Set Up Auth Service

### Create Project

```bash
mkdir auth && cd auth
npm init -y
npm install better-auth pg dotenv
npm install -D typescript @types/node tsx
```

### Configure TypeScript

```json
// auth/tsconfig.json
{
  "compilerOptions": {
    "target": "ES2022",
    "module": "ESNext",
    "moduleResolution": "bundler",
    "strict": true,
    "esModuleInterop": true,
    "outDir": "dist"
  },
  "include": ["src"]
}
```

### Create Auth Configuration

```typescript
// auth/src/auth.ts
import { betterAuth } from "better-auth";
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL,
  }),
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID!,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET!,
      prompt: "select_account",
    },
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // 1 day
  },
  trustedOrigins: [
    "http://localhost:5173", // Docusaurus dev
    "https://yourdomain.com",
  ],
});
```

### Create Server Entry

```typescript
// auth/src/index.ts
import { toNodeHandler } from "better-auth/node";
import { createServer } from "http";
import { auth } from "./auth";

const server = createServer(toNodeHandler(auth));

server.listen(3000, () => {
  console.log("Auth server running on http://localhost:3000");
});
```

### Environment Variables

```env
# auth/.env
DATABASE_URL=postgres://user:password@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
GOOGLE_CLIENT_ID=xxx.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=GOCSPX-xxx
BETTER_AUTH_SECRET=your-32-character-secret-key
BETTER_AUTH_URL=http://localhost:3000
```

### Run Auth Service

```bash
npx tsx src/index.ts
```

---

## 4. Update Backend (FastAPI)

### Install Dependencies

```bash
cd backend
pip install asyncpg python-jose[cryptography]
```

Update `requirements.txt`:
```
asyncpg>=0.29.0
python-jose[cryptography]>=3.3.0
```

### Add Database Connection

```python
# backend/app/db/session.py
import asyncpg
from app.config import settings

pool: asyncpg.Pool = None

async def get_db_pool():
    global pool
    if pool is None:
        pool = await asyncpg.create_pool(
            dsn=settings.DATABASE_URL,
            min_size=1,
            max_size=10,
        )
    return pool

async def close_db_pool():
    global pool
    if pool:
        await pool.close()
```

### Add Auth Middleware

```python
# backend/app/auth/dependencies.py
from fastapi import Depends, HTTPException, Request
from app.db.session import get_db_pool

async def get_current_user(request: Request):
    token = request.cookies.get("better-auth.session_token")
    if not token:
        raise HTTPException(status_code=401, detail="Not authenticated")

    pool = await get_db_pool()
    session = await pool.fetchrow(
        '''
        SELECT u.id, u.name, u.email, u.image, s.id as session_id
        FROM session s
        JOIN "user" u ON s."userId" = u.id
        WHERE s.token = $1 AND s."expiresAt" > NOW()
        ''',
        token
    )

    if not session:
        raise HTTPException(status_code=401, detail="Session expired")

    return dict(session)
```

### Protect Chat Endpoint

```python
# backend/app/api/chat.py
from fastapi import Depends
from app.auth.dependencies import get_current_user

@router.post("/chat")
async def chat(
    request: ChatRequest,
    current_user: dict = Depends(get_current_user)
):
    # User is authenticated
    # current_user contains: id, name, email, image, session_id
    return await process_chat(request, current_user)
```

### Update Config

```python
# backend/app/config.py
class Settings(BaseSettings):
    # ... existing settings ...
    DATABASE_URL: str  # Add Neon connection string

    class Config:
        env_file = ".env"
```

### Update CORS

```python
# backend/app/main.py
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:5173",
        "https://yourdomain.com",
        "https://auth.yourdomain.com",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

---

## 5. Update Frontend (Docusaurus)

### Install Better Auth Client

```bash
cd book
npm install better-auth
```

### Create Auth Client

```typescript
// book/src/lib/auth-client.ts
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: import.meta.env.VITE_AUTH_URL || "http://localhost:3000",
});
```

### Create useAuth Hook

```tsx
// book/src/components/ChatWidget/hooks/useAuth.ts
import { authClient } from "@/lib/auth-client";

export function useAuth() {
  const { data: session, isPending, error } = authClient.useSession();

  const signIn = async () => {
    await authClient.signIn.social({ provider: "google" });
  };

  const signOut = async () => {
    await authClient.signOut();
  };

  return {
    user: session?.user,
    isAuthenticated: !!session,
    isLoading: isPending,
    error,
    signIn,
    signOut,
  };
}
```

### Add Sign-In Button

```tsx
// book/src/components/Auth/SignInButton.tsx
import { useAuth } from "../ChatWidget/hooks/useAuth";

export function SignInButton() {
  const { signIn, isLoading } = useAuth();

  return (
    <button onClick={signIn} disabled={isLoading}>
      Sign in with Google
    </button>
  );
}
```

### Update Chat Widget

```tsx
// book/src/components/ChatWidget/index.tsx
import { useAuth } from "./hooks/useAuth";
import { SignInButton } from "../Auth/SignInButton";

export function ChatWidget() {
  const { isAuthenticated, isLoading, user } = useAuth();

  if (isLoading) {
    return <div>Loading...</div>;
  }

  if (!isAuthenticated) {
    return (
      <div className="chat-signin">
        <p>Sign in to use the AI assistant</p>
        <SignInButton />
      </div>
    );
  }

  return (
    <div className="chat-widget">
      <ChatHeader user={user} />
      <MessageList />
      <InputArea />
    </div>
  );
}
```

---

## 6. Local Development

### Start All Services

Terminal 1 - Auth Service:
```bash
cd auth && npx tsx src/index.ts
```

Terminal 2 - Backend:
```bash
cd backend && uvicorn app.main:app --reload --port 8000
```

Terminal 3 - Frontend:
```bash
cd book && npm run start
```

### Test Flow

1. Open http://localhost:5173
2. Click chat widget
3. Click "Sign in with Google"
4. Complete Google OAuth flow
5. Verify session cookie is set
6. Send a chat message
7. Verify response is received

---

## 7. Deployment (Vercel)

### Auth Service

```json
// auth/vercel.json
{
  "builds": [{ "src": "src/index.ts", "use": "@vercel/node" }],
  "routes": [{ "src": "/(.*)", "dest": "src/index.ts" }]
}
```

Deploy:
```bash
cd auth && vercel --prod
```

### Backend

Already configured in `backend/vercel.json`.

Deploy:
```bash
cd backend && vercel --prod
```

### Frontend

```bash
cd book && npm run build && vercel --prod
```

### Configure Domains

In Vercel dashboard:
- Auth: `auth.yourdomain.com`
- API: `api.yourdomain.com`
- Frontend: `yourdomain.com`

### Update Environment Variables

Set in Vercel dashboard for each project:
- `DATABASE_URL`
- `GOOGLE_CLIENT_ID`
- `GOOGLE_CLIENT_SECRET`
- `BETTER_AUTH_SECRET`
- `BETTER_AUTH_URL`

---

## Troubleshooting

### Cookie Not Set

- Verify CORS `allow_credentials: true`
- Verify same-domain deployment
- Check browser developer tools > Application > Cookies

### Session Not Found

- Verify `DATABASE_URL` is correct
- Check session table in Neon
- Verify cookie name matches (`better-auth.session_token`)

### OAuth Callback Error

- Verify redirect URI matches exactly in Google Console
- Check auth service logs for error details
- Verify `GOOGLE_CLIENT_SECRET` is correct

### 401 on Chat API

- Verify session cookie is being sent (check Network tab)
- Verify backend can connect to Neon
- Check session expiration in database
