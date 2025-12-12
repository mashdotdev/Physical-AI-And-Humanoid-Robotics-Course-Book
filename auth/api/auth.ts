/**
 * Vercel serverless function for Better Auth
 * All auth routes are handled by this single endpoint
 */
import { betterAuth } from "better-auth";
import { toNodeHandler } from "better-auth/node";
import { Pool } from "pg";
import type { IncomingMessage, ServerResponse } from "http";

// Create database pool for Neon PostgreSQL
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl:
    process.env.NODE_ENV === "production"
      ? { rejectUnauthorized: false }
      : undefined,
});

// Parse trusted origins from environment
// "Allow all" approach: We will reflect the request origin in the CORS handler below.
// For Better Auth config, we provide the env var list or defaults.
const trustedOrigins = process.env.TRUSTED_ORIGINS
  ? process.env.TRUSTED_ORIGINS.split(",").map((origin) => origin.trim())
  : [
      "http://localhost:5173",
      "http://localhost:3000",
      "https://auth-sandy-delta.vercel.app",
      "https://physical-ai-and-humanoid-robotics-c-lemon.vercel.app"
    ];

// Sanitize BETTER_AUTH_URL to prevent issues with whitespace or trailing slashes
if (process.env.BETTER_AUTH_URL) {
  process.env.BETTER_AUTH_URL = process.env.BETTER_AUTH_URL.replace(/\s+/g, "").replace(/\/$/, "");
}

// Check for missing environment variables
if (process.env.NODE_ENV === "production") {
  // Log critical vars but don't block
  if (!process.env.GOOGLE_CLIENT_ID || !process.env.BETTER_AUTH_SECRET) {
     console.error("[Auth Warning] Missing critical env vars");
  }
}

// Better Auth configuration
const auth = betterAuth({
  database: pool,
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3001",
  secret: process.env.BETTER_AUTH_SECRET,
  trustedOrigins, // This list is used for internal checks

  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID as string,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET as string,
      accessType: "offline",
      prompt: "select_account",
    },
  },

  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // 1 day
  },

  advanced: {
    // Relax cookie constraints to help with cross-domain issues
    useSecureCookies: process.env.NODE_ENV === "production",
    cookieDomain: process.env.COOKIE_DOMAIN || undefined,
  },
});

const handler = toNodeHandler(auth);

export default async function (req: IncomingMessage, res: ServerResponse) {
  // Allow All Origins (Reflected)
  // We explicitly set the Allowed Origin to the requester's Origin.
  // This allows any domain to connect while still allowing credentials (cookies).
  const origin = req.headers.origin || "*";
  res.setHeader("Access-Control-Allow-Origin", origin);
  res.setHeader("Access-Control-Allow-Credentials", "true");
  res.setHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
  res.setHeader("Access-Control-Allow-Headers", "Content-Type, Authorization, Cookie, X-Requested-With");

  // Handle CORS preflight
  if (req.method === "OPTIONS") {
    res.statusCode = 204;
    res.end();
    return;
  }

  // Health check
  if (req.url === "/health") {
    res.setHeader("Content-Type", "application/json");
    res.statusCode = 200;
    res.end(JSON.stringify({ status: "healthy" }));
    return;
  }

  try {
    // Debug logging
    if (process.env.NODE_ENV === "production" && (req.url?.includes("callback") || req.url?.includes("sign-in"))) {
       console.log(`[Auth Debug] ${req.method} ${req.url} | Origin: ${origin}`);
    }
    return await handler(req, res);
  } catch (error) {
    console.error("[Auth Critical Error] Unhandled exception:", error);
    if (!res.headersSent) {
      res.statusCode = 500;
      res.setHeader("Content-Type", "application/json");
      res.end(JSON.stringify({ error: "Internal Server Error", details: String(error) }));
    }
  }
}
