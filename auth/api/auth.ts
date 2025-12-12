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
const trustedOrigins = process.env.TRUSTED_ORIGINS
  ? process.env.TRUSTED_ORIGINS.split(",").map((origin) => origin.trim())
  : [
      "http://localhost:5173",
      "http://localhost:3000",
      "https://physical-ai-and-humanoid-robotics-c-lemon.vercel.app",
    ];

// Better Auth configuration
const auth = betterAuth({
  database: pool,
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3001",
  secret: process.env.BETTER_AUTH_SECRET,
  trustedOrigins,

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
    cookieCache: {
      enabled: true,
      maxAge: 60 * 5, // 5 minutes
    },
  },

  advanced: {
    useSecureCookies: process.env.NODE_ENV === "production",
    cookieDomain: process.env.COOKIE_DOMAIN || undefined,
  },
});

const handler = toNodeHandler(auth);

export default async function (req: IncomingMessage, res: ServerResponse) {
  // Handle CORS preflight
  const origin = req.headers.origin || "*";
  res.setHeader("Access-Control-Allow-Origin", origin);
  res.setHeader("Access-Control-Allow-Credentials", "true");

  if (req.method === "OPTIONS") {
    res.setHeader(
      "Access-Control-Allow-Methods",
      "GET, POST, PUT, DELETE, OPTIONS"
    );
    res.setHeader(
      "Access-Control-Allow-Headers",
      "Content-Type, Authorization"
    );
    res.setHeader("Access-Control-Max-Age", "86400");
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

  return handler(req, res);
}
