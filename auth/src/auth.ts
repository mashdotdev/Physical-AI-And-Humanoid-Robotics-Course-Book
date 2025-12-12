/**
 * Better Auth server configuration with Google OAuth provider
 */
import { betterAuth } from "better-auth";
import { Pool } from "pg";
import "dotenv/config";

// Create database pool for Neon PostgreSQL
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: process.env.NODE_ENV === "production" ? { rejectUnauthorized: false } : undefined,
});

// Parse trusted origins from environment
const trustedOrigins = process.env.TRUSTED_ORIGINS
  ? process.env.TRUSTED_ORIGINS.split(",").map((origin) => origin.trim())
  : ["http://localhost:5173", "http://localhost:3001"];

export const auth = betterAuth({
  database: pool,
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3000",
  secret: process.env.BETTER_AUTH_SECRET,
  trustedOrigins,

  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID as string,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET as string,
      // Request offline access for refresh tokens
      accessType: "offline",
      // Always show account picker for multi-account support
      prompt: "select_account",
    },
  },

  session: {
    // Session expires after 7 days (in seconds)
    expiresIn: 60 * 60 * 24 * 7,
    // Update session timestamp every day to extend session on activity
    updateAge: 60 * 60 * 24,
    // Cookie configuration for cross-subdomain support
    cookieCache: {
      enabled: true,
      maxAge: 60 * 5, // 5 minutes cache
    },
  },

  // Advanced options for production
  advanced: {
    // Use secure cookies in production
    useSecureCookies: process.env.NODE_ENV === "production",
    // Cookie domain for subdomain sharing (set in production)
    cookieDomain: process.env.COOKIE_DOMAIN || undefined,
  },
});

export type Auth = typeof auth;
