/**
 * Better Auth client configuration for the frontend
 */
import { createAuthClient } from "better-auth/react";

// Auth service URL - defaults to localhost for development
// In production, set window.__AUTH_URL__ before loading the app
const AUTH_URL = typeof window !== "undefined" && (window as any).__AUTH_URL__
  ? (window as any).__AUTH_URL__
  : "http://localhost:3001";

/**
 * Better Auth client instance
 * Provides hooks and methods for authentication
 */
export const authClient = createAuthClient({
  baseURL: AUTH_URL,
});

// Export commonly used methods and hooks
export const {
  useSession,
  signIn,
  signOut,
  getSession,
} = authClient;

// Export types
export type Session = typeof authClient extends { useSession: () => { data: infer T } } ? T : never;
