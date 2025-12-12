/**
 * Better Auth client configuration for the frontend
 */
import { createAuthClient } from "better-auth/react";

// Auth service URL - auto-detect production vs development
const getAuthUrl = () => {
  if (typeof window === "undefined") return "http://localhost:3001";
  if ((window as any).__AUTH_URL__) return (window as any).__AUTH_URL__;
  // Production detection
  if (window.location.hostname.includes("vercel.app") ||
      window.location.hostname.includes("physical-ai-and-humanoid-robotics")) {
    return "https://auth-sandy-delta.vercel.app";
  }
  return "http://localhost:3001";
};

const AUTH_URL = getAuthUrl();

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
