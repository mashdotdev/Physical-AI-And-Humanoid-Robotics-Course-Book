/**
 * Better Auth client configuration for the frontend
 */
import { createAuthClient } from "better-auth/react";

// Auth service URL - use local proxy in production to avoid cross-domain cookie issues
const getAuthUrl = () => {
  if (typeof window === "undefined") return "http://localhost:3001";
  if ((window as any).__AUTH_URL__) return (window as any).__AUTH_URL__;
  
  // In production, use the current origin (proxied via vercel.json)
  if (window.location.hostname.includes("vercel.app") ||
      window.location.hostname.includes("physical-ai-and-humanoid-robotics")) {
    return window.location.origin;
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
