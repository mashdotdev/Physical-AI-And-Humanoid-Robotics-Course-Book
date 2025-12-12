/**
 * Authentication hook for ChatWidget
 * Wraps Better Auth client with chat-specific functionality
 */
import { useCallback, useEffect, useState } from "react";
import { authClient } from "@site/src/lib/auth-client";

export interface AuthUser {
  id: string;
  name: string;
  email: string;
  image?: string | null;
}

export interface AuthState {
  user: AuthUser | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  error: string | null;
}

/**
 * Hook for managing authentication state in the chat widget
 */
export function useAuth() {
  const { data: session, isPending, error, refetch } = authClient.useSession();
  const [authError, setAuthError] = useState<string | null>(null);

  // Clear error on successful session load
  useEffect(() => {
    if (session && !error) {
      setAuthError(null);
    }
  }, [session, error]);

  /**
   * Sign in with Google OAuth
   */
  const signInWithGoogle = useCallback(async () => {
    try {
      setAuthError(null);
      await authClient.signIn.social({
        provider: "google",
        callbackURL: window.location.href,
      });
    } catch (err) {
      const message = err instanceof Error ? err.message : "Sign in failed";
      setAuthError(message);
      console.error("Sign in error:", err);
    }
  }, []);

  /**
   * Sign out and clear session
   */
  const signOutUser = useCallback(async () => {
    try {
      setAuthError(null);
      await authClient.signOut();
      // Refetch session to update state
      refetch();
    } catch (err) {
      const message = err instanceof Error ? err.message : "Sign out failed";
      setAuthError(message);
      console.error("Sign out error:", err);
    }
  }, [refetch]);

  /**
   * Refresh session (useful after auth errors)
   */
  const refreshSession = useCallback(async () => {
    try {
      setAuthError(null);
      await refetch();
    } catch (err) {
      console.error("Session refresh error:", err);
    }
  }, [refetch]);

  /**
   * Clear any auth errors
   */
  const clearError = useCallback(() => {
    setAuthError(null);
  }, []);

  // Derive user from session
  const user: AuthUser | null = session?.user
    ? {
        id: session.user.id,
        name: session.user.name,
        email: session.user.email,
        image: session.user.image,
      }
    : null;

  return {
    // State
    user,
    session: session?.session || null,
    isAuthenticated: !!session?.user,
    isLoading: isPending,
    error: authError || (error ? error.message : null),

    // Actions
    signInWithGoogle,
    signOut: signOutUser,
    refreshSession,
    clearError,
  };
}

export default useAuth;
