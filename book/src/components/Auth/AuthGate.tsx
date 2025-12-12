/**
 * AuthGate component - wraps content that requires authentication
 */
import React, { ReactNode } from "react";
import { useAuth } from "../ChatWidget/hooks/useAuth";
import { SignInButton } from "./SignInButton";
import { AuthLoading } from "./AuthLoading";
import { AuthError } from "./AuthError";
import styles from "./Auth.module.css";

interface AuthGateProps {
  /** Content to render when authenticated */
  children: ReactNode;
  /** Optional custom sign-in prompt message */
  signInMessage?: string;
  /** Show loading state while checking auth */
  showLoading?: boolean;
}

/**
 * Wrapper that shows sign-in prompt for unauthenticated users
 * and renders children for authenticated users
 */
export function AuthGate({
  children,
  signInMessage = "Sign in to use the AI assistant",
  showLoading = true,
}: AuthGateProps) {
  const { isAuthenticated, isLoading, error, clearError } = useAuth();

  // Show loading state while checking authentication
  if (showLoading && isLoading) {
    return <AuthLoading />;
  }

  // Show error state if there's an auth error
  if (error) {
    return <AuthError message={error} onRetry={clearError} />;
  }

  // Show sign-in prompt for unauthenticated users
  if (!isAuthenticated) {
    return (
      <div className={styles.authGate}>
        <div className={styles.authGateContent}>
          <div className={styles.authGateIcon}>
            <ChatIcon />
          </div>
          <h3 className={styles.authGateTitle}>Welcome!</h3>
          <p className={styles.authGateMessage}>{signInMessage}</p>
          <SignInButton fullWidth />
        </div>
      </div>
    );
  }

  // Render children for authenticated users
  return <>{children}</>;
}

/**
 * Chat bubble icon for the auth gate
 */
function ChatIcon() {
  return (
    <svg
      width="48"
      height="48"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="1.5"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
    </svg>
  );
}

export default AuthGate;
