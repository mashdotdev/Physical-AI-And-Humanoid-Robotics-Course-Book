/**
 * Error display component for authentication errors
 */
import React from "react";
import styles from "./Auth.module.css";

interface AuthErrorProps {
  /** Error message to display */
  message: string;
  /** Callback to retry/dismiss the error */
  onRetry?: () => void;
  /** Custom retry button text */
  retryText?: string;
}

/**
 * Displays authentication error with retry option
 */
export function AuthError({
  message,
  onRetry,
  retryText = "Try again",
}: AuthErrorProps) {
  // Parse common OAuth errors into user-friendly messages
  const displayMessage = parseErrorMessage(message);

  return (
    <div className={styles.authError}>
      <div className={styles.authErrorIcon}>
        <ErrorIcon />
      </div>
      <h3 className={styles.authErrorTitle}>Authentication Error</h3>
      <p className={styles.authErrorMessage}>{displayMessage}</p>
      {onRetry && (
        <button
          className={styles.retryButton}
          onClick={onRetry}
          type="button"
        >
          {retryText}
        </button>
      )}
    </div>
  );
}

/**
 * Parse error messages into user-friendly text
 */
function parseErrorMessage(error: string): string {
  const lowered = error.toLowerCase();

  if (lowered.includes("access_denied")) {
    return "You cancelled the sign-in. Please try again when ready.";
  }

  if (lowered.includes("invalid_request")) {
    return "Something went wrong with the sign-in request. Please try again.";
  }

  if (lowered.includes("server_error") || lowered.includes("temporarily")) {
    return "Google is temporarily unavailable. Please try again later.";
  }

  if (lowered.includes("network") || lowered.includes("fetch")) {
    return "Unable to connect. Please check your internet connection.";
  }

  if (lowered.includes("session_expired") || lowered.includes("expired")) {
    return "Your session has expired. Please sign in again.";
  }

  // Default to the original message
  return error;
}

/**
 * Error icon (exclamation in circle)
 */
function ErrorIcon() {
  return (
    <svg
      width="32"
      height="32"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <circle cx="12" cy="12" r="10" />
      <line x1="12" y1="8" x2="12" y2="12" />
      <line x1="12" y1="16" x2="12.01" y2="16" />
    </svg>
  );
}

export default AuthError;
