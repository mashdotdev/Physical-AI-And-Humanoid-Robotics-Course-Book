/**
 * Loading state component for authentication check
 */
import React from "react";
import styles from "./Auth.module.css";

interface AuthLoadingProps {
  /** Optional loading message */
  message?: string;
}

/**
 * Displays a loading spinner while checking authentication status
 */
export function AuthLoading({ message = "Loading..." }: AuthLoadingProps) {
  return (
    <div className={styles.authLoading}>
      <div className={styles.spinner} />
      <span className={styles.loadingText}>{message}</span>
    </div>
  );
}

export default AuthLoading;
