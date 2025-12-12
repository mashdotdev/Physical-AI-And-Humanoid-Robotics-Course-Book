/**
 * Sign Out Button component
 */
import React from "react";
import { useAuth } from "../ChatWidget/hooks/useAuth";
import styles from "./Auth.module.css";

interface SignOutButtonProps {
  /** Optional callback after sign-out completes */
  onSignOut?: () => void;
  /** Custom button text */
  text?: string;
  /** Variant style */
  variant?: "default" | "text" | "danger";
}

/**
 * Button that triggers sign-out and terminates the session
 */
export function SignOutButton({
  onSignOut,
  text = "Sign out",
  variant = "default",
}: SignOutButtonProps) {
  const { signOut, isLoading } = useAuth();

  const handleClick = async () => {
    await signOut();
    onSignOut?.();
  };

  const variantClass = variant === "text" ? styles.textButton :
                       variant === "danger" ? styles.dangerButton :
                       styles.signOutButton;

  return (
    <button
      className={variantClass}
      onClick={handleClick}
      disabled={isLoading}
      type="button"
    >
      {isLoading ? "Signing out..." : text}
    </button>
  );
}

export default SignOutButton;
