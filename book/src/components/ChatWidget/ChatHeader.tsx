/**
 * Chat Header component with title and user menu
 */
import React from "react";
import { UserMenu } from "../Auth/UserMenu";
import { useAuth } from "./hooks/useAuth";
import styles from "./styles.module.css";

interface ChatHeaderProps {
  /** Callback to close the chat panel */
  onClose: () => void;
}

/**
 * Header for the chat panel showing title and user controls
 */
export default function ChatHeader({ onClose }: ChatHeaderProps) {
  const { isAuthenticated } = useAuth();

  return (
    <div className={styles.header}>
      <div className={styles.headerTitle}>
        <span className={styles.headerIcon}>📚</span>
        <span>Book Assistant</span>
      </div>
      <div className={styles.headerActions}>
        {isAuthenticated && <UserMenu />}
        <button
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close chat"
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            width="20"
            height="20"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <line x1="18" y1="6" x2="6" y2="18" />
            <line x1="6" y1="6" x2="18" y2="18" />
          </svg>
        </button>
      </div>
    </div>
  );
}
