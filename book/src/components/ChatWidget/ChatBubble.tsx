import React from 'react';
import styles from './styles.module.css';

interface ChatBubbleProps {
  onClick: () => void;
}

/**
 * Floating chat bubble button that opens the chat panel
 */
export default function ChatBubble({ onClick }: ChatBubbleProps) {
  return (
    <button
      className={styles.bubble}
      onClick={onClick}
      aria-label="Open chat"
      title="Ask about the book"
    >
      <svg
        xmlns="http://www.w3.org/2000/svg"
        width="24"
        height="24"
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      >
        <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
      </svg>
    </button>
  );
}
