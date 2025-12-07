import React from 'react';
import styles from './styles.module.css';
import MessageList from './MessageList';
import InputArea from './InputArea';
import type { Message } from './types';

interface ChatPanelProps {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  onSend: (message: string) => void;
  onClose: () => void;
}

/**
 * The main chat panel containing header, messages, and input
 */
export default function ChatPanel({
  messages,
  isLoading,
  error,
  onSend,
  onClose,
}: ChatPanelProps) {
  return (
    <div className={styles.panel}>
      {/* Header */}
      <div className={styles.header}>
        <div className={styles.headerTitle}>
          <span className={styles.headerIcon}>📚</span>
          <span>Book Assistant</span>
        </div>
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

      {/* Error banner */}
      {error && (
        <div className={styles.errorBanner}>
          {error}
        </div>
      )}

      {/* Messages */}
      <MessageList messages={messages} isLoading={isLoading} />

      {/* Input */}
      <InputArea onSend={onSend} isLoading={isLoading} />
    </div>
  );
}
