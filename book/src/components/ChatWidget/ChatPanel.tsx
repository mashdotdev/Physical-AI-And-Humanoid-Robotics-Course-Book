import React from 'react';
import styles from './styles.module.css';
import ChatHeader from './ChatHeader';
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
      {/* Header with user menu */}
      <ChatHeader onClose={onClose} />

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
