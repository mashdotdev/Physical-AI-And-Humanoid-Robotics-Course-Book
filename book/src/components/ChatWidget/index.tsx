import React, { useState } from 'react';
import ChatBubble from './ChatBubble';
import ChatPanel from './ChatPanel';
import { useChat } from './hooks/useChat';
import { AuthGate } from '../Auth/AuthGate';
import styles from './styles.module.css';

/**
 * Main ChatWidget component that orchestrates the chat experience
 *
 * Renders either:
 * - A floating bubble button (when minimized)
 * - A full chat panel with auth gate (when expanded)
 *
 * The chat panel requires authentication before users can send messages.
 */
export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const { messages, isLoading, error, sendMessage, clearError } = useChat();

  const handleOpen = () => {
    setIsOpen(true);
    clearError();
  };

  const handleClose = () => {
    setIsOpen(false);
  };

  return (
    <div className={styles.container}>
      {isOpen ? (
        <div className={styles.panelWrapper}>
          <AuthGate signInMessage="Sign in to chat with the AI assistant">
            <ChatPanel
              messages={messages}
              isLoading={isLoading}
              error={error}
              onSend={sendMessage}
              onClose={handleClose}
            />
          </AuthGate>
        </div>
      ) : (
        <ChatBubble onClick={handleOpen} />
      )}
    </div>
  );
}
