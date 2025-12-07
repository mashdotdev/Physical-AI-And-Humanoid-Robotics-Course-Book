import React, { useRef, useEffect } from 'react';
import styles from './styles.module.css';
import type { Message, Source } from './types';

interface MessageListProps {
  messages: Message[];
  isLoading: boolean;
}

/**
 * Renders a single source reference
 */
function SourceReference({ source }: { source: Source }) {
  return (
    <span className={styles.source}>
      {source.chapter} - {source.section}
    </span>
  );
}

/**
 * Renders a single message bubble
 */
function MessageBubble({ message }: { message: Message }) {
  const isUser = message.role === 'user';

  return (
    <div className={`${styles.message} ${isUser ? styles.user : styles.assistant}`}>
      <div className={styles.messageContent}>
        {message.content}
      </div>
      {message.sources && message.sources.length > 0 && (
        <div className={styles.sources}>
          <span className={styles.sourcesLabel}>Sources:</span>
          {message.sources.map((source, idx) => (
            <SourceReference key={idx} source={source} />
          ))}
        </div>
      )}
    </div>
  );
}

/**
 * Loading indicator for when the bot is thinking
 */
function LoadingIndicator() {
  return (
    <div className={`${styles.message} ${styles.assistant}`}>
      <div className={styles.messageContent}>
        <span className={styles.typingDots}>
          <span>.</span>
          <span>.</span>
          <span>.</span>
        </span>
      </div>
    </div>
  );
}

/**
 * Empty state when no messages exist
 */
function EmptyState() {
  return (
    <div className={styles.emptyState}>
      <p>Ask me anything about the Physical AI textbook!</p>
      <p className={styles.emptyHint}>
        Try: "What is ROS 2?" or "Explain digital twins"
      </p>
    </div>
  );
}

/**
 * Scrollable list of chat messages
 */
export default function MessageList({ messages, isLoading }: MessageListProps) {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  return (
    <div className={styles.messageList}>
      {messages.length === 0 && !isLoading ? (
        <EmptyState />
      ) : (
        <>
          {messages.map((message, idx) => (
            <MessageBubble key={idx} message={message} />
          ))}
          {isLoading && <LoadingIndicator />}
        </>
      )}
      <div ref={messagesEndRef} />
    </div>
  );
}
