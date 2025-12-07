import React, { useState } from 'react';
import styles from './styles.module.css';

interface InputAreaProps {
  onSend: (message: string) => void;
  isLoading: boolean;
  disabled?: boolean;
}

/**
 * Input area for typing and sending messages
 */
export default function InputArea({ onSend, isLoading, disabled }: InputAreaProps) {
  const [input, setInput] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    const trimmed = input.trim();
    if (trimmed && !isLoading && !disabled) {
      onSend(trimmed);
      setInput('');
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  const isDisabled = isLoading || disabled || !input.trim();

  return (
    <form className={styles.inputArea} onSubmit={handleSubmit}>
      <input
        type="text"
        value={input}
        onChange={(e) => setInput(e.target.value)}
        onKeyDown={handleKeyDown}
        placeholder={isLoading ? 'Thinking...' : 'Ask about the book...'}
        disabled={isLoading || disabled}
        maxLength={1000}
        aria-label="Type your question"
      />
      <button
        type="submit"
        disabled={isDisabled}
        aria-label="Send message"
      >
        {isLoading ? (
          <span className={styles.spinner} />
        ) : (
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
            <line x1="22" y1="2" x2="11" y2="13" />
            <polygon points="22 2 15 22 11 13 2 9 22 2" />
          </svg>
        )}
      </button>
    </form>
  );
}
