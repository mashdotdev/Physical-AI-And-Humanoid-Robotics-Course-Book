import { useState, useCallback } from 'react';
import type { Message, ChatState } from '../types';
import { sendMessage as sendMessageApi } from '../services/chatApi';

const STORAGE_KEY = 'chat_conversation';

/**
 * Load conversation from sessionStorage
 */
function loadConversation(): { messages: Message[]; conversationId: string | null } {
  try {
    const stored = sessionStorage.getItem(STORAGE_KEY);
    if (stored) {
      const data = JSON.parse(stored);
      return {
        messages: data.messages.map((m: any) => ({
          ...m,
          timestamp: new Date(m.timestamp),
        })),
        conversationId: data.conversationId,
      };
    }
  } catch (e) {
    console.error('Failed to load conversation:', e);
  }
  return { messages: [], conversationId: null };
}

/**
 * Save conversation to sessionStorage
 */
function saveConversation(messages: Message[], conversationId: string | null) {
  try {
    sessionStorage.setItem(
      STORAGE_KEY,
      JSON.stringify({ messages, conversationId })
    );
  } catch (e) {
    console.error('Failed to save conversation:', e);
  }
}

/**
 * Hook for managing chat state and sending messages
 */
export function useChat() {
  const [state, setState] = useState<ChatState>(() => {
    const { messages, conversationId } = loadConversation();
    return {
      messages,
      isLoading: false,
      error: null,
      conversationId,
    };
  });

  const sendMessage = useCallback(async (content: string) => {
    // Add user message immediately
    const userMessage: Message = {
      role: 'user',
      content,
      timestamp: new Date(),
    };

    setState((prev) => ({
      ...prev,
      messages: [...prev.messages, userMessage],
      isLoading: true,
      error: null,
    }));

    try {
      // Call the API
      const response = await sendMessageApi(content, state.conversationId || undefined);

      // Add assistant message
      const assistantMessage: Message = {
        role: 'assistant',
        content: response.response,
        sources: response.sources,
        timestamp: new Date(),
      };

      setState((prev) => {
        const newMessages = [...prev.messages, assistantMessage];
        const newConversationId = response.conversationId;

        // Persist to sessionStorage
        saveConversation(newMessages, newConversationId);

        return {
          ...prev,
          messages: newMessages,
          isLoading: false,
          conversationId: newConversationId,
        };
      });
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Something went wrong';

      setState((prev) => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
      }));
    }
  }, [state.conversationId]);

  const clearError = useCallback(() => {
    setState((prev) => ({ ...prev, error: null }));
  }, []);

  const clearConversation = useCallback(() => {
    sessionStorage.removeItem(STORAGE_KEY);
    setState({
      messages: [],
      isLoading: false,
      error: null,
      conversationId: null,
    });
  }, []);

  return {
    messages: state.messages,
    isLoading: state.isLoading,
    error: state.error,
    conversationId: state.conversationId,
    sendMessage,
    clearError,
    clearConversation,
  };
}
