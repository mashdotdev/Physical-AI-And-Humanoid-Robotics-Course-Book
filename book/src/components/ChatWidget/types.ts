/**
 * TypeScript types for the Chat Widget
 */

export type Role = 'user' | 'assistant';

export interface Source {
  chapter: string;
  section: string;
  filePath: string;
  relevance: number;
}

export interface Message {
  role: Role;
  content: string;
  sources?: Source[];
  timestamp: Date;
}

export interface Conversation {
  id: string;
  messages: Message[];
  createdAt: Date;
}

export interface ChatRequest {
  message: string;
  conversationId?: string;
}

export interface ChatResponse {
  response: string;
  sources: Source[];
  conversationId: string;
}

export interface ErrorResponse {
  error: string;
  message: string;
}

export interface ChatState {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  conversationId: string | null;
}
