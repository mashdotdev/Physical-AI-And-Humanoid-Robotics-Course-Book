/**
 * API client service for the chat backend
 */

import type { ChatRequest, ChatResponse, Source } from "../types";

// API URL - configure this for your deployment
// For production, update this to your deployed backend URL
const API_URL = "https://physical-ai-and-humanoid-robotics-c-iota.vercel.app";

/**
 * Custom error class for authentication errors
 */
export class AuthenticationError extends Error {
  constructor(message: string = "Authentication required") {
    super(message);
    this.name = "AuthenticationError";
  }
}

/**
 * Send a message to the chat API and get a response
 * Requires authentication - includes credentials for cookie-based auth
 */
export async function sendMessage(
  message: string,
  conversationId?: string
): Promise<ChatResponse> {
  const request: ChatRequest = {
    message,
    conversationId,
  };

  const response = await fetch(`${API_URL}/api/chat`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    // Include credentials (cookies) for authentication
    credentials: "include",
    body: JSON.stringify(request),
  });

  // Handle 401 Unauthorized - authentication required
  if (response.status === 401) {
    const errorData = await response.json().catch(() => ({}));
    throw new AuthenticationError(
      errorData.detail?.message || "Session expired. Please sign in again."
    );
  }

  if (!response.ok) {
    const errorData = await response.json().catch(() => ({}));
    throw new Error(
      errorData.message || errorData.detail?.message || `Request failed with status ${response.status}`
    );
  }

  const data = await response.json();

  // Transform snake_case to camelCase for frontend
  return {
    response: data.response,
    sources: data.sources.map((s: any) => ({
      chapter: s.chapter,
      section: s.section,
      filePath: s.file_path,
      relevance: s.relevance,
    })),
    conversationId: data.conversation_id,
  };
}

/**
 * Check if the API is healthy
 */
export async function checkHealth(): Promise<boolean> {
  try {
    const response = await fetch(`${API_URL}/api/health`);
    return response.ok;
  } catch {
    return false;
  }
}
