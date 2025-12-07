"""Pydantic schemas for the RAG chatbot API."""

from pydantic import BaseModel, Field
from typing import Optional
from enum import Enum


class Role(str, Enum):
    """Message role enum."""
    USER = "user"
    ASSISTANT = "assistant"


class Source(BaseModel):
    """A reference to documentation content used in a response."""
    chapter: str
    section: str
    file_path: str
    relevance: float = Field(ge=0.0, le=1.0)


class ChatRequest(BaseModel):
    """Request body for the chat endpoint."""
    message: str = Field(min_length=1, max_length=1000)
    conversation_id: Optional[str] = None


class ChatResponse(BaseModel):
    """Response body for the chat endpoint."""
    response: str
    sources: list[Source]
    conversation_id: str


class HealthResponse(BaseModel):
    """Response body for the health endpoint."""
    status: str
    version: str


class ErrorResponse(BaseModel):
    """Error response body."""
    error: str
    message: str
