"""Pydantic models for authentication."""

from datetime import datetime
from typing import Optional
from pydantic import BaseModel, EmailStr


class User(BaseModel):
    """User model matching Better Auth user table."""

    id: str
    name: str
    email: EmailStr
    email_verified: bool = False
    image: Optional[str] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class Session(BaseModel):
    """Session model matching Better Auth session table."""

    id: str
    user_id: str
    token: str
    expires_at: datetime
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class CurrentUser(BaseModel):
    """
    Authenticated user context for request handlers.

    This is a lightweight model used in FastAPI dependencies
    to provide user information to protected endpoints.
    """

    id: str
    name: str
    email: EmailStr
    image: Optional[str] = None
    session_id: str


class AuthError(BaseModel):
    """Authentication error response."""

    error: str
    message: str
