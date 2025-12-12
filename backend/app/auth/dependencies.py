"""FastAPI dependencies for authentication."""

import logging
from typing import Optional
from fastapi import Depends, HTTPException, Request
from app.db.session import get_db_pool
from app.models.auth import CurrentUser

logger = logging.getLogger(__name__)

# Better Auth session cookie names (varies by environment)
SESSION_COOKIE_NAMES = [
    "_Secure-better-auth_token",      # Production with secure cookies
    "better-auth.session_token",       # Development / legacy
    "better-auth_token",               # Alternative format
]


async def get_current_user(request: Request) -> CurrentUser:
    """
    Validate the session token and return the current authenticated user.

    This dependency queries the Neon PostgreSQL database to validate
    the session token from the Better Auth cookie.

    Raises:
        HTTPException: 401 if not authenticated or session expired
    """
    # Debug: Log all cookies received
    all_cookies = dict(request.cookies)
    logger.info(f"[Auth Debug] All cookies received: {list(all_cookies.keys())}")

    # Get session token from cookie - try multiple possible names
    session_token = None
    for cookie_name in SESSION_COOKIE_NAMES:
        session_token = request.cookies.get(cookie_name)
        if session_token:
            logger.info(f"[Auth Debug] Found session token in cookie: {cookie_name}")
            break

    if not session_token:
        logger.warning(f"No session token found. Available cookies: {list(all_cookies.keys())}")
        raise HTTPException(
            status_code=401,
            detail={"error": "not_authenticated", "message": f"Not authenticated. Cookies received: {list(all_cookies.keys())}"},
        )

    try:
        # Get database pool
        pool = await get_db_pool()

        # Query session and user from Neon database
        # Better Auth stores sessions with userId foreign key to user table
        row = await pool.fetchrow(
            """
            SELECT
                u.id as user_id,
                u.name,
                u.email,
                u.image,
                s.id as session_id,
                s."expiresAt" as expires_at
            FROM session s
            JOIN "user" u ON s."userId" = u.id
            WHERE s.token = $1 AND s."expiresAt" > NOW()
            """,
            session_token,
        )

        if not row:
            logger.debug("Session not found or expired")
            raise HTTPException(
                status_code=401,
                detail={"error": "session_expired", "message": "Session expired"},
            )

        # Return authenticated user context
        return CurrentUser(
            id=row["user_id"],
            name=row["name"],
            email=row["email"],
            image=row["image"],
            session_id=row["session_id"],
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error validating session: {e}")
        raise HTTPException(
            status_code=500,
            detail={"error": "auth_error", "message": "Authentication error"},
        )


async def get_optional_user(request: Request) -> Optional[CurrentUser]:
    """
    Get the current user if authenticated, otherwise return None.

    This is useful for endpoints that have optional authentication.
    """
    try:
        return await get_current_user(request)
    except HTTPException:
        return None
