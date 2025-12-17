"""
Middleware
Purpose: JWT validation, authentication dependency, rate limiting
Date: 2025-12-13
"""

import logging
from typing import Optional

from fastapi import Cookie, Depends, HTTPException, Request, status
from slowapi import Limiter
from slowapi.util import get_remote_address

from .config import settings
from .security import decode_access_token

logger = logging.getLogger(__name__)

# ============================================================================
# Rate Limiting
# ============================================================================

limiter = Limiter(key_func=get_remote_address)


def setup_rate_limiting(app):
    """Setup rate limiting middleware for FastAPI app."""
    app.state.limiter = limiter
    return app


# ============================================================================
# Authentication Middleware
# ============================================================================


async def get_current_user(session_token: Optional[str] = Cookie(None, alias="session")) -> dict:
    """
    Dependency to get current authenticated user from JWT token.

    Args:
        session_token: JWT token from HTTP-only cookie

    Returns:
        User data dict with user_id

    Raises:
        HTTPException: If token is missing or invalid
    """
    if not session_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated. Please sign in.",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Decode JWT token
    payload = decode_access_token(session_token)
    if not payload:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired session. Please sign in again.",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Extract user ID from token
    user_id = payload.get("sub")
    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token format.",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return {"user_id": user_id}


async def get_optional_user(session_token: Optional[str] = Cookie(None, alias="session")) -> Optional[dict]:
    """
    Dependency to get current user if authenticated, None otherwise.

    Args:
        session_token: JWT token from HTTP-only cookie

    Returns:
        User data dict if authenticated, None otherwise
    """
    if not session_token:
        return None

    payload = decode_access_token(session_token)
    if not payload:
        return None

    user_id = payload.get("sub")
    if not user_id:
        return None

    return {"user_id": user_id}


# ============================================================================
# CSRF Protection Middleware
# ============================================================================


async def verify_csrf(
    csrf_token: Optional[str] = Cookie(None, alias="csrf_token"),
    x_csrf_token: Optional[str] = None,
):
    """
    Verify CSRF token for mutation requests (POST, PUT, DELETE).

    Args:
        csrf_token: CSRF token from cookie
        x_csrf_token: CSRF token from X-CSRF-Token header

    Raises:
        HTTPException: If CSRF validation fails
    """
    if not csrf_token or not x_csrf_token:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN, detail="CSRF token missing"
        )

    if csrf_token != x_csrf_token:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN, detail="CSRF validation failed"
        )


# ============================================================================
# Request Logging Middleware
# ============================================================================


async def log_requests(request: Request, call_next):
    """
    Middleware to log all incoming requests.

    Args:
        request: FastAPI request object
        call_next: Next middleware in chain

    Returns:
        Response from downstream middleware/endpoint
    """
    logger.info(f"{request.method} {request.url.path} - {get_remote_address(request)}")

    response = await call_next(request)

    logger.info(
        f"{request.method} {request.url.path} - {response.status_code} - {get_remote_address(request)}"
    )

    return response
