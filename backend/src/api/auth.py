"""
Authentication API Endpoints
Purpose: /auth/signup, /auth/signin, /auth/signout, /auth/me, /auth/refresh
Date: 2025-12-13
"""

import logging
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException, Request, Response, status

from ..core.middleware import get_current_user, limiter
from ..models.auth import SigninRequest, SignupRequest
from ..services.auth_service import auth_service

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/auth", tags=["Authentication"])


# ============================================================================
# T030: Signup Endpoint
# ============================================================================


@router.post("/signup", status_code=status.HTTP_201_CREATED)
@limiter.limit("5/minute")
async def signup(
    request: Request,
    response: Response,
    signup_data: SignupRequest,
):
    """
    Create new user account with background questionnaire.

    **Rate Limited**: 5 attempts per minute per IP

    Args:
        signup_data: Signup request with user details and background

    Returns:
        User data and session information

    Raises:
        400: Email already exists or validation fails
        429: Rate limit exceeded
    """
    try:
        # Get client info for audit logging
        ip_address = request.client.host if request.client else None
        user_agent = request.headers.get("user-agent")

        # Create user
        user_data, access_token = await auth_service.signup(
            signup_data, ip_address=ip_address, user_agent=user_agent
        )

        # Set HTTP-only cookie with JWT token
        response.set_cookie(
            key="session",
            value=access_token,
            httponly=True,
            secure=True,  # HTTPS only in production
            samesite="none",  # Allow cross-origin cookies for Vercel frontend
            max_age=86400,  # 24 hours
        )

        return {
            "user": user_data,
            "session": {
                "message": "Session created successfully",
                "token": access_token,  # Return token for cross-origin auth
            },
        }

    except ValueError as e:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(e))
    except Exception as e:
        logger.error(f"Signup error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create account. Please try again.",
        )


# ============================================================================
# T031: Signin Endpoint
# ============================================================================


@router.post("/signin")
@limiter.limit("5/minute")
async def signin(
    request: Request,
    response: Response,
    signin_data: SigninRequest,
):
    """
    Authenticate user with email and password.

    **Rate Limited**: 5 attempts per minute per IP

    Args:
        signin_data: Signin request with email and password

    Returns:
        User data and session information

    Raises:
        401: Invalid credentials
        429: Rate limit exceeded
    """
    try:
        # Get client info for audit logging
        ip_address = request.client.host if request.client else None
        user_agent = request.headers.get("user-agent")

        # Authenticate user
        user_data, access_token = await auth_service.signin(
            signin_data, ip_address=ip_address, user_agent=user_agent
        )

        # Set HTTP-only cookie with JWT token
        max_age = 2592000 if signin_data.remember_me else 86400  # 30 days or 24 hours
        response.set_cookie(
            key="session",
            value=access_token,
            httponly=True,
            secure=True,
            samesite="none",  # Allow cross-origin cookies for Vercel frontend
            max_age=max_age,
        )

        return {
            "user": user_data,
            "session": {
                "message": "Signin successful",
                "remember_me": signin_data.remember_me,
                "token": access_token,  # Return token for cross-origin auth
            },
        }

    except ValueError as e:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail=str(e))
    except Exception as e:
        logger.error(f"Signin error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to sign in. Please try again.",
        )


# ============================================================================
# T032: Signout Endpoint
# ============================================================================


@router.post("/signout")
async def signout(
    response: Response,
    session_token: Optional[str] = None,
    current_user: dict = Depends(get_current_user),
):
    """
    Sign out user by deleting session.

    **Requires Authentication**

    Returns:
        Success message

    Raises:
        401: Not authenticated
    """
    try:
        # Sign out user
        if session_token:
            await auth_service.signout(current_user["user_id"], session_token)

        # Clear session cookie
        response.delete_cookie(key="session", samesite="none", secure=True)

        return {"message": "Signout successful"}

    except Exception as e:
        logger.error(f"Signout error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to sign out. Please try again.",
        )


# ============================================================================
# T033: Get Current User Endpoint
# ============================================================================


@router.get("/me")
async def get_me(current_user: dict = Depends(get_current_user)):
    """
    Get current logged-in user with profile data.

    **Requires Authentication**

    Returns:
        User data with profile, software background, and hardware background

    Raises:
        401: Not authenticated
    """
    try:
        user_data = await auth_service.get_current_user(current_user["user_id"])

        if not user_data:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND, detail="User not found"
            )

        return user_data

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get current user error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve user data.",
        )


# ============================================================================
# T034: Refresh Token Endpoint
# ============================================================================


@router.post("/refresh")
async def refresh_token(
    response: Response,
    session_token: Optional[str] = None,
    current_user: dict = Depends(get_current_user),
):
    """
    Refresh JWT token and extend session.

    **Requires Authentication**

    Returns:
        Success message with new session

    Raises:
        401: Not authenticated
    """
    try:
        if not session_token:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED, detail="No session token provided"
            )

        # Refresh token
        new_token = await auth_service.refresh_token(current_user["user_id"], session_token)

        # Set new cookie
        response.set_cookie(
            key="session",
            value=new_token,
            httponly=True,
            secure=True,
            samesite="none",  # Allow cross-origin cookies for Vercel frontend
            max_age=86400,  # 24 hours
        )

        return {"message": "Token refreshed successfully"}

    except Exception as e:
        logger.error(f"Refresh token error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to refresh token. Please sign in again.",
        )
