"""
Authentication Models
Purpose: AuthSession schema with session_id, user_id, token_hash, expires_at
Date: 2025-12-13
"""

from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field


# ============================================================================
# Auth Session Model
# ============================================================================


class AuthSession(BaseModel):
    """Auth session model for database operations."""

    session_id: str = Field(..., description="Session ID (UUID)")
    user_id: str = Field(..., description="User ID (UUID)")
    token_hash: str = Field(..., description="SHA-256 hash of session token")
    expires_at: datetime = Field(..., description="Session expiration timestamp")
    created_at: datetime = Field(..., description="Session creation timestamp")


class AuthSessionCreate(BaseModel):
    """Auth session creation schema."""

    user_id: str
    token_hash: str
    expires_at: datetime


# ============================================================================
# Authentication Request/Response Models
# ============================================================================


class SignupRequest(BaseModel):
    """Signup request with user details and background."""

    email: str = Field(..., description="User email address")
    password: str = Field(..., min_length=8, description="User password (min 8 characters)")
    full_name: str = Field(..., min_length=2, description="User full name")
    experience_level: str = Field(
        ..., description="Experience level: Beginner, Intermediate, Advanced, Expert"
    )
    programming_languages: list[str] = Field(..., description="List of programming languages")
    frameworks: list[str] = Field(default=[], description="List of frameworks")
    available_hardware: list[str] = Field(default=[], description="Available compute hardware")
    robotics_hardware: list[str] = Field(default=[], description="Available robotics hardware")


class SigninRequest(BaseModel):
    """Signin request with credentials."""

    email: str = Field(..., description="User email address")
    password: str = Field(..., description="User password")
    remember_me: bool = Field(default=False, description="Remember me option")


class AuthResponse(BaseModel):
    """Authentication response with user data and session info."""

    user: dict = Field(..., description="User data")
    session: dict = Field(..., description="Session information")


class SessionInfo(BaseModel):
    """Session information."""

    session_id: str
    expires_at: datetime


# ============================================================================
# Audit Log Model
# ============================================================================


class AuditLog(BaseModel):
    """Audit log model for authentication events."""

    id: str = Field(..., description="Audit log ID (UUID)")
    user_id: Optional[str] = Field(None, description="User ID (null for failed logins)")
    event_type: str = Field(..., description="Event type (signup_success, signin_failed, etc.)")
    details: Optional[dict] = Field(None, description="Additional event details")
    ip_address: Optional[str] = Field(None, description="Client IP address")
    user_agent: Optional[str] = Field(None, description="Client user agent string")
    created_at: datetime = Field(..., description="Event timestamp")


class AuditLogCreate(BaseModel):
    """Audit log creation schema."""

    user_id: Optional[str] = None
    event_type: str
    details: Optional[dict] = None
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None
