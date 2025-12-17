"""
Validation Utilities
Purpose: Pydantic schemas for request/response validation
Date: 2025-12-13
"""

import re
from typing import List, Optional

from pydantic import BaseModel, EmailStr, Field, field_validator


# ============================================================================
# Common Validators
# ============================================================================


def validate_email_format(email: str) -> bool:
    """Validate email format using regex."""
    pattern = r"^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$"
    return re.match(pattern, email) is not None


def validate_password_strength(password: str) -> tuple[bool, Optional[str]]:
    """Validate password meets minimum requirements."""
    if len(password) < 8:
        return False, "Password must be at least 8 characters long"
    return True, None


# ============================================================================
# Authentication Schemas
# ============================================================================


class SignupRequest(BaseModel):
    """Signup request schema."""

    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., min_length=8, description="User password (min 8 characters)")
    full_name: str = Field(..., min_length=2, description="User full name")
    experience_level: str = Field(
        ..., description="Experience level: Beginner, Intermediate, Advanced, Expert"
    )
    programming_languages: List[str] = Field(..., description="List of programming languages")
    frameworks: List[str] = Field(default=[], description="List of frameworks")
    available_hardware: List[str] = Field(default=[], description="Available compute hardware")
    robotics_hardware: List[str] = Field(default=[], description="Available robotics hardware")

    @field_validator("experience_level")
    @classmethod
    def validate_experience_level(cls, v):
        """Validate experience level is one of allowed values."""
        allowed = ["Beginner", "Intermediate", "Advanced", "Expert"]
        if v not in allowed:
            raise ValueError(f"Experience level must be one of: {', '.join(allowed)}")
        return v


class SigninRequest(BaseModel):
    """Signin request schema."""

    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., description="User password")
    remember_me: bool = Field(default=False, description="Remember me option")


class UserResponse(BaseModel):
    """User response schema."""

    id: str = Field(..., description="User ID")
    email: str = Field(..., description="User email")
    full_name: str = Field(..., description="User full name")
    created_at: str = Field(..., description="Account creation timestamp")


# ============================================================================
# Chat Schemas
# ============================================================================


class GlobalQueryRequest(BaseModel):
    """Global book query request schema."""

    query: str = Field(..., min_length=3, description="User question about book content")
    conversation_id: Optional[str] = Field(None, description="Optional conversation ID")


class LocalQueryRequest(BaseModel):
    """Selected text query request schema."""

    query: str = Field(..., min_length=3, description="User question about selected text")
    selected_text: str = Field(..., min_length=50, description="Selected text from book")
    file_path: str = Field(..., description="File path of selected text")
    chunk_indices: List[int] = Field(..., description="Chunk indices for selected text")
    conversation_id: Optional[str] = Field(None, description="Optional conversation ID")


class ChatResponse(BaseModel):
    """Chat response schema."""

    answer: str = Field(..., description="Chatbot answer")
    sources: List[dict] = Field(..., description="Source references from book")
    conversation_id: str = Field(..., description="Conversation ID")


# ============================================================================
# Conversation Schemas
# ============================================================================


class ConversationResponse(BaseModel):
    """Conversation response schema."""

    id: str
    title: Optional[str]
    created_at: str
    updated_at: str
    archived: bool


class ChatInteractionResponse(BaseModel):
    """Chat interaction response schema."""

    id: str
    query_text: str
    answer_text: str
    query_mode: str
    created_at: str


# ============================================================================
# Progress Schemas
# ============================================================================


class UpdateProgressRequest(BaseModel):
    """Update reading progress request schema."""

    chapter_id: str = Field(..., description="Chapter identifier")
    completion_percentage: int = Field(..., ge=0, le=100, description="Completion percentage")
    time_spent_seconds: int = Field(..., ge=0, description="Time spent on chapter in seconds")


class ProgressResponse(BaseModel):
    """Reading progress response schema."""

    chapter_id: str
    completion_percentage: int
    time_spent_seconds: int
    completed: bool
    last_accessed: str


# ============================================================================
# Recommendation Schemas
# ============================================================================


class RecommendationResponse(BaseModel):
    """Recommendation response schema."""

    id: str
    recommended_chapter_id: str
    score: float
    reason: str
    dismissed: bool
    created_at: str


# ============================================================================
# Error Response Schema
# ============================================================================


class ErrorResponse(BaseModel):
    """Standard error response schema."""

    error: str = Field(..., description="Error message")
    detail: Optional[str] = Field(None, description="Detailed error information")
    status_code: int = Field(..., description="HTTP status code")
