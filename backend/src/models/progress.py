"""
Reading Progress Models
Purpose: ReadingProgress schema with user_id, chapter_id, completion_percentage, time_spent_seconds, completed
Date: 2025-12-14
"""

from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field


# ============================================================================
# Reading Progress Model
# ============================================================================


class ReadingProgress(BaseModel):
    """Reading progress model for database operations."""

    progress_id: str = Field(..., description="Progress ID (UUID)")
    user_id: str = Field(..., description="User ID (UUID)")
    chapter_id: str = Field(..., description="Chapter ID")
    completion_percentage: int = Field(..., ge=0, le=100, description="Completion percentage (0-100)")
    time_spent_seconds: int = Field(..., ge=0, description="Time spent reading in seconds")
    completed: bool = Field(default=False, description="Whether chapter is completed (>=90%)")
    last_position: Optional[int] = Field(None, description="Last scroll position")
    created_at: datetime = Field(..., description="First read timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")


class ReadingProgressCreate(BaseModel):
    """Reading progress creation schema."""

    chapter_id: str = Field(..., description="Chapter ID")
    completion_percentage: int = Field(..., ge=0, le=100, description="Completion percentage")
    time_spent_seconds: int = Field(..., ge=0, description="Time spent in seconds")
    last_position: Optional[int] = Field(None, description="Last scroll position")


class ReadingProgressUpdate(BaseModel):
    """Reading progress update schema."""

    completion_percentage: int = Field(..., ge=0, le=100, description="Completion percentage")
    time_spent_seconds: int = Field(..., ge=0, description="Time spent in seconds")
    last_position: Optional[int] = Field(None, description="Last scroll position")


class ReadingProgressResponse(BaseModel):
    """Reading progress response schema."""

    progress_id: str
    chapter_id: str
    chapter_title: str
    completion_percentage: int
    time_spent_seconds: int
    completed: bool
    last_position: Optional[int]
    created_at: datetime
    updated_at: datetime


class ProgressListResponse(BaseModel):
    """List of reading progress response."""

    progress: list[ReadingProgressResponse]
    total_chapters: int
    completed_chapters: int
