"""
Recommendation Models
Purpose: Recommendation schema with user_id, recommended_chapter_id, score, reason, dismissed
Date: 2025-12-14
"""

from datetime import datetime
from typing import Optional

from pydantic import BaseModel, Field


# ============================================================================
# Recommendation Model
# ============================================================================


class Recommendation(BaseModel):
    """Recommendation model for database operations."""

    recommendation_id: str = Field(..., description="Recommendation ID (UUID)")
    user_id: str = Field(..., description="User ID (UUID)")
    recommended_chapter_id: str = Field(..., description="Recommended chapter ID")
    score: float = Field(..., description="Recommendation score (0-100)")
    reason: str = Field(..., description="Reason for recommendation")
    dismissed: bool = Field(default=False, description="Whether user dismissed this recommendation")
    created_at: datetime = Field(..., description="Recommendation creation timestamp")


class RecommendationCreate(BaseModel):
    """Recommendation creation schema."""

    user_id: str
    recommended_chapter_id: str
    score: float
    reason: str


class RecommendationResponse(BaseModel):
    """Recommendation response schema."""

    recommendation_id: str
    recommended_chapter_id: str
    chapter_title: str
    score: float
    reason: str
    dismissed: bool
    created_at: datetime


class RecommendationListResponse(BaseModel):
    """List of recommendations response."""

    recommendations: list[RecommendationResponse]
    total: int
