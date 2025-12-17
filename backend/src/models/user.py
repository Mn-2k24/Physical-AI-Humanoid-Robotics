"""
User Models
Purpose: User, UserProfile, UserSoftwareBackground, UserHardwareBackground Pydantic schemas
Date: 2025-12-13
"""

from datetime import datetime
from typing import List, Optional

from pydantic import BaseModel, EmailStr, Field


# ============================================================================
# User Model
# ============================================================================


class User(BaseModel):
    """User model for database operations."""

    id: str = Field(..., description="User ID (UUID)")
    email: EmailStr = Field(..., description="User email address")
    hashed_password: str = Field(..., description="Hashed password")
    full_name: str = Field(..., description="User full name")
    created_at: datetime = Field(..., description="Account creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")


class UserCreate(BaseModel):
    """User creation schema."""

    email: EmailStr
    password: str = Field(..., min_length=8)
    full_name: str = Field(..., min_length=2)


class UserInDB(User):
    """User model with hashed password for database storage."""

    pass


class UserPublic(BaseModel):
    """Public user information (no password)."""

    id: str
    email: str
    full_name: str
    created_at: datetime


# ============================================================================
# User Profile Model
# ============================================================================


class UserProfile(BaseModel):
    """User profile model."""

    user_id: str = Field(..., description="User ID (UUID)")
    experience_level: str = Field(
        ..., description="Experience level: Beginner, Intermediate, Advanced, Expert"
    )
    created_at: datetime
    updated_at: datetime


class UserProfileCreate(BaseModel):
    """User profile creation schema."""

    user_id: str
    experience_level: str


# ============================================================================
# User Software Background Model
# ============================================================================


class UserSoftwareBackground(BaseModel):
    """User software background model."""

    user_id: str = Field(..., description="User ID (UUID)")
    programming_languages: List[str] = Field(
        default=[], description="List of programming languages"
    )
    frameworks: List[str] = Field(default=[], description="List of frameworks")


class UserSoftwareBackgroundCreate(BaseModel):
    """User software background creation schema."""

    user_id: str
    programming_languages: List[str]
    frameworks: List[str] = []


# ============================================================================
# User Hardware Background Model
# ============================================================================


class UserHardwareBackground(BaseModel):
    """User hardware background model."""

    user_id: str = Field(..., description="User ID (UUID)")
    available_hardware: List[str] = Field(
        default=[], description="Available compute hardware"
    )
    robotics_hardware: List[str] = Field(default=[], description="Available robotics hardware")


class UserHardwareBackgroundCreate(BaseModel):
    """User hardware background creation schema."""

    user_id: str
    available_hardware: List[str] = []
    robotics_hardware: List[str] = []


# ============================================================================
# Complete User with Profile
# ============================================================================


class UserWithProfile(BaseModel):
    """User with complete profile information."""

    user: UserPublic
    profile: Optional[UserProfile] = None
    software_background: Optional[UserSoftwareBackground] = None
    hardware_background: Optional[UserHardwareBackground] = None
