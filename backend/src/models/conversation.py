"""
Conversation Models
Purpose: Conversation and ChatInteraction Pydantic schemas for multi-turn dialogue
Date: 2025-12-14
"""

from datetime import datetime
from typing import List, Optional

from pydantic import BaseModel, Field


# ============================================================================
# Conversation Model
# ============================================================================


class Conversation(BaseModel):
    """Conversation model for database operations."""

    conversation_id: str = Field(..., description="Conversation ID (UUID)")
    user_id: str = Field(..., description="User ID (UUID)")
    title: str = Field(..., description="Conversation title (auto-generated from first query)")
    archived: bool = Field(default=False, description="Archived status")
    created_at: datetime = Field(..., description="Conversation creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")


class ConversationCreate(BaseModel):
    """Conversation creation schema."""

    user_id: str
    title: str = "New Conversation"


class ConversationResponse(BaseModel):
    """Conversation response schema with interaction count."""

    conversation_id: str
    user_id: str
    title: str
    archived: bool
    created_at: datetime
    updated_at: datetime
    interaction_count: Optional[int] = None


# ============================================================================
# Chat Interaction Model
# ============================================================================


class ChatInteraction(BaseModel):
    """Chat interaction model for database operations."""

    interaction_id: str = Field(..., description="Interaction ID (UUID)")
    conversation_id: str = Field(..., description="Conversation ID (UUID)")
    user_query: str = Field(..., description="User's query text")
    bot_response: str = Field(..., description="Bot's response text")
    source_chunks: Optional[List[str]] = Field(None, description="Source chunk IDs from Qdrant")
    query_mode: str = Field(
        default="global", description="Query mode: 'global' or 'local' (selected text)"
    )
    response_time_ms: Optional[int] = Field(None, description="Response time in milliseconds")
    created_at: datetime = Field(..., description="Interaction creation timestamp")


class ChatInteractionCreate(BaseModel):
    """Chat interaction creation schema."""

    conversation_id: str
    user_query: str
    bot_response: str
    source_chunks: Optional[List[str]] = None
    query_mode: str = "global"
    response_time_ms: Optional[int] = None


class ChatInteractionResponse(BaseModel):
    """Chat interaction response schema."""

    interaction_id: str
    conversation_id: str
    user_query: str
    bot_response: str
    source_chunks: Optional[List[str]] = None
    query_mode: str
    response_time_ms: Optional[int] = None
    created_at: datetime


# ============================================================================
# Chat Request/Response Models
# ============================================================================


class ChatRequest(BaseModel):
    """Chat request schema for /chat/global endpoint."""

    query: str = Field(..., min_length=1, max_length=1000, description="User's query")
    conversation_id: Optional[str] = Field(
        None, description="Conversation ID for multi-turn context (optional)"
    )


class LocalChatRequest(BaseModel):
    """Local chat request schema for /chat/local endpoint."""

    query: str = Field(..., min_length=1, max_length=1000, description="User's query")
    selected_text: str = Field(
        ..., min_length=50, description="Selected text for context isolation"
    )
    file_path: str = Field(..., description="Source file path for selected text")
    chunk_indices: Optional[List[int]] = Field(
        None, description="Chunk indices for selected text (optional)"
    )


class SourceReference(BaseModel):
    """Source reference schema for chat responses."""

    chunk_id: str = Field(..., description="Qdrant chunk ID")
    file_path: str = Field(..., description="Source file path")
    content: str = Field(..., description="Chunk content excerpt")
    score: float = Field(..., description="Relevance score")


class ChatResponse(BaseModel):
    """Chat response schema."""

    answer: str = Field(..., description="Bot's grounded answer")
    sources: List[SourceReference] = Field(..., description="Source references with citations")
    conversation_id: str = Field(..., description="Conversation ID")
    interaction_id: str = Field(..., description="Interaction ID")
    query_mode: str = Field(..., description="Query mode: 'global' or 'local'")


# ============================================================================
# Conversation History Models
# ============================================================================


class ConversationDetailResponse(BaseModel):
    """Conversation detail response with all interactions."""

    conversation_id: str
    user_id: str
    title: str
    archived: bool
    created_at: datetime
    updated_at: datetime
    interactions: List[ChatInteractionResponse]


class ConversationListResponse(BaseModel):
    """Paginated conversation list response."""

    conversations: List[ConversationResponse]
    total: int
    skip: int
    limit: int
