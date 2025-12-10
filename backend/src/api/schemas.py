"""API request/response schemas."""

from typing import List
from pydantic import BaseModel, Field


# Request schemas
class AskRequest(BaseModel):
    """Request schema for /ask endpoint."""
    query: str = Field(..., min_length=1, max_length=500, description="User query")


class AskLocalRequest(BaseModel):
    """Request schema for /ask-local endpoint."""
    query: str = Field(..., min_length=1, max_length=500, description="User query")
    selected_text: str = Field(..., min_length=10, description="Selected text from book")
    source_file_path: str = Field(..., description="Source file path")


class TrackRequest(BaseModel):
    """Request schema for /track endpoint."""
    query_id: str = Field(..., description="Query identifier")
    query_text: str = Field(..., description="Query text")
    answer_text: str = Field(..., description="Answer text")
    sources: List[dict] = Field(..., description="Source citations")
    query_type: str = Field(..., description="Query type: full_book or local")
    latency_ms: int = Field(..., description="Total latency in milliseconds")


# Response schemas
class SourceCitationResponse(BaseModel):
    """Source citation in response."""
    file_path: str
    section: str
    similarity_score: float


class AskResponse(BaseModel):
    """Response schema for /ask and /ask-local endpoints."""
    query_id: str = Field(..., description="Query identifier")
    answer: str = Field(..., description="Generated answer")
    sources: List[SourceCitationResponse] = Field(..., description="Source citations")
    latency_ms: int = Field(..., description="Total latency in milliseconds")


class TrackResponse(BaseModel):
    """Response schema for /track endpoint."""
    conversation_id: str = Field(..., description="Conversation identifier")
    status: str = Field(..., description="Status: logged or failed")


class HealthResponse(BaseModel):
    """Response schema for /health endpoint."""
    status: str = Field(..., description="Overall status")
    qdrant_connected: bool = Field(..., description="Qdrant connection status")
    neon_connected: bool = Field(..., description="Neon connection status")
    model_loaded: bool = Field(..., description="Models loaded status")
