"""Data models for queries and retrieval results."""

from datetime import datetime
from typing import List, Optional
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, model_validator, field_validator


class Query(BaseModel):
    """Represents a user query."""

    query_id: UUID = Field(default_factory=uuid4, description="Unique query identifier")
    query_text: str = Field(..., min_length=1, max_length=500, description="Query text (1-500 chars)")
    query_type: str = Field(..., description="Query type: 'full_book' or 'local'")
    selected_text: Optional[str] = Field(None, description="Selected text for local queries")
    source_file_path: Optional[str] = Field(None, description="Source file path for local queries")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Query timestamp")

    @model_validator(mode='after')
    def validate_local_query(self):
        """Ensure selected_text and source_file_path are required for local queries."""
        if self.query_type == 'local':
            if not self.selected_text:
                raise ValueError("selected_text is required when query_type is 'local'")
            if not self.source_file_path:
                raise ValueError("source_file_path is required when query_type is 'local'")
        return self


class RetrievedChunk(BaseModel):
    """Represents a retrieved chunk from Qdrant."""

    chunk_id: str = Field(..., description="Chunk identifier")
    file_path: str = Field(..., description="Source file path")
    section: str = Field(..., description="Section heading")
    text: str = Field(..., description="Chunk text content")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Similarity score (0.0-1.0)")
    rank: int = Field(..., ge=1, le=3, description="Rank (1-3)")


class RetrievalResult(BaseModel):
    """Represents the complete retrieval result."""

    query_id: UUID = Field(..., description="Query identifier")
    chunks: List[RetrievedChunk] = Field(..., description="Retrieved chunks (exactly 3)")
    retrieval_latency_ms: int = Field(..., description="Retrieval latency in milliseconds")

    @field_validator("chunks")
    @classmethod
    def validate_chunks(cls, v: List[RetrievedChunk]) -> List[RetrievedChunk]:
        """Ensure chunks list has exactly 3 elements and is sorted by similarity_score."""
        if len(v) != 3:
            raise ValueError(f"chunks must have exactly 3 elements, got {len(v)}")

        # Check that chunks are sorted by similarity_score descending
        scores = [chunk.similarity_score for chunk in v]
        if scores != sorted(scores, reverse=True):
            raise ValueError("chunks must be sorted by similarity_score descending")

        # Check that ranks are [1, 2, 3]
        ranks = [chunk.rank for chunk in v]
        if ranks != [1, 2, 3]:
            raise ValueError(f"ranks must be [1, 2, 3], got {ranks}")

        return v
