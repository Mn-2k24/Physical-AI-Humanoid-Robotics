"""Data models for chatbot answers."""

from datetime import datetime
from typing import List
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, field_validator


class SourceCitation(BaseModel):
    """Represents a source citation for an answer."""

    file_path: str = Field(..., description="Source file path")
    section: str = Field(..., description="Section heading")
    chunk_index: int = Field(..., description="Chunk index")
    similarity_score: float = Field(..., description="Similarity score")


class ChatbotAnswer(BaseModel):
    """Represents a generated answer from the chatbot."""

    query_id: UUID = Field(..., description="Query identifier")
    answer_text: str = Field(..., min_length=1, max_length=1000, description="Answer text (1-1000 chars)")
    sources: List[SourceCitation] = Field(..., description="Source citations (exactly 3)")
    generation_latency_ms: int = Field(..., description="Answer generation latency in milliseconds")
    total_latency_ms: int = Field(..., description="Total latency (retrieval + generation)")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Answer timestamp")

    @field_validator("sources")
    @classmethod
    def validate_sources(cls, v: List[SourceCitation]) -> List[SourceCitation]:
        """Ensure sources list has exactly 3 elements."""
        if len(v) != 3:
            raise ValueError(f"sources must have exactly 3 elements, got {len(v)}")
        return v

    @field_validator("total_latency_ms")
    @classmethod
    def validate_latency(cls, v: int, info) -> int:
        """Ensure total_latency_ms >= generation_latency_ms."""
        generation_latency = info.data.get("generation_latency_ms")
        if generation_latency and v < generation_latency:
            raise ValueError(f"total_latency_ms ({v}) must be >= generation_latency_ms ({generation_latency})")
        return v
