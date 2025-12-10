"""Data models for Markdown files and text chunks."""

from datetime import datetime
from pathlib import Path
from typing import Optional
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, field_validator


class MarkdownFile(BaseModel):
    """Represents a source Markdown document."""

    file_path: str = Field(..., description="Path to the Markdown file")
    raw_content: str = Field(..., description="Raw Markdown content")
    ast: Optional[dict] = Field(None, description="Mistune AST representation")
    heading_path: Optional[list[str]] = Field(None, description="List of heading titles")
    file_size_bytes: int = Field(..., description="File size in bytes")
    last_modified: datetime = Field(..., description="Last modified timestamp")

    @field_validator("file_path")
    @classmethod
    def validate_file_path(cls, v: str) -> str:
        """Validate that file_path starts with 'docs/' and ends with '.md'."""
        if not v.startswith("docs/"):
            raise ValueError(f"file_path must start with 'docs/', got: {v}")
        if not v.endswith(".md"):
            raise ValueError(f"file_path must end with '.md', got: {v}")
        return v


class TextChunk(BaseModel):
    """Represents a semantically coherent text chunk."""

    chunk_id: UUID = Field(default_factory=uuid4, description="Unique chunk identifier")
    file_path: str = Field(..., description="Source Markdown file path")
    section: str = Field(..., description="Section heading")
    text: str = Field(..., description="Chunk text content")
    token_count: int = Field(..., ge=300, le=1000, description="Number of tokens (300-1,000)")
    chunk_index: int = Field(..., ge=0, description="Zero-based chunk index in file")
    overlap_with_prev: bool = Field(False, description="Whether this chunk overlaps with previous")
    overlap_with_next: bool = Field(False, description="Whether this chunk overlaps with next")

    @field_validator("text")
    @classmethod
    def validate_text(cls, v: str) -> str:
        """Validate that text is non-empty after stripping."""
        if not v.strip():
            raise ValueError("text cannot be empty or whitespace-only")
        return v
