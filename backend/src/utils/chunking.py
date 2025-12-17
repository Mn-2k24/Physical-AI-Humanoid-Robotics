"""
Text Chunking Utility
Purpose: Semantic chunking, preserve paragraph boundaries, max 500 tokens per chunk
Date: 2025-12-13
"""

import logging
import re
from typing import List, Tuple

logger = logging.getLogger(__name__)


class TextChunker:
    """Semantic text chunking for book content."""

    def __init__(self, max_chunk_size: int = 500):
        """
        Initialize text chunker.

        Args:
            max_chunk_size: Maximum tokens per chunk (approximate)
        """
        self.max_chunk_size = max_chunk_size
        # Approximate 4 characters per token (English text average)
        self.max_chars = max_chunk_size * 4

    def chunk_text(self, text: str, preserve_paragraphs: bool = True) -> List[Tuple[str, int]]:
        """
        Chunk text into semantic segments.

        Args:
            text: Text to chunk
            preserve_paragraphs: Whether to preserve paragraph boundaries

        Returns:
            List of (chunk_text, chunk_index) tuples
        """
        if preserve_paragraphs:
            return self._chunk_by_paragraphs(text)
        else:
            return self._chunk_by_size(text)

    def _chunk_by_paragraphs(self, text: str) -> List[Tuple[str, int]]:
        """
        Chunk text by paragraph boundaries.

        Preserves semantic meaning by not splitting paragraphs unless they exceed max_chunk_size.

        Args:
            text: Text to chunk

        Returns:
            List of (chunk_text, chunk_index) tuples
        """
        # Split by double newlines (paragraph boundaries)
        paragraphs = re.split(r"\n\s*\n", text)

        chunks = []
        current_chunk = ""
        chunk_index = 0

        for paragraph in paragraphs:
            paragraph = paragraph.strip()
            if not paragraph:
                continue

            # If paragraph alone exceeds max size, split it
            if len(paragraph) > self.max_chars:
                # Save current chunk if not empty
                if current_chunk:
                    chunks.append((current_chunk.strip(), chunk_index))
                    chunk_index += 1
                    current_chunk = ""

                # Split large paragraph by sentences
                sentences = self._split_by_sentences(paragraph)
                for sentence in sentences:
                    if len(current_chunk) + len(sentence) > self.max_chars:
                        chunks.append((current_chunk.strip(), chunk_index))
                        chunk_index += 1
                        current_chunk = sentence
                    else:
                        current_chunk += " " + sentence if current_chunk else sentence

            # Check if adding paragraph exceeds max size
            elif len(current_chunk) + len(paragraph) > self.max_chars:
                # Save current chunk
                chunks.append((current_chunk.strip(), chunk_index))
                chunk_index += 1
                current_chunk = paragraph
            else:
                # Add paragraph to current chunk
                current_chunk += "\n\n" + paragraph if current_chunk else paragraph

        # Add final chunk
        if current_chunk.strip():
            chunks.append((current_chunk.strip(), chunk_index))

        logger.info(f"Chunked text into {len(chunks)} chunks (paragraph-based)")
        return chunks

    def _chunk_by_size(self, text: str) -> List[Tuple[str, int]]:
        """
        Chunk text by fixed size with sentence boundaries.

        Args:
            text: Text to chunk

        Returns:
            List of (chunk_text, chunk_index) tuples
        """
        sentences = self._split_by_sentences(text)

        chunks = []
        current_chunk = ""
        chunk_index = 0

        for sentence in sentences:
            if len(current_chunk) + len(sentence) > self.max_chars:
                if current_chunk:
                    chunks.append((current_chunk.strip(), chunk_index))
                    chunk_index += 1
                current_chunk = sentence
            else:
                current_chunk += " " + sentence if current_chunk else sentence

        # Add final chunk
        if current_chunk.strip():
            chunks.append((current_chunk.strip(), chunk_index))

        logger.info(f"Chunked text into {len(chunks)} chunks (size-based)")
        return chunks

    def _split_by_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences.

        Args:
            text: Text to split

        Returns:
            List of sentences
        """
        # Simple sentence splitting (handles ., !, ? followed by space or newline)
        sentences = re.split(r"(?<=[.!?])\s+", text)
        return [s.strip() for s in sentences if s.strip()]

    def extract_section_heading(self, text: str) -> str:
        """
        Extract section heading from markdown text.

        Args:
            text: Markdown text

        Returns:
            Section heading or "No heading"
        """
        # Extract first markdown heading (# Heading)
        heading_match = re.search(r"^#+\s+(.+)$", text, re.MULTILINE)
        if heading_match:
            return heading_match.group(1).strip()

        # Extract first line if it looks like a heading
        first_line = text.split("\n")[0].strip()
        if len(first_line) < 100:  # Likely a heading if short
            return first_line

        return "No heading"
