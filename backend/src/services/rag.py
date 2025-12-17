"""
RAG Service
Purpose: Retrieve chunks from Qdrant and generate grounded answers using Gemini
Date: 2025-12-14
"""

import logging
import time
from datetime import datetime, timedelta
from functools import lru_cache
from hashlib import sha256
from typing import List, Optional, Tuple

import google.generativeai as genai

from ..core.config import settings
from .embedding import embedding_service
from .qdrant import qdrant_service

logger = logging.getLogger(__name__)

# ============================================================================
# Quota Tracking
# ============================================================================

_daily_request_count = 0
_quota_reset_date = datetime.utcnow().date()
DAILY_QUOTA_LIMIT = 1200  # Gemini API daily quota


def _reset_quota_if_needed():
    """Reset quota counter if new day."""
    global _daily_request_count, _quota_reset_date
    current_date = datetime.utcnow().date()

    if current_date > _quota_reset_date:
        _daily_request_count = 0
        _quota_reset_date = current_date
        logger.info(f"Quota counter reset for {current_date}")


def _increment_quota() -> bool:
    """
    Increment quota counter and check if limit exceeded.

    Returns:
        True if quota available, False if exceeded
    """
    global _daily_request_count
    _reset_quota_if_needed()

    if _daily_request_count >= DAILY_QUOTA_LIMIT:
        logger.warning(f"Daily quota exceeded: {_daily_request_count}/{DAILY_QUOTA_LIMIT}")
        return False

    _daily_request_count += 1
    return True


# ============================================================================
# Response Caching
# ============================================================================


def _generate_cache_key(query: str, context: str) -> str:
    """Generate cache key from query and context."""
    combined = f"{query}|{context}"
    return sha256(combined.encode()).hexdigest()


# Cache: (answer, sources, timestamp)
_response_cache: dict[str, Tuple[str, List[dict], datetime]] = {}
CACHE_TTL_MINUTES = 30
MAX_CACHE_SIZE = 1000


def _get_cached_response(
    query: str, context: str
) -> Optional[Tuple[str, List[dict]]]:
    """Get cached response if available and not expired."""
    cache_key = _generate_cache_key(query, context)

    if cache_key in _response_cache:
        answer, sources, timestamp = _response_cache[cache_key]

        # Check if cache is still valid
        if datetime.utcnow() - timestamp < timedelta(minutes=CACHE_TTL_MINUTES):
            logger.info(f"Cache hit for query: {query[:50]}...")
            return answer, sources

        # Remove expired cache entry
        del _response_cache[cache_key]

    return None


def _cache_response(query: str, context: str, answer: str, sources: List[dict]):
    """Cache response with timestamp."""
    # Enforce max cache size
    if len(_response_cache) >= MAX_CACHE_SIZE:
        # Remove oldest entry
        oldest_key = min(_response_cache.keys(), key=lambda k: _response_cache[k][2])
        del _response_cache[oldest_key]

    cache_key = _generate_cache_key(query, context)
    _response_cache[cache_key] = (answer, sources, datetime.utcnow())


# ============================================================================
# RAG Service
# ============================================================================


class RAGService:
    """RAG service for retrieval and generation."""

    def __init__(self):
        """Initialize RAG service with Gemini model."""
        genai.configure(api_key=settings.gemini_api_key)
        self.model = genai.GenerativeModel("gemini-2.0-flash-exp")
        self.top_k = 5  # Number of chunks to retrieve

        # Strict grounding system prompt
        self.system_prompt = """You are a helpful AI assistant specialized in Physical AI and robotics.

CRITICAL INSTRUCTIONS - YOU MUST FOLLOW THESE EXACTLY:
1. ONLY answer questions using the provided context from the Physical AI book
2. NEVER use information from your general knowledge
3. If the answer is not in the provided context, say "I don't have enough information in the book to answer this question."
4. ALWAYS cite the source file path when providing information
5. Maintain conversation context from previous turns when relevant
6. Be concise and technical when appropriate
7. Use code examples from the context when available

Remember: Your knowledge is STRICTLY LIMITED to the provided book content."""

    async def retrieve_chunks(
        self, query: str, top_k: Optional[int] = None,
        file_path: Optional[str] = None, chunk_indices: Optional[List[int]] = None
    ) -> List[dict]:
        """
        Retrieve relevant chunks from Qdrant.

        Args:
            query: User's query
            top_k: Number of chunks to retrieve (default: 5)
            file_path: Optional file path filter for selected-text queries
            chunk_indices: Optional chunk index range for selected-text queries

        Returns:
            List of retrieved chunks with metadata
        """
        k = top_k or self.top_k

        try:
            # Generate embedding for query
            query_vector = embedding_service.generate_embedding(
                text=query, task_type="retrieval_query"
            )

            # Search Qdrant with query vector
            results = qdrant_service.search(
                query_vector=query_vector,
                limit=k,
                file_path=file_path,
                chunk_indices=chunk_indices
            )

            chunks = []
            for result in results:
                chunks.append(
                    {
                        "chunk_id": result.get("id", ""),
                        "content": result.get("raw_text", ""),
                        "file_path": result.get("file_path", ""),
                        "chunk_index": result.get("chunk_index", 0),
                        "score": result.get("score", 0.0),
                    }
                )

            logger.info(f"Retrieved {len(chunks)} chunks for query: {query[:50]}...")
            return chunks

        except Exception as e:
            logger.error(f"Error retrieving chunks: {e}", exc_info=True)
            return []

    def _format_context(self, chunks: List[dict]) -> str:
        """Format retrieved chunks into context string."""
        if not chunks:
            return "No relevant context found."

        context_parts = []
        for i, chunk in enumerate(chunks, 1):
            file_path = chunk.get("file_path", "unknown")
            content = chunk.get("content", "")
            context_parts.append(f"[Source {i}: {file_path}]\n{content}\n")

        return "\n".join(context_parts)

    def _format_conversation_history(
        self, conversation_history: Optional[List[dict]]
    ) -> str:
        """Format conversation history for context."""
        if not conversation_history:
            return ""

        history_parts = ["Previous conversation:"]
        for interaction in conversation_history[-5:]:  # Last 5 turns
            user_query = interaction.get("user_query", "")
            bot_response = interaction.get("bot_response", "")
            history_parts.append(f"User: {user_query}")
            history_parts.append(f"Assistant: {bot_response}")

        return "\n".join(history_parts) + "\n\n"

    async def generate_answer(
        self,
        query: str,
        chunks: List[dict],
        conversation_history: Optional[List[dict]] = None,
    ) -> Tuple[str, List[dict]]:
        """
        Generate grounded answer using Gemini.

        Args:
            query: User's query
            chunks: Retrieved chunks from Qdrant
            conversation_history: Previous conversation turns (optional)

        Returns:
            Tuple of (answer, source_references)

        Raises:
            ValueError: If quota exceeded
        """
        # Check quota
        if not _increment_quota():
            raise ValueError("Daily API quota exceeded. Please try again tomorrow.")

        # Format context
        context = self._format_context(chunks)
        history = self._format_conversation_history(conversation_history)

        # Check cache
        cached = _get_cached_response(query, context + history)
        if cached:
            return cached

        # Build prompt
        prompt = f"""{self.system_prompt}

{history}Context from the Physical AI book:
{context}

User question: {query}

Provide a detailed answer based ONLY on the context above. Include source citations."""

        try:
            start_time = time.time()

            # Generate response
            response = self.model.generate_content(prompt)
            answer = response.text

            response_time_ms = int((time.time() - start_time) * 1000)

            # Prepare source references
            sources = [
                {
                    "chunk_id": chunk.get("chunk_id", ""),
                    "file_path": chunk.get("file_path", ""),
                    "content": chunk.get("content", "")[:200] + "...",  # Excerpt
                    "score": chunk.get("score", 0.0),
                }
                for chunk in chunks
            ]

            # Cache response
            _cache_response(query, context + history, answer, sources)

            logger.info(
                f"Generated answer in {response_time_ms}ms for query: {query[:50]}..."
            )

            return answer, sources

        except Exception as e:
            logger.error(f"Error generating answer: {e}", exc_info=True)
            raise ValueError(f"Failed to generate answer: {str(e)}")


# Global RAG service instance
rag_service = RAGService()
