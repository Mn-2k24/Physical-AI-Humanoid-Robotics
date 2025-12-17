"""
Gemini Embedding Service
Purpose: text-embedding-004 with task_type="retrieval_document", batch processing support
Date: 2025-12-13
Updated: 2025-12-14 - Added API quota monitoring
"""

import asyncio
import logging
from datetime import datetime, timedelta
from typing import List

import google.generativeai as genai

from ..core.config import settings

logger = logging.getLogger(__name__)

# Configure Gemini API
genai.configure(api_key=settings.gemini_api_key)


class GeminiEmbeddingService:
    """Gemini embedding service for text-embedding-004 with quota monitoring."""

    def __init__(self):
        """Initialize Gemini embedding service."""
        self.model_name = "models/text-embedding-004"
        self.embedding_dim = 768

        # Quota monitoring
        self.daily_request_count = 0
        self.daily_quota_limit = 1500  # Gemini API free tier daily limit
        self.quota_reset_time = datetime.now() + timedelta(days=1)
        self.quota_warned_80 = False
        self.quota_warned_90 = False
        self.quota_warned_95 = False

    def _check_and_reset_quota(self) -> None:
        """Check if quota needs to be reset (daily)."""
        if datetime.now() >= self.quota_reset_time:
            self.daily_request_count = 0
            self.quota_reset_time = datetime.now() + timedelta(days=1)
            self.quota_warned_80 = False
            self.quota_warned_90 = False
            self.quota_warned_95 = False
            logger.info("Gemini API quota reset for new day")

    def _track_request(self, count: int = 1) -> None:
        """
        Track API request and log warnings when approaching quota.

        Args:
            count: Number of requests made (default: 1)
        """
        self._check_and_reset_quota()
        self.daily_request_count += count

        # Calculate usage percentage
        usage_pct = (self.daily_request_count / self.daily_quota_limit) * 100

        # Log warnings at thresholds
        if usage_pct >= 95 and not self.quota_warned_95:
            logger.warning(
                f"🔴 CRITICAL: Gemini API quota at {usage_pct:.1f}% "
                f"({self.daily_request_count}/{self.daily_quota_limit} requests used)"
            )
            self.quota_warned_95 = True
        elif usage_pct >= 90 and not self.quota_warned_90:
            logger.warning(
                f"🟠 WARNING: Gemini API quota at {usage_pct:.1f}% "
                f"({self.daily_request_count}/{self.daily_quota_limit} requests used)"
            )
            self.quota_warned_90 = True
        elif usage_pct >= 80 and not self.quota_warned_80:
            logger.info(
                f"🟡 NOTICE: Gemini API quota at {usage_pct:.1f}% "
                f"({self.daily_request_count}/{self.daily_quota_limit} requests used)"
            )
            self.quota_warned_80 = True

        # Log every 100 requests
        if self.daily_request_count % 100 == 0:
            logger.info(
                f"Gemini API usage: {self.daily_request_count}/{self.daily_quota_limit} "
                f"requests ({usage_pct:.1f}%)"
            )

    def get_quota_status(self) -> dict:
        """
        Get current quota usage status.

        Returns:
            Dictionary with quota information
        """
        self._check_and_reset_quota()
        usage_pct = (self.daily_request_count / self.daily_quota_limit) * 100
        return {
            "requests_used": self.daily_request_count,
            "quota_limit": self.daily_quota_limit,
            "usage_percentage": round(usage_pct, 2),
            "requests_remaining": self.daily_quota_limit - self.daily_request_count,
            "reset_time": self.quota_reset_time.isoformat(),
        }

    def generate_embedding(self, text: str, task_type: str = "retrieval_document") -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to embed
            task_type: Task type for embedding optimization
                      - "retrieval_document": For documents to be stored in vector DB
                      - "retrieval_query": For search queries

        Returns:
            Embedding vector (768-dim)
        """
        try:
            result = genai.embed_content(
                model=self.model_name, content=text, task_type=task_type
            )
            self._track_request(1)  # Track successful request
            return result["embedding"]
        except Exception as e:
            logger.error(f"Failed to generate embedding: {e}")
            raise

    def generate_embeddings_batch(
        self, texts: List[str], task_type: str = "retrieval_document", batch_size: int = 100
    ) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in batches.

        Args:
            texts: List of texts to embed
            task_type: Task type for embedding optimization
            batch_size: Number of texts to embed per API call (max 2048)

        Returns:
            List of embedding vectors
        """
        embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i : i + batch_size]
            try:
                # Batch embed multiple documents
                result = genai.embed_content(
                    model=self.model_name, content=batch, task_type=task_type
                )

                # Extract embeddings from batch result
                if isinstance(result, dict) and "embedding" in result:
                    embedding = result["embedding"]
                    # Check if it's a list of embeddings (batch) or single embedding
                    if isinstance(embedding, list) and len(embedding) > 0:
                        if isinstance(embedding[0], list):
                            # Batch of embeddings - extend with all embeddings
                            embeddings.extend(embedding)
                            self._track_request(len(embedding))
                        else:
                            # Single embedding vector
                            embeddings.append(embedding)
                            self._track_request(1)
                    else:
                        # Empty or invalid result
                        logger.warning("Received empty embedding result")
                elif isinstance(result, dict) and "embeddings" in result:
                    # Multiple embeddings result
                    embeddings.extend([emb["values"] for emb in result["embeddings"]])
                    self._track_request(len(batch))  # Track batch requests
                else:
                    # Handle different response format
                    # Note: generate_embedding already tracks quota
                    batch_embeddings = [
                        self.generate_embedding(text, task_type) for text in batch
                    ]
                    embeddings.extend(batch_embeddings)

                logger.info(f"✓ Generated {len(batch)} embeddings (batch {i // batch_size + 1})")

            except Exception as e:
                logger.error(f"Failed to generate batch embeddings: {e}")
                # Fallback: Generate embeddings one by one
                for text in batch:
                    embedding = self.generate_embedding(text, task_type)
                    embeddings.append(embedding)

        return embeddings

    async def generate_embeddings_async(
        self, texts: List[str], task_type: str = "retrieval_document", batch_size: int = 100
    ) -> List[List[float]]:
        """
        Async version of batch embedding generation.

        Args:
            texts: List of texts to embed
            task_type: Task type for embedding optimization
            batch_size: Number of texts to embed per API call

        Returns:
            List of embedding vectors
        """
        # Run synchronous batch processing in executor to avoid blocking
        loop = asyncio.get_event_loop()
        embeddings = await loop.run_in_executor(
            None, self.generate_embeddings_batch, texts, task_type, batch_size
        )
        return embeddings

    def health_check(self) -> bool:
        """
        Check Gemini API health by generating a test embedding.

        Returns:
            True if API is accessible, False otherwise
        """
        try:
            test_embedding = self.generate_embedding("health check test")
            return len(test_embedding) == self.embedding_dim
        except Exception as e:
            logger.error(f"Gemini API health check failed: {e}")
            return False


# Global embedding service instance
embedding_service = GeminiEmbeddingService()


# Dependency for FastAPI
def get_embedding_service():
    """FastAPI dependency for embedding service access."""
    return embedding_service
