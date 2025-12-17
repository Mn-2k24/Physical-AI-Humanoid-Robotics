"""
Qdrant Client Wrapper
Purpose: Initialize QdrantClient with HNSW config, create collection with 768-dim vectors
Date: 2025-12-13
"""

import logging
from typing import List, Optional

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    FieldCondition,
    Filter,
    HnswConfigDiff,
    MatchValue,
    PayloadSchemaType,
    PointStruct,
    Range,
    VectorParams,
)
from tenacity import retry, stop_after_attempt, wait_exponential

from ..core.config import settings

logger = logging.getLogger(__name__)


class QdrantService:
    """Qdrant vector database service."""

    def __init__(self):
        """Initialize Qdrant client."""
        self.client: Optional[QdrantClient] = None
        self.collection_name = settings.qdrant_collection

    def connect(self):
        """Create Qdrant client connection."""
        try:
            self.client = QdrantClient(
                url=settings.qdrant_endpoint,
                api_key=settings.qdrant_api_key,
                timeout=10,
                prefer_grpc=True,  # Use gRPC for better performance
            )
            logger.info("✓ Qdrant client initialized")
            self._ensure_collection_exists()
        except Exception as e:
            logger.error(f"✗ Failed to connect to Qdrant: {e}")
            raise

    def _ensure_collection_exists(self):
        """Create collection if it doesn't exist."""
        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            collection_names = [col.name for col in collections]

            if self.collection_name not in collection_names:
                # Create collection with 768-dim vectors (text-embedding-004)
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=768, distance=Distance.COSINE),
                )

                # Configure HNSW index
                self.client.update_collection(
                    collection_name=self.collection_name,
                    hnsw_config=HnswConfigDiff(
                        m=16,  # Number of edges per node
                        ef_construct=100,  # Construction-time search depth
                        full_scan_threshold=10000,
                    ),
                )

                # Set payload schema
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="file_path",
                    field_schema=PayloadSchemaType.KEYWORD,
                )
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="chunk_index",
                    field_schema=PayloadSchemaType.INTEGER,
                )

                logger.info(f"✓ Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"✓ Qdrant collection exists: {self.collection_name}")

        except Exception as e:
            logger.error(f"✗ Failed to ensure collection exists: {e}")
            raise

    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=1, max=10))
    def search(
        self,
        query_vector: List[float],
        limit: int = 5,
        file_path: Optional[str] = None,
        chunk_indices: Optional[List[int]] = None,
    ) -> List[dict]:
        """
        Search for similar vectors in Qdrant.

        Args:
            query_vector: Query embedding vector (768-dim)
            limit: Maximum number of results to return
            file_path: Optional file path filter for selected-text queries
            chunk_indices: Optional chunk index range for selected-text queries

        Returns:
            List of search results with payload and score
        """
        if not self.client:
            raise RuntimeError("Qdrant client not initialized. Call connect() first.")

        # Build filter for selected-text queries
        query_filter = None
        if file_path and chunk_indices:
            query_filter = Filter(
                must=[
                    FieldCondition(key="file_path", match=MatchValue(value=file_path)),
                    FieldCondition(
                        key="chunk_index",
                        range=Range(gte=min(chunk_indices), lte=max(chunk_indices)),
                    ),
                ]
            )

        # Search with HNSW parameters using query_points
        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_vector,
            query_filter=query_filter,
            limit=limit,
            search_params={"hnsw_ef": 128},  # Runtime search depth
        ).points

        # Format results
        return [
            {
                "id": result.id,
                "score": result.score,
                "file_path": result.payload.get("file_path"),
                "section_heading": result.payload.get("section_heading"),
                "chunk_index": result.payload.get("chunk_index"),
                "raw_text": result.payload.get("raw_text"),
            }
            for result in results
        ]

    def upsert(self, points: List[PointStruct]):
        """
        Insert or update vectors in Qdrant.

        Args:
            points: List of PointStruct objects with id, vector, and payload
        """
        if not self.client:
            raise RuntimeError("Qdrant client not initialized. Call connect() first.")

        self.client.upsert(collection_name=self.collection_name, points=points)

    def delete_collection(self):
        """Delete the collection (use with caution)."""
        if not self.client:
            raise RuntimeError("Qdrant client not initialized. Call connect() first.")

        self.client.delete_collection(collection_name=self.collection_name)
        logger.info(f"✓ Deleted Qdrant collection: {self.collection_name}")

    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=1, max=10))
    def search_selected_text(
        self,
        query_vector: List[float],
        file_path: str,
        chunk_index_range: tuple[int, int],
        limit: int = 5,
    ) -> List[dict]:
        """
        Search for similar vectors with strict filtering for selected text queries.

        Args:
            query_vector: Query embedding vector (768-dim)
            file_path: File path to filter by
            chunk_index_range: Tuple of (min_index, max_index) for chunk range
            limit: Maximum number of results to return

        Returns:
            List of search results filtered by file path and chunk index range
        """
        if not self.client:
            raise RuntimeError("Qdrant client not initialized. Call connect() first.")

        min_index, max_index = chunk_index_range

        # Build filter for selected text
        query_filter = Filter(
            must=[
                FieldCondition(key="file_path", match=MatchValue(value=file_path)),
                FieldCondition(
                    key="chunk_index",
                    range=Range(gte=min_index, lte=max_index),
                ),
            ]
        )

        # Search with filter using query_points
        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_vector,
            query_filter=query_filter,
            limit=limit,
            search_params={"hnsw_ef": 128},
        ).points

        # Format results
        return [
            {
                "id": result.id,
                "score": result.score,
                "file_path": result.payload.get("file_path"),
                "section_heading": result.payload.get("section_heading"),
                "chunk_index": result.payload.get("chunk_index"),
                "raw_text": result.payload.get("raw_text"),
            }
            for result in results
        ]

    def get_collection_info(self) -> dict:
        """Get collection information including vector count."""
        if not self.client:
            raise RuntimeError("Qdrant client not initialized. Call connect() first.")

        collection_info = self.client.get_collection(collection_name=self.collection_name)
        return {
            "points_count": collection_info.points_count,
            "status": collection_info.status,
        }

    async def health_check(self) -> bool:
        """
        Check Qdrant connection health.

        Returns:
            True if Qdrant is reachable, False otherwise
        """
        try:
            collections = self.client.get_collections()
            return collections is not None
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}")
            return False


# Global Qdrant service instance
qdrant_service = QdrantService()


# Dependency for FastAPI
def get_qdrant():
    """FastAPI dependency for Qdrant access."""
    return qdrant_service
