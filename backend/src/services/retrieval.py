"""Retrieval service for querying Qdrant."""

from typing import List, Optional

from qdrant_client import QdrantClient
from qdrant_client.models import ScoredPoint, Filter, FieldCondition, MatchValue

from ..core import Config, TOP_K_RETRIEVAL, TOP_N_RESULTS
from ..models.query import RetrievedChunk

# Initialize Qdrant client (singleton pattern)
_client: Optional[QdrantClient] = None


def get_qdrant_client() -> QdrantClient:
    """Get or initialize the Qdrant client."""
    global _client
    if _client is None:
        _client = QdrantClient(
            url=Config.QDRANT_ENDPOINT,
            api_key=Config.QDRANT_API_KEY,
        )
    return _client


def search_qdrant(
    query_embedding: List[float],
    query_type: str,
    source_file_path: Optional[str] = None
) -> List[ScoredPoint]:
    """Search Qdrant collection for relevant chunks.

    Args:
        query_embedding: The query embedding vector (384 dimensions)
        query_type: 'full_book' or 'local'
        source_file_path: Source file path for local queries (required if query_type='local')

    Returns:
        List of ScoredPoint objects from Qdrant (top-K results)
    """
    client = get_qdrant_client()

    # Build filter for local queries
    query_filter = None
    if query_type == 'local' and source_file_path:
        query_filter = Filter(
            must=[
                FieldCondition(
                    key="file_path",
                    match=MatchValue(value=source_file_path)
                )
            ]
        )

    # Search Qdrant
    results = client.search(
        collection_name=Config.QDRANT_COLLECTION_NAME,
        query_vector=query_embedding,
        limit=TOP_K_RETRIEVAL,
        query_filter=query_filter,
    )

    return results


def rerank_and_select(scored_points: List[ScoredPoint]) -> List[RetrievedChunk]:
    """Rerank scored points by similarity and select top-N.

    Args:
        scored_points: List of ScoredPoint objects from Qdrant

    Returns:
        List of RetrievedChunk objects (top-3, sorted by similarity descending)
    """
    # Sort by score descending (Qdrant already returns sorted, but be explicit)
    sorted_points = sorted(scored_points, key=lambda x: x.score, reverse=True)

    # Take top-N
    top_n = sorted_points[:TOP_N_RESULTS]

    # Convert to RetrievedChunk objects
    retrieved_chunks = []
    for rank, point in enumerate(top_n, start=1):
        chunk = RetrievedChunk(
            chunk_id=str(point.id),
            file_path=point.payload.get("file_path", ""),
            section=point.payload.get("section", ""),
            text=point.payload.get("text", ""),
            similarity_score=float(point.score),
            rank=rank,
        )
        retrieved_chunks.append(chunk)

    return retrieved_chunks
