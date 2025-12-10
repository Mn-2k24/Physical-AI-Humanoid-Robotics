"""Embedding generation service using sentence-transformers."""

from typing import List
from sentence_transformers import SentenceTransformer

from ..core import Config, EMBEDDING_DIMENSION
from ..models.chunk import TextChunk

# Initialize the embedding model (loaded once at module import)
_model: SentenceTransformer | None = None


def get_embedding_model() -> SentenceTransformer:
    """Get or initialize the embedding model (singleton pattern)."""
    global _model
    if _model is None:
        _model = SentenceTransformer(Config.EMBEDDING_MODEL_NAME)
    return _model


def generate_embeddings(chunks: List[TextChunk]) -> List[List[float]]:
    """Generate embeddings for a list of text chunks.

    Args:
        chunks: List of TextChunk objects

    Returns:
        List of embedding vectors (384 dimensions each)
    """
    if not chunks:
        return []

    model = get_embedding_model()

    # Extract text from chunks
    texts = [chunk.text for chunk in chunks]

    # Generate embeddings (batch processing)
    embeddings = model.encode(texts, show_progress_bar=False, convert_to_numpy=True)

    # Convert numpy arrays to lists
    embedding_list = [emb.tolist() for emb in embeddings]

    # Validate dimensions
    for emb in embedding_list:
        if len(emb) != EMBEDDING_DIMENSION:
            raise ValueError(
                f"Expected embedding dimension {EMBEDDING_DIMENSION}, got {len(emb)}"
            )

    return embedding_list


def generate_query_embedding(query_text: str) -> List[float]:
    """Generate embedding for a single query string.

    Args:
        query_text: The query text

    Returns:
        Embedding vector (384 dimensions)
    """
    model = get_embedding_model()
    embedding = model.encode(query_text, show_progress_bar=False, convert_to_numpy=True)
    return embedding.tolist()
