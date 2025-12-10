"""Core configuration and constants."""

from .config import Config
from .constants import (
    MAX_CHUNK_SIZE_TOKENS,
    MIN_CHUNK_SIZE_TOKENS,
    CHUNK_OVERLAP_TOKENS,
    TOP_K_RETRIEVAL,
    TOP_N_RESULTS,
    EMBEDDING_DIMENSION,
    MIN_SIMILARITY_SCORE,
    TARGET_LATENCY_MS,
)

__all__ = [
    "Config",
    "MAX_CHUNK_SIZE_TOKENS",
    "MIN_CHUNK_SIZE_TOKENS",
    "CHUNK_OVERLAP_TOKENS",
    "TOP_K_RETRIEVAL",
    "TOP_N_RESULTS",
    "EMBEDDING_DIMENSION",
    "MIN_SIMILARITY_SCORE",
    "TARGET_LATENCY_MS",
]
