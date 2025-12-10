"""Constants for the RAG backend."""

# Chunking parameters
MAX_CHUNK_SIZE_TOKENS = 1000
MIN_CHUNK_SIZE_TOKENS = 300
CHUNK_OVERLAP_TOKENS = 50

# Retrieval parameters
TOP_K_RETRIEVAL = 10
TOP_N_RESULTS = 3

# Model dimensions
EMBEDDING_DIMENSION = 384  # all-MiniLM-L6-v2

# Similarity threshold
MIN_SIMILARITY_SCORE = 0.5

# Latency targets (milliseconds)
TARGET_LATENCY_MS = 2000
