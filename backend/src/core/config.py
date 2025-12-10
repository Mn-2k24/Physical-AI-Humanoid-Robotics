"""Configuration management for the RAG backend.

Loads environment variables and validates required settings.
"""

import os
from typing import List
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()


class Config:
    """Application configuration loaded from environment variables."""

    # Qdrant Configuration
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_ENDPOINT: str = os.getenv("QDRANT_ENDPOINT", "")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "physical-ai-book")

    # Neon Postgres Configuration
    NEON_CONNECTION_STRING: str = os.getenv("NEON_CONNECTION_STRING", "")

    # Model Configuration
    EMBEDDING_MODEL_NAME: str = os.getenv(
        "EMBEDDING_MODEL_NAME",
        "sentence-transformers/all-MiniLM-L6-v2"
    )
    ANSWER_MODEL_NAME: str = os.getenv("ANSWER_MODEL_NAME", "google/flan-t5-base")

    # API Configuration
    API_CORS_ORIGINS: List[str] = os.getenv(
        "API_CORS_ORIGINS",
        "http://localhost:3000"
    ).split(",")

    @classmethod
    def validate(cls) -> None:
        """Validate that all required environment variables are set.

        Raises:
            ValueError: If any required environment variable is missing.
        """
        required_vars = {
            "QDRANT_API_KEY": cls.QDRANT_API_KEY,
            "QDRANT_ENDPOINT": cls.QDRANT_ENDPOINT,
            "QDRANT_COLLECTION_NAME": cls.QDRANT_COLLECTION_NAME,
            "NEON_CONNECTION_STRING": cls.NEON_CONNECTION_STRING,
            "EMBEDDING_MODEL_NAME": cls.EMBEDDING_MODEL_NAME,
            "ANSWER_MODEL_NAME": cls.ANSWER_MODEL_NAME,
        }

        missing = [key for key, value in required_vars.items() if not value]

        if missing:
            raise ValueError(
                f"Missing required environment variables: {', '.join(missing)}. "
                "Please check your .env file."
            )


# Validate configuration on import
Config.validate()
