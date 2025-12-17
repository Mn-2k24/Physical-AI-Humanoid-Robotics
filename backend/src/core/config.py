"""
Configuration Management
Purpose: Load and validate environment variables using pydantic-settings
Date: 2025-12-13
"""

import logging
from typing import List

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8", extra="allow")

    # ============================================================================
    # Gemini API Configuration
    # ============================================================================
    gemini_api_key: str = Field(..., description="Gemini API key for embeddings and LLM")

    # ============================================================================
    # Qdrant Cloud Configuration
    # ============================================================================
    qdrant_api_key: str = Field(..., description="Qdrant Cloud API key")
    qdrant_endpoint: str = Field(..., description="Qdrant Cloud endpoint URL")
    qdrant_collection: str = Field(default="physical_ai_book", description="Qdrant collection name")

    # ============================================================================
    # Neon Serverless Postgres Configuration
    # ============================================================================
    neon_connection_string: str = Field(..., description="Neon Postgres connection string")

    # ============================================================================
    # Better Auth Configuration
    # ============================================================================
    better_auth_secret: str = Field(..., description="Better Auth secret key (min 32 characters)")
    better_auth_url: str = Field(default="http://localhost:8000/auth", description="Better Auth URL")

    # ============================================================================
    # Backend Configuration
    # ============================================================================
    backend_host: str = Field(default="0.0.0.0", description="Backend server host")
    backend_port: int = Field(default=8000, description="Backend server port")
    frontend_url: str = Field(default="http://localhost:3000", description="Frontend URL for CORS")

    # ============================================================================
    # CORS Configuration
    # ============================================================================
    cors_origins: str = Field(
        default="http://localhost:3000",
        description="Comma-separated list of allowed CORS origins",
    )

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins string into list."""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    # ============================================================================
    # Security Configuration
    # ============================================================================
    rate_limit_per_minute: int = Field(default=5, description="Rate limit per minute for auth endpoints")
    session_expiration_hours: int = Field(default=24, description="Session expiration time in hours")
    csrf_secret: str = Field(..., description="CSRF secret key (min 32 characters)")

    # ============================================================================
    # Application Configuration
    # ============================================================================
    environment: str = Field(default="development", description="Environment (development/production)")
    log_level: str = Field(default="INFO", description="Logging level")
    max_conversation_messages: int = Field(default=10, description="Max messages in conversation context")
    recommendation_limit: int = Field(default=5, description="Max recommendations to generate")

    # ============================================================================
    # Error Tracking (Optional)
    # ============================================================================
    sentry_dsn: str | None = Field(default=None, description="Sentry DSN for error tracking (optional)")

    def validate_secrets(self) -> None:
        """Validate that secrets meet minimum length requirements."""
        if len(self.better_auth_secret) < 32:
            raise ValueError("BETTER_AUTH_SECRET must be at least 32 characters")
        if len(self.csrf_secret) < 32:
            raise ValueError("CSRF_SECRET must be at least 32 characters")

    def configure_logging(self) -> None:
        """Configure structured logging based on settings."""
        logging.basicConfig(
            level=getattr(logging, self.log_level.upper()),
            format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            handlers=[
                logging.StreamHandler(),
                logging.FileHandler("backend/logs/app.log") if self.environment == "production" else logging.NullHandler(),
            ],
        )


# Global settings instance
settings = Settings()

# Validate secrets on startup
settings.validate_secrets()

# Configure logging
settings.configure_logging()
