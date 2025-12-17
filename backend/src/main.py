"""
FastAPI App Initialization
Purpose: App setup, CORS middleware, error handlers, health check endpoint
Date: 2025-12-13
Updated: 2025-12-14 - Added optional Sentry error tracking
"""




import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI, Request, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from slowapi.errors import RateLimitExceeded

from .core.config import settings
from .core.middleware import limiter, log_requests, setup_rate_limiting
from .services.neon import db
from .services.qdrant import qdrant_service

logger = logging.getLogger(__name__)

# ============================================================================
# Optional Sentry Error Tracking
# ============================================================================

# Initialize Sentry if DSN is provided
if settings.sentry_dsn:
    try:
        import sentry_sdk
        from sentry_sdk.integrations.fastapi import FastApiIntegration
        from sentry_sdk.integrations.logging import LoggingIntegration

        sentry_sdk.init(
            dsn=settings.sentry_dsn,
            environment=settings.environment,
            traces_sample_rate=1.0 if settings.environment == "development" else 0.1,
            integrations=[
                FastApiIntegration(transaction_style="endpoint"),
                LoggingIntegration(
                    level=logging.INFO,  # Capture info and above as breadcrumbs
                    event_level=logging.ERROR,  # Send errors as events
                ),
            ],
        )
        logger.info("✓ Sentry error tracking initialized")
    except ImportError:
        logger.warning("Sentry DSN provided but sentry-sdk not installed. Install with: pip install sentry-sdk[fastapi]")
    except Exception as e:
        logger.error(f"Failed to initialize Sentry: {e}")


# ============================================================================
# Lifespan Context Manager
# ============================================================================


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for startup and shutdown events.

    Handles:
    - Database connection initialization
    - Qdrant client initialization
    - Cleanup on shutdown
    """
    # Startup
    logger.info("=" * 60)
    logger.info("Physical AI RAG Chatbot - Starting Up")
    logger.info("=" * 60)

    try:
        # Initialize database connection pool
        await db.connect()

        # Initialize Qdrant client
        qdrant_service.connect()

        logger.info("✓ All services initialized successfully")
        logger.info("=" * 60)

    except Exception as e:
        logger.error(f"✗ Failed to initialize services: {e}")
        raise

    yield

    # Shutdown
    logger.info("=" * 60)
    logger.info("Physical AI RAG Chatbot - Shutting Down")
    logger.info("=" * 60)

    try:
        await db.disconnect()
        logger.info("✓ All services shut down successfully")
    except Exception as e:
        logger.error(f"✗ Error during shutdown: {e}")


# ============================================================================
# FastAPI App
# ============================================================================

app = FastAPI(
    title="Physical AI RAG Chatbot API",
    description="Backend API for RAG chatbot with authentication and conversation history",
    version="1.0.0",
    lifespan=lifespan,
)

# ============================================================================
# Middleware Configuration
# ============================================================================

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Rate limiting
setup_rate_limiting(app)

# Request logging middleware
app.middleware("http")(log_requests)

# ============================================================================
# Error Handlers
# ============================================================================


@app.exception_handler(RateLimitExceeded)
async def rate_limit_handler(request: Request, exc: RateLimitExceeded):
    """Handle rate limit exceeded errors."""
    return JSONResponse(
        status_code=status.HTTP_429_TOO_MANY_REQUESTS,
        content={"error": "Rate limit exceeded", "detail": "Too many requests. Please try again later."},
    )


@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """Handle unhandled exceptions."""
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content={"error": "Internal server error", "detail": str(exc)},
    )


# ============================================================================
# Health Check Endpoint
# ============================================================================


@app.get("/health", tags=["Health"])
async def health_check():
    """
    Health check endpoint.

    Checks:
    - Neon Postgres connection
    - Qdrant connection
    - Gemini API availability

    Returns:
        Health status with individual service checks
    """
    from .services.embedding import embedding_service

    checks = {
        "neon": await db.health_check(),
        "qdrant": await qdrant_service.health_check(),
        "gemini": embedding_service.health_check(),
    }

    status_code = status.HTTP_200_OK if all(checks.values()) else status.HTTP_503_SERVICE_UNAVAILABLE
    overall_status = "healthy" if all(checks.values()) else "degraded"

    return JSONResponse(
        status_code=status_code, content={"status": overall_status, "checks": checks}
    )


# ============================================================================
# API Routes
# ============================================================================

from .api.auth import router as auth_router
from .api.chat import router as chat_router
from .api.conversations import router as conversations_router
from .api.progress import router as progress_router
from .api.recommendations import router as recommendations_router

# Register routers
app.include_router(auth_router)
app.include_router(chat_router)
app.include_router(conversations_router)
app.include_router(progress_router)
app.include_router(recommendations_router)


# ============================================================================
# Root Endpoint
# ============================================================================


@app.get("/", tags=["Root"])
async def root():
    """Root endpoint with API information."""
    return {
        "message": "Physical AI RAG Chatbot API",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/health",
    }
