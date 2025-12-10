"""FastAPI application entry point."""

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .core import Config
from .api.routes import router
from .services.embeddings import get_embedding_model
from .services.generation import get_answer_model
from .services.retrieval import get_qdrant_client
from .services.analytics import get_connection

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan context manager for startup and shutdown events."""
    # Startup
    logger.info("🚀 Starting RAG Backend API...")

    try:
        # Pre-load embedding model
        logger.info("Loading embedding model...")
        get_embedding_model()
        logger.info("✅ Embedding model loaded")

        # Pre-load answer generation model
        logger.info("Loading answer generation model...")
        get_answer_model()
        logger.info("✅ Answer generation model loaded")

        # Connect to Qdrant
        logger.info("Connecting to Qdrant...")
        client = get_qdrant_client()
        collections = client.get_collections()
        logger.info(f"✅ Connected to Qdrant ({len(collections.collections)} collections)")

        # Connect to Neon
        logger.info("Connecting to Neon Postgres...")
        get_connection()
        logger.info("✅ Connected to Neon Postgres")

        logger.info("✨ API ready to serve requests!")

    except Exception as e:
        logger.error(f"❌ Startup failed: {e}")
        raise

    yield

    # Shutdown
    logger.info("Shutting down...")


# Create FastAPI app
app = FastAPI(
    title="Physical AI RAG Backend",
    description="RAG backend API for the Physical AI & Humanoid Robotics book",
    version="1.0.0",
    lifespan=lifespan,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=Config.API_CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes with /api prefix
app.include_router(router, prefix="/api", tags=["RAG"])


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "Physical AI RAG Backend API",
        "version": "1.0.0",
        "docs": "/docs",
    }


# Exception handlers
@app.exception_handler(ValueError)
async def value_error_handler(request, exc):
    """Handle ValueError exceptions."""
    logger.error(f"ValueError: {exc}")
    return {
        "error": "Invalid request",
        "detail": str(exc),
    }


@app.exception_handler(Exception)
async def generic_exception_handler(request, exc):
    """Handle generic exceptions."""
    logger.error(f"Unhandled exception: {exc}")
    return {
        "error": "Internal server error",
        "detail": "An unexpected error occurred. Please try again later.",
    }
