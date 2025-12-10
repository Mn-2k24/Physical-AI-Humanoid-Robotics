"""API routes for the RAG backend."""

import time
import logging
from typing import List
from uuid import UUID, uuid4

from fastapi import APIRouter, HTTPException, status
from qdrant_client.http.exceptions import UnexpectedResponse

from .schemas import (
    AskRequest,
    AskLocalRequest,
    TrackRequest,
    AskResponse,
    TrackResponse,
    HealthResponse,
    SourceCitationResponse,
)
from ..core import Config, MIN_SIMILARITY_SCORE
from ..models.query import Query, RetrievedChunk
from ..models.answer import ChatbotAnswer, SourceCitation
from ..services.embeddings import generate_query_embedding, get_embedding_model
from ..services.retrieval import search_qdrant, rerank_and_select, get_qdrant_client
from ..services.generation import generate_answer, get_answer_model
from ..services.analytics import log_conversation, get_connection

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/ask", response_model=AskResponse)
async def ask_full_book(request: AskRequest):
    """Handle full-book query.

    Args:
        request: AskRequest with query text

    Returns:
        AskResponse with answer and sources
    """
    start_time = time.time()
    query_id = uuid4()

    try:
        # Generate query embedding
        query_embedding = generate_query_embedding(request.query)

        # Search Qdrant
        retrieval_start = time.time()
        scored_points = search_qdrant(
            query_embedding=query_embedding,
            query_type="full_book",
        )
        retrieval_time = int((time.time() - retrieval_start) * 1000)

        # Check if we have enough results
        if len(scored_points) < 3:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="No relevant book content found for this query"
            )

        # Rerank and select top-3
        retrieved_chunks = rerank_and_select(scored_points)

        # Check similarity threshold
        if all(chunk.similarity_score < MIN_SIMILARITY_SCORE for chunk in retrieved_chunks):
            return AskResponse(
                query_id=str(query_id),
                answer="No relevant book content found for this query",
                sources=[],
                latency_ms=int((time.time() - start_time) * 1000),
            )

        # Generate answer
        generation_start = time.time()
        answer_text = generate_answer(request.query, retrieved_chunks)
        generation_time = int((time.time() - generation_start) * 1000)

        total_latency = int((time.time() - start_time) * 1000)

        # Build response
        sources = [
            SourceCitationResponse(
                file_path=chunk.file_path,
                section=chunk.section,
                similarity_score=chunk.similarity_score,
            )
            for chunk in retrieved_chunks
        ]

        response = AskResponse(
            query_id=str(query_id),
            answer=answer_text,
            sources=sources,
            latency_ms=total_latency,
        )

        return response

    except UnexpectedResponse as e:
        logger.error(f"Qdrant error: {e}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Vector database temporarily unavailable. Please try again."
        )
    except Exception as e:
        logger.error(f"Error in /ask: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred processing your query. Please try again."
        )


@router.post("/ask-local", response_model=AskResponse)
async def ask_local_chapter(request: AskLocalRequest):
    """Handle chapter-scoped query.

    Args:
        request: AskLocalRequest with query, selected text, and source file

    Returns:
        AskResponse with answer and sources (only from same file)
    """
    start_time = time.time()
    query_id = uuid4()

    try:
        # Generate query embedding
        query_embedding = generate_query_embedding(request.query)

        # Search Qdrant with file filter
        retrieval_start = time.time()
        scored_points = search_qdrant(
            query_embedding=query_embedding,
            query_type="local",
            source_file_path=request.source_file_path,
        )
        retrieval_time = int((time.time() - retrieval_start) * 1000)

        # Check if we have enough results
        if len(scored_points) < 3:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"No relevant content found in {request.source_file_path}"
            )

        # Rerank and select top-3
        retrieved_chunks = rerank_and_select(scored_points)

        # Check similarity threshold
        if all(chunk.similarity_score < MIN_SIMILARITY_SCORE for chunk in retrieved_chunks):
            return AskResponse(
                query_id=str(query_id),
                answer=f"No relevant content found in {request.source_file_path} for this query",
                sources=[],
                latency_ms=int((time.time() - start_time) * 1000),
            )

        # Generate answer
        generation_start = time.time()
        answer_text = generate_answer(request.query, retrieved_chunks)
        generation_time = int((time.time() - generation_start) * 1000)

        total_latency = int((time.time() - start_time) * 1000)

        # Build response
        sources = [
            SourceCitationResponse(
                file_path=chunk.file_path,
                section=chunk.section,
                similarity_score=chunk.similarity_score,
            )
            for chunk in retrieved_chunks
        ]

        response = AskResponse(
            query_id=str(query_id),
            answer=answer_text,
            sources=sources,
            latency_ms=total_latency,
        )

        return response

    except UnexpectedResponse as e:
        logger.error(f"Qdrant error: {e}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Vector database temporarily unavailable. Please try again."
        )
    except Exception as e:
        logger.error(f"Error in /ask-local: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred processing your query. Please try again."
        )


@router.post("/track", response_model=TrackResponse)
async def track_conversation(request: TrackRequest):
    """Log conversation analytics.

    Args:
        request: TrackRequest with conversation details

    Returns:
        TrackResponse with conversation ID and status
    """
    try:
        # Convert sources to SourceCitation objects
        sources = [
            SourceCitation(
                file_path=src.get("file_path", ""),
                section=src.get("section", ""),
                chunk_index=src.get("chunk_index", 0),
                similarity_score=src.get("similarity_score", 0.0),
            )
            for src in request.sources
        ]

        # Log to database
        conversation_id = log_conversation(
            query_id=UUID(request.query_id),
            query_text=request.query_text,
            answer_text=request.answer_text,
            sources=sources,
            query_type=request.query_type,
            latency_ms=request.latency_ms,
        )

        if conversation_id:
            return TrackResponse(
                conversation_id=conversation_id,
                status="logged"
            )
        else:
            return TrackResponse(
                conversation_id="",
                status="failed"
            )

    except Exception as e:
        logger.error(f"Error in /track: {e}")
        return TrackResponse(
            conversation_id="",
            status="failed"
        )


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint.

    Returns:
        HealthResponse with service status
    """
    qdrant_ok = False
    neon_ok = False
    models_ok = False

    # Check Qdrant
    try:
        client = get_qdrant_client()
        client.get_collections()
        qdrant_ok = True
    except Exception as e:
        logger.error(f"Qdrant health check failed: {e}")

    # Check Neon
    try:
        conn = get_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT 1")
        cursor.close()
        neon_ok = True
    except Exception as e:
        logger.error(f"Neon health check failed: {e}")

    # Check models
    try:
        get_embedding_model()
        get_answer_model()
        models_ok = True
    except Exception as e:
        logger.error(f"Model health check failed: {e}")

    overall_status = "healthy" if (qdrant_ok and neon_ok and models_ok) else "degraded"

    return HealthResponse(
        status=overall_status,
        qdrant_connected=qdrant_ok,
        neon_connected=neon_ok,
        model_loaded=models_ok,
    )
