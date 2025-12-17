"""
Chat API Endpoints
Purpose: /chat/global and /chat/local endpoints for RAG chatbot
Date: 2025-12-14
"""

import logging
import time

from fastapi import APIRouter, Depends, HTTPException, Request, status

from ..core.middleware import get_current_user, limiter
from ..models.conversation import (
    ChatInteractionCreate,
    ChatRequest,
    ChatResponse,
    LocalChatRequest,
    SourceReference,
)
from ..services.conversation import conversation_service
from ..services.rag import rag_service

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/chat", tags=["Chat"])


# ============================================================================
# T051: Global Chat Endpoint
# ============================================================================


@router.post("/global", response_model=ChatResponse)
@limiter.limit("20/minute")
async def chat_global(
    request: Request,
    chat_request: ChatRequest,
    current_user: dict = Depends(get_current_user),
):
    """
    Ask questions about book content with conversation history context.

    **Rate Limited**: 20 requests per minute per user

    **Requires Authentication**

    Args:
        chat_request: Query and optional conversation_id for multi-turn context

    Returns:
        Chat response with grounded answer and source references

    Raises:
        400: Invalid request or quota exceeded
        401: Not authenticated
        429: Rate limit exceeded
        500: Internal server error
    """
    try:
        start_time = time.time()
        user_id = current_user["user_id"]

        # Create new conversation if not provided
        conversation_id = chat_request.conversation_id
        if not conversation_id:
            conversation_id = await conversation_service.create_conversation(
                user_id=user_id, title="New Conversation"
            )
            logger.info(f"Created new conversation: {conversation_id}")

        # Verify conversation belongs to user
        conversation = await conversation_service.get_conversation_detail(
            conversation_id, user_id
        )
        if not conversation:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Conversation not found or access denied",
            )

        # Get conversation history for context (last 5 turns)
        conversation_history = (
            await conversation_service.get_conversation_history_for_context(
                conversation_id, limit=5
            )
        )

        # Retrieve relevant chunks from Qdrant
        chunks = await rag_service.retrieve_chunks(
            query=chat_request.query, top_k=5
        )

        if not chunks:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="No relevant content found in the book. Please try rephrasing your question.",
            )

        # Generate grounded answer using Gemini
        try:
            answer, sources = await rag_service.generate_answer(
                query=chat_request.query,
                chunks=chunks,
                conversation_history=conversation_history,
            )
        except ValueError as e:
            if "quota" in str(e).lower():
                raise HTTPException(
                    status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                    detail="Daily API quota exceeded. Please try again tomorrow.",
                )
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST, detail=str(e)
            )

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        # Store interaction in database
        source_chunk_ids = [s["chunk_id"] for s in sources]
        interaction_data = ChatInteractionCreate(
            conversation_id=conversation_id,
            user_query=chat_request.query,
            bot_response=answer,
            source_chunks=source_chunk_ids,
            query_mode="global",
            response_time_ms=response_time_ms,
        )

        interaction_id = await conversation_service.create_interaction(
            interaction_data
        )

        # Format source references
        source_references = [
            SourceReference(
                chunk_id=s["chunk_id"],
                file_path=s["file_path"],
                content=s["content"],
                score=s["score"],
            )
            for s in sources
        ]

        logger.info(
            f"Global chat response generated in {response_time_ms}ms for user {user_id}"
        )

        return ChatResponse(
            answer=answer,
            sources=source_references,
            conversation_id=conversation_id,
            interaction_id=interaction_id,
            query_mode="global",
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Chat global error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to process chat request. Please try again.",
        )


# ============================================================================
# T058: Local Chat Endpoint (User Story 3)
# ============================================================================


@router.post("/local", response_model=ChatResponse)
@limiter.limit("20/minute")
async def chat_local(
    request: Request,
    chat_request: LocalChatRequest,
    current_user: dict = Depends(get_current_user),
):
    """
    Ask questions about selected text with strict context isolation.

    **Rate Limited**: 20 requests per minute per user

    **Requires Authentication**

    Args:
        chat_request: Query with selected text, file path, and chunk indices

    Returns:
        Chat response with answer based ONLY on selected text

    Raises:
        400: Invalid request (selected text too short)
        401: Not authenticated
        429: Rate limit exceeded
        500: Internal server error
    """
    try:
        start_time = time.time()
        user_id = current_user["user_id"]

        # Validate selected text length (minimum 50 characters)
        if len(chat_request.selected_text) < 50:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Selected text is too short. Please select at least 50 characters for meaningful context.",
            )

        # Create new conversation for local query
        conversation_id = await conversation_service.create_conversation(
            user_id=user_id, title=f"Selected text: {chat_request.query[:40]}..."
        )

        # Use selected text directly as context (no Qdrant search)
        # Format as a single chunk for consistency
        chunks = [
            {
                "chunk_id": "selected_text",
                "content": chat_request.selected_text,
                "file_path": chat_request.file_path,
                "chunk_index": 0,
                "score": 1.0,
            }
        ]

        # Generate answer using ONLY selected text
        try:
            answer, sources = await rag_service.generate_answer(
                query=chat_request.query,
                chunks=chunks,
                conversation_history=None,  # No conversation history for local queries
            )
        except ValueError as e:
            if "quota" in str(e).lower():
                raise HTTPException(
                    status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                    detail="Daily API quota exceeded. Please try again tomorrow.",
                )
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST, detail=str(e)
            )

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        # Store interaction
        interaction_data = ChatInteractionCreate(
            conversation_id=conversation_id,
            user_query=chat_request.query,
            bot_response=answer,
            source_chunks=["selected_text"],
            query_mode="local",
            response_time_ms=response_time_ms,
        )

        interaction_id = await conversation_service.create_interaction(
            interaction_data
        )

        # Format source references
        source_references = [
            SourceReference(
                chunk_id="selected_text",
                file_path=chat_request.file_path,
                content=chat_request.selected_text[:200] + "...",
                score=1.0,
            )
        ]

        logger.info(
            f"Local chat response generated in {response_time_ms}ms for user {user_id}"
        )

        return ChatResponse(
            answer=answer,
            sources=source_references,
            conversation_id=conversation_id,
            interaction_id=interaction_id,
            query_mode="local",
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Chat local error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to process local chat request. Please try again.",
        )
