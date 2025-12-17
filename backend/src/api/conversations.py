"""
Conversation History API Endpoints
Purpose: /chat/history, /chat/history/{id}, /chat/new endpoints for conversation management
Date: 2025-12-14
"""

import logging

from fastapi import APIRouter, Depends, HTTPException, Query, status

from ..core.middleware import get_current_user
from ..models.conversation import (
    ConversationDetailResponse,
    ConversationListResponse,
    ConversationResponse,
)
from ..services.conversation import conversation_service

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/chat", tags=["Conversations"])


# ============================================================================
# T054: Create New Conversation
# ============================================================================


@router.post("/new", status_code=status.HTTP_201_CREATED)
async def create_new_conversation(
    current_user: dict = Depends(get_current_user),
):
    """
    Create a new conversation.

    **Requires Authentication**

    Returns:
        Conversation ID and creation timestamp

    Raises:
        401: Not authenticated
        500: Internal server error
    """
    try:
        user_id = current_user["user_id"]

        conversation_id = await conversation_service.create_conversation(
            user_id=user_id, title="New Conversation"
        )

        logger.info(f"Created new conversation {conversation_id} for user {user_id}")

        return {
            "conversation_id": conversation_id,
            "message": "Conversation created successfully",
        }

    except Exception as e:
        logger.error(f"Create conversation error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create conversation. Please try again.",
        )


# ============================================================================
# T052: Get Conversation History List
# ============================================================================


@router.get("/history", response_model=ConversationListResponse)
async def get_conversation_history(
    skip: int = Query(0, ge=0, description="Number of records to skip"),
    limit: int = Query(20, ge=1, le=100, description="Maximum number of records"),
    include_archived: bool = Query(
        False, description="Include archived conversations"
    ),
    current_user: dict = Depends(get_current_user),
):
    """
    Get paginated list of user's conversations.

    **Requires Authentication**

    Query Parameters:
        - skip: Number of records to skip (default: 0)
        - limit: Maximum records to return (default: 20, max: 100)
        - include_archived: Include archived conversations (default: False)

    Returns:
        Paginated conversation list with interaction counts

    Raises:
        401: Not authenticated
        500: Internal server error
    """
    try:
        user_id = current_user["user_id"]

        conversations, total = await conversation_service.get_conversations(
            user_id=user_id,
            skip=skip,
            limit=limit,
            include_archived=include_archived,
        )

        # Format response
        conversation_responses = [
            ConversationResponse(**conv) for conv in conversations
        ]

        logger.info(
            f"Retrieved {len(conversations)} conversations for user {user_id}"
        )

        return ConversationListResponse(
            conversations=conversation_responses,
            total=total,
            skip=skip,
            limit=limit,
        )

    except Exception as e:
        logger.error(f"Get conversation history error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve conversation history. Please try again.",
        )


# ============================================================================
# T053: Get Conversation Detail
# ============================================================================


@router.get("/history/{conversation_id}", response_model=ConversationDetailResponse)
async def get_conversation_detail(
    conversation_id: str,
    current_user: dict = Depends(get_current_user),
):
    """
    Get full conversation with all chat interactions.

    **Requires Authentication**

    Path Parameters:
        - conversation_id: Conversation ID (UUID)

    Returns:
        Conversation details with all interactions ordered by timestamp

    Raises:
        401: Not authenticated
        404: Conversation not found or access denied
        500: Internal server error
    """
    try:
        user_id = current_user["user_id"]

        conversation = await conversation_service.get_conversation_detail(
            conversation_id=conversation_id, user_id=user_id
        )

        if not conversation:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Conversation not found or access denied",
            )

        logger.info(f"Retrieved conversation {conversation_id} for user {user_id}")

        return ConversationDetailResponse(**conversation)

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get conversation detail error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve conversation. Please try again.",
        )


# ============================================================================
# Archive Conversation
# ============================================================================


@router.put("/history/{conversation_id}/archive")
async def archive_conversation(
    conversation_id: str,
    current_user: dict = Depends(get_current_user),
):
    """
    Archive a conversation.

    **Requires Authentication**

    Path Parameters:
        - conversation_id: Conversation ID (UUID)

    Returns:
        Success message

    Raises:
        401: Not authenticated
        404: Conversation not found or access denied
        500: Internal server error
    """
    try:
        user_id = current_user["user_id"]

        success = await conversation_service.archive_conversation(
            conversation_id=conversation_id, user_id=user_id
        )

        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Conversation not found or access denied",
            )

        logger.info(f"Archived conversation {conversation_id} for user {user_id}")

        return {"message": "Conversation archived successfully"}

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Archive conversation error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to archive conversation. Please try again.",
        )


# ============================================================================
# Delete Conversation
# ============================================================================


@router.delete("/history/{conversation_id}")
async def delete_conversation(
    conversation_id: str,
    current_user: dict = Depends(get_current_user),
):
    """
    Delete a conversation and all its interactions.

    **Requires Authentication**

    Path Parameters:
        - conversation_id: Conversation ID (UUID)

    Returns:
        Success message

    Raises:
        401: Not authenticated
        404: Conversation not found or access denied
        500: Internal server error
    """
    try:
        user_id = current_user["user_id"]

        success = await conversation_service.delete_conversation(
            conversation_id=conversation_id, user_id=user_id
        )

        if not success:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Conversation not found or access denied",
            )

        logger.info(f"Deleted conversation {conversation_id} for user {user_id}")

        return {"message": "Conversation deleted successfully"}

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Delete conversation error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to delete conversation. Please try again.",
        )
