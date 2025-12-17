"""
Conversation Service
Purpose: CRUD operations for conversations and chat interactions using Neon Postgres
Date: 2025-12-14
"""

import logging
import uuid
from typing import List, Optional

from ..models.conversation import (
    ChatInteractionCreate,
    ConversationCreate,
)
from ..services.neon import db

logger = logging.getLogger(__name__)


class ConversationService:
    """Conversation service for managing conversations and chat interactions."""

    async def create_conversation(
        self, user_id: str, title: str = "New Conversation"
    ) -> str:
        """
        Create a new conversation.

        Args:
            user_id: User ID
            title: Conversation title (default: "New Conversation")

        Returns:
            conversation_id: Created conversation ID
        """
        conversation_id = str(uuid.uuid4())

        await db.execute_write(
            """
            INSERT INTO conversations (id, user_id, title)
            VALUES (%s, %s, %s)
            """,
            (conversation_id, user_id, title),
        )

        logger.info(f"Created conversation {conversation_id} for user {user_id}")
        return conversation_id

    async def get_conversations(
        self,
        user_id: str,
        skip: int = 0,
        limit: int = 20,
        include_archived: bool = False,
    ) -> tuple[List[dict], int]:
        """
        Get paginated list of user's conversations.

        Args:
            user_id: User ID
            skip: Number of records to skip
            limit: Maximum number of records to return
            include_archived: Include archived conversations (default: False)

        Returns:
            Tuple of (conversations, total_count)
        """
        # Build query
        archived_filter = "" if include_archived else "AND archived = FALSE"

        # Get conversations with interaction count
        conversations = await db.execute_query(
            f"""
            SELECT
                c.id,
                c.user_id,
                c.title,
                c.archived,
                c.created_at,
                c.updated_at,
                COUNT(ci.id) as interaction_count
            FROM conversations c
            LEFT JOIN chat_interactions ci ON c.id = ci.conversation_id
            WHERE c.user_id = %s {archived_filter}
            GROUP BY c.id
            ORDER BY c.updated_at DESC
            LIMIT %s OFFSET %s
            """,
            (user_id, limit, skip),
        )

        # Get total count
        count_result = await db.execute_one(
            f"""
            SELECT COUNT(*) FROM conversations
            WHERE user_id = %s {archived_filter}
            """,
            (user_id,),
        )
        total = count_result[0] if count_result else 0

        # Format results
        conversation_list = []
        for row in conversations:
            conversation_list.append(
                {
                    "conversation_id": row[0],
                    "user_id": row[1],
                    "title": row[2],
                    "archived": row[3],
                    "created_at": row[4],
                    "updated_at": row[5],
                    "interaction_count": row[6],
                }
            )

        logger.info(
            f"Retrieved {len(conversation_list)} conversations for user {user_id}"
        )
        return conversation_list, total

    async def get_conversation_detail(
        self, conversation_id: str, user_id: str
    ) -> Optional[dict]:
        """
        Get conversation with all interactions.

        Args:
            conversation_id: Conversation ID
            user_id: User ID (for authorization)

        Returns:
            Conversation dict with interactions, or None if not found
        """
        # Get conversation
        conversation = await db.execute_one(
            """
            SELECT id, user_id, title, archived, created_at, updated_at
            FROM conversations
            WHERE id = %s AND user_id = %s
            """,
            (conversation_id, user_id),
        )

        if not conversation:
            return None

        # Get all interactions
        interactions = await db.execute_query(
            """
            SELECT
                id as interaction_id,
                conversation_id,
                query_text as user_query,
                answer_text as bot_response,
                source_chunks,
                query_mode,
                created_at
            FROM chat_interactions
            WHERE conversation_id = %s
            ORDER BY created_at ASC
            """,
            (conversation_id,),
        )

        # Format interactions
        import json

        interaction_list = []
        for row in interactions:
            interaction_list.append(
                {
                    "interaction_id": row[0],
                    "conversation_id": row[1],
                    "user_query": row[2],
                    "bot_response": row[3],
                    "source_chunks": row[4] if row[4] else None,  # Already parsed by psycopg3
                    "query_mode": row[5],
                    "created_at": row[6],
                }
            )

        # Build response
        conversation_detail = {
            "conversation_id": conversation[0],
            "user_id": conversation[1],
            "title": conversation[2],
            "archived": conversation[3],
            "created_at": conversation[4],
            "updated_at": conversation[5],
            "interactions": interaction_list,
        }

        logger.info(
            f"Retrieved conversation {conversation_id} with {len(interaction_list)} interactions"
        )
        return conversation_detail

    async def create_interaction(
        self, interaction_data: ChatInteractionCreate
    ) -> str:
        """
        Create a new chat interaction.

        Args:
            interaction_data: Chat interaction data

        Returns:
            interaction_id: Created interaction ID
        """
        import json

        interaction_id = str(uuid.uuid4())

        # Serialize source_chunks to JSON
        source_chunks_json = (
            json.dumps(interaction_data.source_chunks)
            if interaction_data.source_chunks
            else None
        )

        await db.execute_write(
            """
            INSERT INTO chat_interactions (
                id,
                conversation_id,
                user_id,
                query_text,
                answer_text,
                source_chunks,
                query_mode
            ) VALUES (%s, %s, %s, %s, %s, %s, %s)
            """,
            (
                interaction_id,
                interaction_data.conversation_id,
                # TODO: Extract user_id from conversation or pass it in
                (await db.execute_one("SELECT user_id FROM conversations WHERE id = %s", (interaction_data.conversation_id,)))[0],
                interaction_data.user_query,
                interaction_data.bot_response,
                source_chunks_json,
                interaction_data.query_mode,
            ),
        )

        # Update conversation updated_at
        await db.execute_write(
            """
            UPDATE conversations
            SET updated_at = CURRENT_TIMESTAMP
            WHERE id = %s
            """,
            (interaction_data.conversation_id,),
        )

        # Auto-update conversation title from first interaction
        await self._update_conversation_title_if_needed(
            interaction_data.conversation_id, interaction_data.user_query
        )

        logger.info(f"Created interaction {interaction_id}")
        return interaction_id

    async def _update_conversation_title_if_needed(
        self, conversation_id: str, user_query: str
    ):
        """Update conversation title from first query if still default."""
        conversation = await db.execute_one(
            "SELECT title FROM conversations WHERE id = %s",
            (conversation_id,),
        )

        if conversation and conversation[0] == "New Conversation":
            # Generate title from first 50 chars of query
            title = user_query[:50] + ("..." if len(user_query) > 50 else "")

            await db.execute_write(
                "UPDATE conversations SET title = %s WHERE id = %s",
                (title, conversation_id),
            )

    async def archive_conversation(self, conversation_id: str, user_id: str) -> bool:
        """
        Archive a conversation.

        Args:
            conversation_id: Conversation ID
            user_id: User ID (for authorization)

        Returns:
            True if archived, False if not found
        """
        result = await db.execute_write(
            """
            UPDATE conversations
            SET archived = TRUE, updated_at = CURRENT_TIMESTAMP
            WHERE id = %s AND user_id = %s
            """,
            (conversation_id, user_id),
        )

        if result:
            logger.info(f"Archived conversation {conversation_id}")
            return True

        return False

    async def delete_conversation(self, conversation_id: str, user_id: str) -> bool:
        """
        Delete a conversation and all its interactions.

        Args:
            conversation_id: Conversation ID
            user_id: User ID (for authorization)

        Returns:
            True if deleted, False if not found
        """
        # Delete interactions first (CASCADE should handle this, but explicit is safer)
        await db.execute_write(
            "DELETE FROM chat_interactions WHERE conversation_id = %s",
            (conversation_id,),
        )

        # Delete conversation
        result = await db.execute_write(
            "DELETE FROM conversations WHERE id = %s AND user_id = %s",
            (conversation_id, user_id),
        )

        if result:
            logger.info(f"Deleted conversation {conversation_id}")
            return True

        return False

    async def get_conversation_history_for_context(
        self, conversation_id: str, limit: int = 5
    ) -> List[dict]:
        """
        Get recent conversation history for context in RAG.

        Args:
            conversation_id: Conversation ID
            limit: Number of recent interactions to retrieve (default: 5)

        Returns:
            List of recent interactions
        """
        interactions = await db.execute_query(
            """
            SELECT query_text as user_query, answer_text as bot_response
            FROM chat_interactions
            WHERE conversation_id = %s
            ORDER BY created_at DESC
            LIMIT %s
            """,
            (conversation_id, limit),
        )

        # Format and reverse (oldest first for context)
        history = [
            {"user_query": row[0], "bot_response": row[1]} for row in interactions
        ]
        history.reverse()

        return history


# Global conversation service instance
conversation_service = ConversationService()
