"""Analytics service for logging conversations to Neon Postgres."""

import json
import logging
from typing import List, Optional
from uuid import UUID

import psycopg2
from psycopg2 import sql

from ..core import Config
from ..models.answer import SourceCitation

logger = logging.getLogger(__name__)

# Connection pool (simple singleton pattern)
_connection: Optional[psycopg2.extensions.connection] = None


def get_connection():
    """Get or create a database connection."""
    global _connection

    if _connection is None or _connection.closed:
        try:
            _connection = psycopg2.connect(Config.NEON_CONNECTION_STRING)
        except Exception as e:
            logger.error(f"Failed to connect to Neon Postgres: {e}")
            raise

    return _connection


def log_conversation(
    query_id: UUID,
    query_text: str,
    answer_text: str,
    sources: List[SourceCitation],
    query_type: str,
    latency_ms: int,
    user_session_id: Optional[UUID] = None
) -> Optional[str]:
    """Log a conversation to the Neon Postgres database.

    Args:
        query_id: Query identifier
        query_text: The user's query
        answer_text: The generated answer
        sources: List of source citations
        query_type: 'full_book' or 'local'
        latency_ms: Total latency in milliseconds
        user_session_id: Optional user session identifier

    Returns:
        Conversation ID if successful, None if failed
    """
    try:
        conn = get_connection()
        cursor = conn.cursor()

        # Convert sources to JSON
        sources_json = json.dumps([
            {
                "file_path": source.file_path,
                "section": source.section,
                "chunk_index": source.chunk_index,
                "similarity_score": source.similarity_score,
            }
            for source in sources
        ])

        # Insert conversation
        insert_query = """
        INSERT INTO conversations (
            query_id, query_text, answer_text, sources, query_type, latency_ms, user_session_id
        )
        VALUES (%s, %s, %s, %s, %s, %s, %s)
        RETURNING conversation_id;
        """

        cursor.execute(
            insert_query,
            (
                str(query_id),
                query_text,
                answer_text,
                sources_json,
                query_type,
                latency_ms,
                str(user_session_id) if user_session_id else None,
            )
        )

        conversation_id = cursor.fetchone()[0]
        conn.commit()
        cursor.close()

        logger.info(f"Conversation logged: {conversation_id}")
        return str(conversation_id)

    except Exception as e:
        logger.error(f"Failed to log conversation: {e}")
        # Don't raise - analytics should not block the main response
        return None
