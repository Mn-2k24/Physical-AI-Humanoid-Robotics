#!/usr/bin/env python3
"""
Archive Old Conversations
Purpose: Archive conversations older than 30 days
Date: 2025-12-14

Usage:
    python backend/scripts/archive_conversations.py

Schedule with cron (daily at 2 AM):
    0 2 * * * cd /path/to/project && python backend/scripts/archive_conversations.py
"""

import asyncio
import logging
import sys
from datetime import datetime, timedelta
from pathlib import Path

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent.parent))

from backend.src.services.neon import db

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

ARCHIVE_THRESHOLD_DAYS = 30


async def archive_old_conversations():
    """
    Archive conversations older than 30 days.

    Sets archived=TRUE for conversations where:
    - created_at < NOW() - 30 days
    - archived = FALSE
    """
    try:
        # Connect to database
        await db.connect()

        # Calculate threshold date
        threshold_date = datetime.utcnow() - timedelta(days=ARCHIVE_THRESHOLD_DAYS)

        logger.info(f"Archiving conversations created before {threshold_date}")

        # Update conversations
        result = await db.execute_write(
            """
            UPDATE conversations
            SET archived = TRUE, updated_at = CURRENT_TIMESTAMP
            WHERE created_at < $1 AND archived = FALSE
            """,
            (threshold_date,),
        )

        # Get count of archived conversations
        count_result = await db.execute_one(
            """
            SELECT COUNT(*) FROM conversations
            WHERE archived = TRUE AND updated_at > NOW() - INTERVAL '1 minute'
            """
        )

        count = count_result[0] if count_result else 0

        logger.info(f"Successfully archived {count} conversations")

        # Disconnect
        await db.disconnect()

        return count

    except Exception as e:
        logger.error(f"Error archiving conversations: {e}", exc_info=True)
        raise


async def main():
    """Main entry point."""
    try:
        logger.info("=" * 60)
        logger.info("Conversation Archival Job - Starting")
        logger.info("=" * 60)

        count = await archive_old_conversations()

        logger.info("=" * 60)
        logger.info(f"Conversation Archival Job - Completed ({count} archived)")
        logger.info("=" * 60)

    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
