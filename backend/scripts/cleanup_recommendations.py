#!/usr/bin/env python3
"""
Cleanup Old Recommendations
Purpose: Keep last 10 recommendations per user, delete older ones
Date: 2025-12-14

Usage:
    python backend/scripts/cleanup_recommendations.py

Schedule with cron (daily at 3 AM):
    0 3 * * * cd /path/to/project && python backend/scripts/cleanup_recommendations.py
"""

import asyncio
import logging
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent.parent))

from backend.src.services.neon import db

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

MAX_RECOMMENDATIONS_PER_USER = 10


async def cleanup_old_recommendations():
    """
    Cleanup old recommendations.

    Keeps the last 10 recommendations per user (by created_at DESC).
    Deletes older recommendations.
    """
    try:
        # Connect to database
        await db.connect()

        logger.info(f"Starting recommendation cleanup (keeping last {MAX_RECOMMENDATIONS_PER_USER} per user)")

        # Get all user IDs
        users = await db.execute_query("SELECT DISTINCT user_id FROM recommendations")

        total_deleted = 0

        for user_row in users:
            user_id = user_row[0]

            # Get recommendations for this user, ordered by created_at DESC
            recommendations = await db.execute_query(
                """
                SELECT recommendation_id, created_at
                FROM recommendations
                WHERE user_id = $1
                ORDER BY created_at DESC
                """,
                (user_id,),
            )

            # If user has more than MAX recommendations, delete the older ones
            if len(recommendations) > MAX_RECOMMENDATIONS_PER_USER:
                # Get IDs of recommendations to delete (everything after the first MAX)
                to_delete = [rec[0] for rec in recommendations[MAX_RECOMMENDATIONS_PER_USER:]]

                # Delete old recommendations
                for rec_id in to_delete:
                    await db.execute_write(
                        "DELETE FROM recommendations WHERE recommendation_id = $1",
                        (rec_id,),
                    )

                deleted_count = len(to_delete)
                total_deleted += deleted_count
                logger.info(f"Deleted {deleted_count} old recommendations for user {user_id}")

        logger.info(f"Cleanup completed. Total recommendations deleted: {total_deleted}")

        # Disconnect
        await db.disconnect()

        return total_deleted

    except Exception as e:
        logger.error(f"Error cleaning up recommendations: {e}", exc_info=True)
        raise


async def main():
    """Main entry point."""
    try:
        logger.info("=" * 60)
        logger.info("Recommendation Cleanup Job - Starting")
        logger.info("=" * 60)

        count = await cleanup_old_recommendations()

        logger.info("=" * 60)
        logger.info(f"Recommendation Cleanup Job - Completed ({count} deleted)")
        logger.info("=" * 60)

    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
