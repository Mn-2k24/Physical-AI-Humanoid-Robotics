"""
Reading Progress API Endpoints
Purpose: /progress endpoints for tracking user reading progress
Date: 2025-12-14
"""

import logging
import uuid

from fastapi import APIRouter, Depends, HTTPException, status

from ..core.middleware import get_current_user
from ..models.progress import (
    ProgressListResponse,
    ReadingProgressCreate,
    ReadingProgressResponse,
)
from ..services.neon import db

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/progress", tags=["Progress"])


# ============================================================================
# T072: Update Reading Progress
# ============================================================================


@router.post("/", status_code=status.HTTP_200_OK)
async def update_progress(
    progress_data: ReadingProgressCreate,
    current_user: dict = Depends(get_current_user),
):
    """
    Update user's reading progress for a chapter.

    **Requires Authentication**

    Creates new progress record or updates existing one (upsert).
    Sets completed=TRUE if completion_percentage >= 90.

    Args:
        progress_data: Chapter progress data

    Returns:
        Updated progress record

    Raises:
        401: Not authenticated
        500: Internal server error
    """
    try:
        user_id = current_user["user_id"]
        chapter_id = progress_data.chapter_id
        completion_percentage = progress_data.completion_percentage
        time_spent_seconds = progress_data.time_spent_seconds
        last_position = progress_data.last_position

        # Determine if chapter is completed
        completed = completion_percentage >= 90

        # Check if progress record exists
        existing = await db.execute_one(
            """
            SELECT progress_id FROM reading_progress
            WHERE user_id = %s AND chapter_id = %s
            """,
            (user_id, chapter_id),
        )

        if existing:
            # Update existing record
            progress_id = existing[0]
            await db.execute_write(
                """
                UPDATE reading_progress
                SET completion_percentage = %s,
                    time_spent_seconds = time_spent_seconds + %s,
                    completed = %s,
                    last_position = %s,
                    updated_at = CURRENT_TIMESTAMP
                WHERE progress_id = %s
                """,
                (completion_percentage, time_spent_seconds, completed, last_position, progress_id),
            )
        else:
            # Create new record
            progress_id = str(uuid.uuid4())
            await db.execute_write(
                """
                INSERT INTO reading_progress (
                    progress_id,
                    user_id,
                    chapter_id,
                    completion_percentage,
                    time_spent_seconds,
                    completed,
                    last_position
                ) VALUES (%s, %s, %s, %s, %s, %s, %s)
                """,
                (
                    progress_id,
                    user_id,
                    chapter_id,
                    completion_percentage,
                    time_spent_seconds,
                    completed,
                    last_position,
                ),
            )

        # Retrieve updated record
        progress = await db.execute_one(
            """
            SELECT
                progress_id,
                chapter_id,
                completion_percentage,
                time_spent_seconds,
                completed,
                last_position,
                created_at,
                updated_at
            FROM reading_progress
            WHERE progress_id = %s
            """,
            (progress_id,),
        )

        logger.info(
            f"Updated progress for user {user_id}, chapter {chapter_id}: {completion_percentage}%"
        )

        return {
            "progress_id": progress[0],
            "chapter_id": progress[1],
            "completion_percentage": progress[2],
            "time_spent_seconds": progress[3],
            "completed": progress[4],
            "last_position": progress[5],
            "created_at": progress[6],
            "updated_at": progress[7],
        }

    except Exception as e:
        logger.error(f"Update progress error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to update progress. Please try again.",
        )


# ============================================================================
# T073: Get Reading Progress
# ============================================================================


@router.get("/", response_model=ProgressListResponse)
async def get_progress(
    current_user: dict = Depends(get_current_user),
):
    """
    Get user's reading progress for all chapters.

    **Requires Authentication**

    Returns:
        List of progress records with chapter titles

    Raises:
        401: Not authenticated
        500: Internal server error
    """
    try:
        user_id = current_user["user_id"]

        # Get all progress records
        progress_records = await db.execute_query(
            """
            SELECT
                progress_id,
                chapter_id,
                completion_percentage,
                time_spent_seconds,
                completed,
                last_position,
                created_at,
                updated_at
            FROM reading_progress
            WHERE user_id = %s
            ORDER BY updated_at DESC
            """,
            (user_id,),
        )

        # Load chapter metadata to get titles
        import json
        from pathlib import Path

        chapters_file = Path(__file__).parent.parent / "data" / "chapters.json"
        with open(chapters_file, "r") as f:
            chapters_data = json.load(f)

        chapter_map = {ch["chapter_id"]: ch["title"] for ch in chapters_data["chapters"]}

        # Format progress records
        progress_list = []
        completed_count = 0

        for record in progress_records:
            chapter_id = record[1]
            chapter_title = chapter_map.get(chapter_id, "Unknown Chapter")

            progress_list.append(
                ReadingProgressResponse(
                    progress_id=record[0],
                    chapter_id=chapter_id,
                    chapter_title=chapter_title,
                    completion_percentage=record[2],
                    time_spent_seconds=record[3],
                    completed=record[4],
                    last_position=record[5],
                    created_at=record[6],
                    updated_at=record[7],
                )
            )

            if record[4]:  # completed
                completed_count += 1

        logger.info(f"Retrieved {len(progress_list)} progress records for user {user_id}")

        return ProgressListResponse(
            progress=progress_list,
            total_chapters=len(chapters_data["chapters"]),
            completed_chapters=completed_count,
        )

    except Exception as e:
        logger.error(f"Get progress error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve progress. Please try again.",
        )
