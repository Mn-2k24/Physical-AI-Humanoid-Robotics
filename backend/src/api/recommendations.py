"""
Recommendations API Endpoints
Purpose: /recommendations endpoints for personalized chapter recommendations
Date: 2025-12-14
"""

import json
import logging
import uuid

from fastapi import APIRouter, Depends, HTTPException, status

from ..core.middleware import get_current_user
from ..models.recommendation import RecommendationListResponse, RecommendationResponse
from ..services.neon import db
from ..services.recommendation_engine import recommendation_engine

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/recommendations", tags=["Recommendations"])


# ============================================================================
# T074: Get Personalized Recommendations
# ============================================================================


@router.get("/", response_model=RecommendationListResponse)
async def get_recommendations(
    current_user: dict = Depends(get_current_user),
):
    """
    Get personalized chapter recommendations.

    **Requires Authentication**

    Generates recommendations based on:
    - User experience level (40%)
    - Software background (30%)
    - Hardware availability (20%)
    - Sequential progression (10%)

    Returns top 5 recommendations with scores and reasons.

    Returns:
        List of personalized recommendations

    Raises:
        401: Not authenticated
        500: Internal server error
    """
    try:
        user_id = current_user["user_id"]

        # Get user profile
        user_profile_data = await db.execute_one(
            """
            SELECT u.email, u.full_name, up.experience_level,
                   usb.programming_languages, usb.frameworks,
                   uhb.available_hardware, uhb.robotics_hardware
            FROM users u
            LEFT JOIN user_profiles up ON u.id = up.user_id
            LEFT JOIN user_software_background usb ON u.id = usb.user_id
            LEFT JOIN user_hardware_background uhb ON u.id = uhb.user_id
            WHERE u.id = %s
            """,
            (user_id,),
        )

        if not user_profile_data:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND, detail="User profile not found"
            )

        # Parse user profile
        user_profile = {
            "experience_level": user_profile_data[2] or "Beginner",
            "programming_languages": json.loads(user_profile_data[3]) if user_profile_data[3] else [],
            "frameworks": json.loads(user_profile_data[4]) if user_profile_data[4] else [],
            "available_hardware": json.loads(user_profile_data[5]) if user_profile_data[5] else [],
            "robotics_hardware": json.loads(user_profile_data[6]) if user_profile_data[6] else [],
        }

        # Get reading progress
        progress_data = await db.execute_query(
            """
            SELECT chapter_id, completed
            FROM reading_progress
            WHERE user_id = %s
            """,
            (user_id,),
        )

        completed_chapters = [row[0] for row in progress_data if row[1]]
        in_progress_chapters = [row[0] for row in progress_data if not row[1]]

        # Generate recommendations
        recommendations = recommendation_engine.generate_recommendations(
            user_profile=user_profile,
            completed_chapters=completed_chapters,
            in_progress_chapters=in_progress_chapters,
            limit=5,
        )

        # Store recommendations in database
        recommendation_records = []
        for rec in recommendations:
            recommendation_id = str(uuid.uuid4())

            # Check if already exists (avoid duplicates)
            existing = await db.execute_one(
                """
                SELECT recommendation_id FROM recommendations
                WHERE user_id = %s AND recommended_chapter_id = %s AND dismissed = FALSE
                """,
                (user_id, rec["chapter_id"]),
            )

            if not existing:
                await db.execute_write(
                    """
                    INSERT INTO recommendations (
                        recommendation_id,
                        user_id,
                        recommended_chapter_id,
                        score,
                        reason,
                        dismissed
                    ) VALUES (%s, %s, %s, %s, %s, %s)
                    """,
                    (
                        recommendation_id,
                        user_id,
                        rec["chapter_id"],
                        rec["score"],
                        rec["reason"],
                        False,
                    ),
                )
            else:
                recommendation_id = existing[0]

            # Retrieve created recommendation
            rec_data = await db.execute_one(
                """
                SELECT recommendation_id, recommended_chapter_id, score, reason, dismissed, created_at
                FROM recommendations
                WHERE recommendation_id = %s
                """,
                (recommendation_id,),
            )

            recommendation_records.append(
                RecommendationResponse(
                    recommendation_id=rec_data[0],
                    recommended_chapter_id=rec_data[1],
                    chapter_title=rec["chapter_title"],
                    score=rec_data[2],
                    reason=rec_data[3],
                    dismissed=rec_data[4],
                    created_at=rec_data[5],
                )
            )

        logger.info(f"Generated {len(recommendation_records)} recommendations for user {user_id}")

        return RecommendationListResponse(
            recommendations=recommendation_records,
            total=len(recommendation_records),
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get recommendations error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to generate recommendations. Please try again.",
        )


# ============================================================================
# T075: Dismiss Recommendation
# ============================================================================


@router.put("/{recommendation_id}/dismiss")
async def dismiss_recommendation(
    recommendation_id: str,
    current_user: dict = Depends(get_current_user),
):
    """
    Dismiss a recommendation.

    **Requires Authentication**

    Sets dismissed=TRUE for the specified recommendation.

    Args:
        recommendation_id: Recommendation ID to dismiss

    Returns:
        Success message

    Raises:
        401: Not authenticated
        404: Recommendation not found
        500: Internal server error
    """
    try:
        user_id = current_user["user_id"]

        # Update recommendation
        result = await db.execute_write(
            """
            UPDATE recommendations
            SET dismissed = TRUE
            WHERE recommendation_id = %s AND user_id = %s
            """,
            (recommendation_id, user_id),
        )

        if not result:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Recommendation not found or access denied",
            )

        logger.info(f"Dismissed recommendation {recommendation_id} for user {user_id}")

        return {"message": "Recommendation dismissed successfully"}

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Dismiss recommendation error: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to dismiss recommendation. Please try again.",
        )
