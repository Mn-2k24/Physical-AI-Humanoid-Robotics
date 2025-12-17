"""
Recommendation Engine
Purpose: Generate personalized chapter recommendations based on user profile and progress
Date: 2025-12-14
"""

import json
import logging
from pathlib import Path
from typing import List, Dict, Optional

logger = logging.getLogger(__name__)

# Load chapter metadata
CHAPTERS_FILE = Path(__file__).parent.parent / "data" / "chapters.json"


class RecommendationEngine:
    """
    Recommendation engine for personalized chapter suggestions.

    Scoring weights:
    - Experience level: 40%
    - Software background: 30%
    - Hardware availability: 20%
    - Sequential progression: 10%
    """

    def __init__(self):
        """Initialize recommendation engine with chapter metadata."""
        self.chapters = self._load_chapters()
        self.difficulty_map = {
            "Beginner": 1,
            "Intermediate": 2,
            "Advanced": 3,
            "Expert": 4,
        }

    def _load_chapters(self) -> List[Dict]:
        """Load chapter metadata from JSON file."""
        try:
            with open(CHAPTERS_FILE, "r") as f:
                data = json.load(f)
                return data.get("chapters", [])
        except Exception as e:
            logger.error(f"Failed to load chapters.json: {e}")
            return []

    def generate_recommendations(
        self,
        user_profile: Dict,
        completed_chapters: List[str],
        in_progress_chapters: List[str],
        limit: int = 5,
    ) -> List[Dict]:
        """
        Generate personalized chapter recommendations.

        Args:
            user_profile: User profile with experience_level, programming_languages,
                         frameworks, available_hardware, robotics_hardware
            completed_chapters: List of completed chapter IDs
            in_progress_chapters: List of in-progress chapter IDs
            limit: Maximum number of recommendations to return

        Returns:
            List of recommendations with chapter_id, score, and reason
        """
        # Get user attributes
        experience_level = user_profile.get("experience_level", "Beginner")
        programming_languages = user_profile.get("programming_languages", [])
        frameworks = user_profile.get("frameworks", [])
        available_hardware = user_profile.get("available_hardware", [])
        robotics_hardware = user_profile.get("robotics_hardware", [])

        # Filter out completed and in-progress chapters
        excluded_ids = set(completed_chapters + in_progress_chapters)
        candidate_chapters = [ch for ch in self.chapters if ch["chapter_id"] not in excluded_ids]

        # Score each candidate chapter
        scored_chapters = []
        for chapter in candidate_chapters:
            score, reason = self._score_chapter(
                chapter=chapter,
                experience_level=experience_level,
                programming_languages=programming_languages,
                frameworks=frameworks,
                available_hardware=available_hardware,
                robotics_hardware=robotics_hardware,
                completed_chapters=completed_chapters,
            )

            scored_chapters.append(
                {
                    "chapter_id": chapter["chapter_id"],
                    "chapter_title": chapter["title"],
                    "score": score,
                    "reason": reason,
                    "difficulty": chapter["difficulty"],
                }
            )

        # Sort by score descending
        scored_chapters.sort(key=lambda x: x["score"], reverse=True)

        # Return top N
        return scored_chapters[:limit]

    def _score_chapter(
        self,
        chapter: Dict,
        experience_level: str,
        programming_languages: List[str],
        frameworks: List[str],
        available_hardware: List[str],
        robotics_hardware: List[str],
        completed_chapters: List[str],
    ) -> tuple[float, str]:
        """
        Score a chapter based on user profile.

        Returns:
            Tuple of (score, reason)
        """
        total_score = 0.0
        reasons = []

        # 1. Experience Level Match (40 points)
        experience_score = self._score_experience_level(chapter, experience_level)
        total_score += experience_score * 0.4
        if experience_score >= 80:
            reasons.append(f"Perfect match for {experience_level} level")
        elif experience_score >= 60:
            reasons.append(f"Good fit for {experience_level} level")

        # 2. Software Background Match (30 points)
        software_score = self._score_software_background(
            chapter, programming_languages, frameworks
        )
        total_score += software_score * 0.3
        if software_score >= 80:
            matching_langs = set(chapter.get("languages", [])) & set(programming_languages)
            matching_frameworks = set(chapter.get("frameworks", [])) & set(frameworks)
            if matching_langs:
                reasons.append(f"Uses {', '.join(matching_langs)}")
            if matching_frameworks:
                reasons.append(f"Covers {', '.join(matching_frameworks)}")

        # 3. Hardware Availability Match (20 points)
        hardware_score = self._score_hardware_availability(
            chapter, available_hardware, robotics_hardware
        )
        total_score += hardware_score * 0.2
        if hardware_score >= 80:
            reasons.append("Compatible with your hardware")
        elif hardware_score < 50 and chapter.get("hardware_requirements"):
            reasons.append("May require additional hardware")

        # 4. Sequential Progression (10 points)
        progression_score = self._score_sequential_progression(chapter, completed_chapters)
        total_score += progression_score * 0.1
        if progression_score == 100:
            reasons.append("Prerequisites completed")
        elif progression_score < 50 and chapter.get("prerequisite_chapters"):
            reasons.append("Some prerequisites not completed")

        # Generate final reason
        final_reason = "; ".join(reasons) if reasons else "General recommendation"

        return round(total_score, 2), final_reason

    def _score_experience_level(self, chapter: Dict, user_experience: str) -> float:
        """Score chapter based on experience level match."""
        chapter_difficulty = chapter.get("difficulty", "Beginner")

        user_level = self.difficulty_map.get(user_experience, 1)
        chapter_level = self.difficulty_map.get(chapter_difficulty, 1)

        # Perfect match
        if user_level == chapter_level:
            return 100.0

        # One level difference
        if abs(user_level - chapter_level) == 1:
            return 80.0

        # Two levels difference
        if abs(user_level - chapter_level) == 2:
            return 50.0

        # Too far apart
        return 20.0

    def _score_software_background(
        self, chapter: Dict, programming_languages: List[str], frameworks: List[str]
    ) -> float:
        """Score chapter based on software background match."""
        chapter_languages = set(chapter.get("languages", []))
        chapter_frameworks = set(chapter.get("frameworks", []))

        # If chapter has no requirements, it's accessible to everyone
        if not chapter_languages and not chapter_frameworks:
            return 100.0

        user_languages = set(programming_languages)
        user_frameworks = set(frameworks)

        # Calculate language match
        lang_score = 0.0
        if chapter_languages:
            matched_langs = chapter_languages & user_languages
            lang_score = (len(matched_langs) / len(chapter_languages)) * 100

        # Calculate framework match
        framework_score = 0.0
        if chapter_frameworks:
            matched_frameworks = chapter_frameworks & user_frameworks
            framework_score = (len(matched_frameworks) / len(chapter_frameworks)) * 100
        else:
            framework_score = 100.0  # No framework required

        # Weight languages more heavily (70%) than frameworks (30%)
        if chapter_languages:
            return lang_score * 0.7 + framework_score * 0.3
        else:
            return framework_score

    def _score_hardware_availability(
        self, chapter: Dict, available_hardware: List[str], robotics_hardware: List[str]
    ) -> float:
        """Score chapter based on hardware availability."""
        chapter_hardware = set(chapter.get("hardware_requirements", []))

        # If chapter has no hardware requirements, it's accessible
        if not chapter_hardware:
            return 100.0

        # Combine all user hardware
        all_user_hardware = set(available_hardware + robotics_hardware)

        # Check if user has required hardware
        matched_hardware = chapter_hardware & all_user_hardware

        if len(matched_hardware) == len(chapter_hardware):
            return 100.0  # All hardware available

        if len(matched_hardware) > 0:
            return (len(matched_hardware) / len(chapter_hardware)) * 100

        # Special case: CPU is always available
        if "CPU (x86/ARM)" in chapter_hardware:
            return 50.0

        return 0.0  # No hardware match

    def _score_sequential_progression(
        self, chapter: Dict, completed_chapters: List[str]
    ) -> float:
        """Score chapter based on sequential progression (prerequisites)."""
        prerequisites = chapter.get("prerequisite_chapters", [])

        # If no prerequisites, chapter is accessible
        if not prerequisites:
            return 100.0

        # Check how many prerequisites are completed
        completed_set = set(completed_chapters)
        completed_prereqs = set(prerequisites) & completed_set

        if len(completed_prereqs) == len(prerequisites):
            return 100.0  # All prerequisites completed

        if len(completed_prereqs) > 0:
            return (len(completed_prereqs) / len(prerequisites)) * 100

        return 0.0  # No prerequisites completed


# Global recommendation engine instance
recommendation_engine = RecommendationEngine()
