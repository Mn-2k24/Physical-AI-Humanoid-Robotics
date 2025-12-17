/**
 * Recommendations Sidebar Component
 * Purpose: Display 3-5 personalized chapter recommendations with scores, reasons, and dismiss button
 * Date: 2025-12-14
 * Updated: 2025-12-14 - Fixed process.env usage for browser compatibility
 */

import React, { useEffect, useState } from 'react';
import { useHistory } from '@docusaurus/router';
import { getBackendUrl } from '../../utils/config';
import styles from './RecommendationsSidebar.module.css';

const BACKEND_URL = getBackendUrl();

interface Recommendation {
  recommendation_id: string;
  recommended_chapter_id: string;
  chapter_title: string;
  score: number;
  reason: string;
  dismissed: boolean;
  created_at: string;
}

interface RecommendationsSidebarProps {
  refreshTrigger?: number;
}

export const RecommendationsSidebar: React.FC<RecommendationsSidebarProps> = ({
  refreshTrigger,
}) => {
  const history = useHistory();
  const [recommendations, setRecommendations] = useState<Recommendation[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  /**
   * Fetch recommendations from backend
   */
  const fetchRecommendations = async () => {
    try {
      setIsLoading(true);
      setError(null);

      const response = await fetch(`${BACKEND_URL}/recommendations`, {
        method: 'GET',
        credentials: 'include',
      });

      if (!response.ok) {
        if (response.status === 401) {
          // User not authenticated
          setRecommendations([]);
          return;
        }
        throw new Error('Failed to fetch recommendations');
      }

      const data = await response.json();
      setRecommendations(data.recommendations || []);
    } catch (err) {
      console.error('Error fetching recommendations:', err);
      setError('Failed to load recommendations');
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Dismiss a recommendation
   */
  const handleDismiss = async (recommendationId: string) => {
    try {
      const response = await fetch(
        `${BACKEND_URL}/recommendations/${recommendationId}/dismiss`,
        {
          method: 'PUT',
          credentials: 'include',
        }
      );

      if (!response.ok) {
        throw new Error('Failed to dismiss recommendation');
      }

      // Remove from local state
      setRecommendations((prev) =>
        prev.filter((rec) => rec.recommendation_id !== recommendationId)
      );
    } catch (err) {
      console.error('Error dismissing recommendation:', err);
    }
  };

  /**
   * Navigate to recommended chapter
   */
  const handleChapterClick = (chapterId: string) => {
    // Assuming chapter URL pattern: /book/chapter-id
    history.push(`/book/${chapterId}`);
  };

  /**
   * Fetch recommendations on mount
   */
  useEffect(() => {
    fetchRecommendations();
  }, []);

  /**
   * Refresh recommendations when trigger changes (e.g., chapter completion)
   */
  useEffect(() => {
    if (refreshTrigger !== undefined && refreshTrigger > 0) {
      fetchRecommendations();
    }
  }, [refreshTrigger]);

  /**
   * Get score color based on value
   */
  const getScoreColor = (score: number): string => {
    if (score >= 80) return styles.scoreHigh;
    if (score >= 60) return styles.scoreMedium;
    return styles.scoreLow;
  };

  if (isLoading) {
    return (
      <div className={styles.sidebar}>
        <h3 className={styles.title}>Recommended Chapters</h3>
        <div className={styles.loading}>Loading recommendations...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.sidebar}>
        <h3 className={styles.title}>Recommended Chapters</h3>
        <div className={styles.error}>{error}</div>
      </div>
    );
  }

  if (recommendations.length === 0) {
    return (
      <div className={styles.sidebar}>
        <h3 className={styles.title}>Recommended Chapters</h3>
        <div className={styles.empty}>
          <p>Complete more chapters to get personalized recommendations!</p>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.sidebar}>
      <h3 className={styles.title}>Recommended Chapters</h3>

      <div className={styles.recommendationsList}>
        {recommendations.slice(0, 5).map((rec, index) => (
          <div key={rec.recommendation_id} className={styles.recommendationCard}>
            <div className={styles.cardHeader}>
              <span className={styles.rank}>#{index + 1}</span>
              <span className={`${styles.score} ${getScoreColor(rec.score)}`}>
                {rec.score.toFixed(0)}%
              </span>
            </div>

            <h4
              className={styles.chapterTitle}
              onClick={() => handleChapterClick(rec.recommended_chapter_id)}
            >
              {rec.chapter_title}
            </h4>

            <p className={styles.reason}>{rec.reason}</p>

            <div className={styles.cardFooter}>
              <button
                className={styles.viewButton}
                onClick={() => handleChapterClick(rec.recommended_chapter_id)}
              >
                View Chapter
              </button>
              <button
                className={styles.dismissButton}
                onClick={() => handleDismiss(rec.recommendation_id)}
                title="Dismiss this recommendation"
              >
                ✕
              </button>
            </div>
          </div>
        ))}
      </div>

      <button className={styles.refreshButton} onClick={fetchRecommendations}>
        Refresh Recommendations
      </button>
    </div>
  );
};
