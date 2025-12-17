/**
 * Reading Progress Tracker Hook
 * Purpose: Track scroll position and time spent on chapter, send updates to backend
 * Date: 2025-12-14
 * Updated: 2025-12-14 - Fixed process.env usage for browser compatibility
 */

import { useEffect, useRef, useState } from 'react';
import { getBackendUrl } from '../utils/config';

const BACKEND_URL = getBackendUrl();
const UPDATE_INTERVAL = 30000; // 30 seconds

interface ProgressData {
  chapterId: string;
  completionPercentage: number;
  timeSpentSeconds: number;
  lastPosition: number;
}

interface UseReadingProgressOptions {
  onComplete?: () => void;
}

export const useReadingProgress = (chapterId: string, options?: UseReadingProgressOptions) => {
  const [completionPercentage, setCompletionPercentage] = useState(0);
  const [timeSpent, setTimeSpent] = useState(0);
  const [isCompleted, setIsCompleted] = useState(false);
  const startTimeRef = useRef<number>(Date.now());
  const lastUpdateRef = useRef<number>(0);
  const intervalRef = useRef<NodeJS.Timeout>();
  const onCompleteRef = useRef(options?.onComplete);

  /**
   * Calculate completion percentage based on scroll position
   */
  const updateScrollProgress = () => {
    const windowHeight = window.innerHeight;
    const documentHeight = document.documentElement.scrollHeight;
    const scrollTop = window.scrollY || document.documentElement.scrollTop;

    // Calculate percentage
    const scrollable = documentHeight - windowHeight;
    const percentage = scrollable > 0 ? Math.round((scrollTop / scrollable) * 100) : 0;

    setCompletionPercentage(Math.min(percentage, 100));

    return {
      percentage: Math.min(percentage, 100),
      scrollTop,
    };
  };

  /**
   * Send progress update to backend
   */
  const sendProgressUpdate = async (data: ProgressData) => {
    try {
      const response = await fetch(`${BACKEND_URL}/progress`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include',
        body: JSON.stringify({
          chapter_id: data.chapterId,
          completion_percentage: data.completionPercentage,
          time_spent_seconds: data.timeSpentSeconds,
          last_position: data.lastPosition,
        }),
      });

      if (!response.ok) {
        console.error('Failed to update reading progress');
        return;
      }

      // Trigger completion callback when >= 90%
      if (data.completionPercentage >= 90 && !isCompleted) {
        setIsCompleted(true);
        if (onCompleteRef.current) {
          onCompleteRef.current();
        }
      }
    } catch (error) {
      console.error('Error updating reading progress:', error);
    }
  };

  /**
   * Periodic update (every 30 seconds)
   */
  const performPeriodicUpdate = () => {
    const now = Date.now();
    const elapsed = Math.floor((now - startTimeRef.current) / 1000);
    const { percentage, scrollTop } = updateScrollProgress();

    setTimeSpent(elapsed);

    // Only send if something changed
    if (percentage !== lastUpdateRef.current) {
      sendProgressUpdate({
        chapterId,
        completionPercentage: percentage,
        timeSpentSeconds: elapsed,
        lastPosition: scrollTop,
      });

      lastUpdateRef.current = percentage;
    }
  };

  /**
   * Final update on component unmount (chapter exit)
   */
  const performFinalUpdate = () => {
    const now = Date.now();
    const elapsed = Math.floor((now - startTimeRef.current) / 1000);
    const { percentage, scrollTop } = updateScrollProgress();

    sendProgressUpdate({
      chapterId,
      completionPercentage: percentage,
      timeSpentSeconds: elapsed,
      lastPosition: scrollTop,
    });
  };

  /**
   * Keep onComplete callback ref synced
   */
  useEffect(() => {
    onCompleteRef.current = options?.onComplete;
  }, [options?.onComplete]);

  /**
   * Setup tracking
   */
  useEffect(() => {
    // Reset start time for new chapter
    startTimeRef.current = Date.now();
    setTimeSpent(0);
    setIsCompleted(false);
    lastUpdateRef.current = 0;

    // Initial scroll position update
    updateScrollProgress();

    // Listen for scroll events
    const handleScroll = () => {
      updateScrollProgress();
    };

    window.addEventListener('scroll', handleScroll, { passive: true });

    // Periodic updates every 30 seconds
    intervalRef.current = setInterval(performPeriodicUpdate, UPDATE_INTERVAL);

    // Cleanup on unmount (send final update)
    return () => {
      window.removeEventListener('scroll', handleScroll);

      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }

      // Send final update
      performFinalUpdate();
    };
  }, [chapterId]);

  return {
    completionPercentage,
    timeSpent,
  };
};
