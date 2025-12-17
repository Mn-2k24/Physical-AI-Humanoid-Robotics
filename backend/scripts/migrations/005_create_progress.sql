-- Migration 005: Create Reading Progress Table
-- Purpose: Track user progress through book chapters
-- Date: 2025-12-13

CREATE TABLE IF NOT EXISTS reading_progress (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    chapter_id VARCHAR(255) NOT NULL,
    completion_percentage INTEGER NOT NULL CHECK (completion_percentage >= 0 AND completion_percentage <= 100),
    time_spent_seconds INTEGER DEFAULT 0,
    last_accessed TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    completed BOOLEAN DEFAULT FALSE,
    CONSTRAINT unique_user_chapter UNIQUE (user_id, chapter_id)
);

-- Create indexes for fast progress queries
CREATE INDEX IF NOT EXISTS idx_reading_progress_user_id ON reading_progress(user_id);
CREATE INDEX IF NOT EXISTS idx_reading_progress_completed ON reading_progress(completed);

COMMENT ON TABLE reading_progress IS 'User progress through book chapters';
COMMENT ON COLUMN reading_progress.chapter_id IS 'Chapter identifier (e.g., "module-1-ros2/intro")';
COMMENT ON COLUMN reading_progress.completion_percentage IS 'Scroll progress (0% = not started, 100% = fully scrolled)';
COMMENT ON COLUMN reading_progress.completed IS 'Whether user completed chapter (completion_percentage >= 90)';
