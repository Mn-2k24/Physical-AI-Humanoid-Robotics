-- Migration 006: Create Recommendations Table
-- Purpose: Store generated personalized chapter recommendations
-- Date: 2025-12-13

CREATE TABLE IF NOT EXISTS recommendations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    recommended_chapter_id VARCHAR(255) NOT NULL,
    score FLOAT NOT NULL CHECK (score >= 0 AND score <= 1),
    reason TEXT,
    dismissed BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create indexes for fast recommendation queries
CREATE INDEX IF NOT EXISTS idx_recommendations_user_id ON recommendations(user_id);
CREATE INDEX IF NOT EXISTS idx_recommendations_dismissed ON recommendations(dismissed);
CREATE INDEX IF NOT EXISTS idx_recommendations_score ON recommendations(score DESC);

COMMENT ON TABLE recommendations IS 'Generated personalized chapter recommendations';
COMMENT ON COLUMN recommendations.score IS 'Recommendation score (0.0 = poor match, 1.0 = perfect match)';
COMMENT ON COLUMN recommendations.reason IS 'Human-readable explanation (e.g., "Matches your NVIDIA GPU")';
