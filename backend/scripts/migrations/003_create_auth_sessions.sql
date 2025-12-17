-- Migration 003: Create Authentication Sessions Table
-- Purpose: Manage active user sessions for authentication
-- Date: 2025-12-13

CREATE TABLE IF NOT EXISTS auth_sessions (
    session_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token_hash VARCHAR(255) NOT NULL,
    expires_at TIMESTAMP NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create indexes for fast session lookups and cleanup
CREATE INDEX IF NOT EXISTS idx_sessions_user_id ON auth_sessions(user_id);
CREATE INDEX IF NOT EXISTS idx_sessions_expires_at ON auth_sessions(expires_at);

COMMENT ON TABLE auth_sessions IS 'Active user sessions for authentication tracking';
COMMENT ON COLUMN auth_sessions.token_hash IS 'SHA-256 hash of session token (never store plaintext tokens)';
COMMENT ON COLUMN auth_sessions.expires_at IS 'Session expiration timestamp (24 hours from creation)';
