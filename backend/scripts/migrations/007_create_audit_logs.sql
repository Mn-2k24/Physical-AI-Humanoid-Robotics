-- Migration 007: Create Audit Logs Table
-- Purpose: Track authentication events for security auditing
-- Date: 2025-12-13

CREATE TABLE IF NOT EXISTS audit_logs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,
    event_type VARCHAR(50) NOT NULL,
    details JSONB,
    ip_address INET,
    user_agent TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create indexes for fast audit log queries
CREATE INDEX IF NOT EXISTS idx_audit_logs_user_id ON audit_logs(user_id);
CREATE INDEX IF NOT EXISTS idx_audit_logs_event_type ON audit_logs(event_type);
CREATE INDEX IF NOT EXISTS idx_audit_logs_created_at ON audit_logs(created_at DESC);

COMMENT ON TABLE audit_logs IS 'Authentication events for security auditing';
COMMENT ON COLUMN audit_logs.user_id IS 'User who triggered event (null for failed logins)';
COMMENT ON COLUMN audit_logs.event_type IS 'Event category (signup_success, signin_failed, etc.)';
COMMENT ON COLUMN audit_logs.details IS 'Additional event-specific data';
