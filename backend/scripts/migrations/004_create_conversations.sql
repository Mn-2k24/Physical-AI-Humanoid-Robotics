-- Migration 004: Create Conversations and Chat Interactions Tables
-- Purpose: Store conversation threads and individual chat messages
-- Date: 2025-12-13

-- Conversation threads
CREATE TABLE IF NOT EXISTS conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    title VARCHAR(255),
    archived BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create composite index for efficient conversation list queries
CREATE INDEX IF NOT EXISTS idx_conversations_user_id ON conversations(user_id);
CREATE INDEX IF NOT EXISTS idx_conversations_archived ON conversations(archived);
CREATE INDEX IF NOT EXISTS idx_conversations_user_archived_created
ON conversations(user_id, archived, created_at DESC);

COMMENT ON TABLE conversations IS 'Conversation threads grouping related chat interactions';
COMMENT ON COLUMN conversations.title IS 'Auto-generated from first query (first 50 characters)';
COMMENT ON COLUMN conversations.archived IS 'Whether conversation is archived (>30 days old)';

-- Individual chat messages
CREATE TABLE IF NOT EXISTS chat_interactions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conversation_id UUID NOT NULL REFERENCES conversations(id) ON DELETE CASCADE,
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    query_text TEXT NOT NULL,
    answer_text TEXT NOT NULL,
    query_mode VARCHAR(10) NOT NULL CHECK (query_mode IN ('global', 'local')),
    source_chunks JSONB NOT NULL DEFAULT '[]',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create indexes for fast conversation detail queries
CREATE INDEX IF NOT EXISTS idx_chat_interactions_conversation_id ON chat_interactions(conversation_id);
CREATE INDEX IF NOT EXISTS idx_chat_interactions_created_at ON chat_interactions(created_at);

COMMENT ON TABLE chat_interactions IS 'Individual chat messages (query + answer pairs)';
COMMENT ON COLUMN chat_interactions.query_mode IS 'Query mode: ''global'' (full-book search) or ''local'' (selected text only)';
COMMENT ON COLUMN chat_interactions.source_chunks IS 'Array of retrieved chunk references with similarity scores';
