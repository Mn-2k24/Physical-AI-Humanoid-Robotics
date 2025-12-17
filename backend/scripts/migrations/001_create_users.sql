-- Migration 001: Create Users Table
-- Purpose: Core user account information
-- Date: 2025-12-13

CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    full_name VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create index for fast email lookups during signin
CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);

-- Add comment for documentation
COMMENT ON TABLE users IS 'Core user account information for authentication';
COMMENT ON COLUMN users.email IS 'User email address (unique, used for signin)';
COMMENT ON COLUMN users.hashed_password IS 'Bcrypt/Argon2 hashed password (never store plaintext)';
