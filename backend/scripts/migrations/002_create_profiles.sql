-- Migration 002: Create User Profiles Tables
-- Purpose: Store user background information for personalized recommendations
-- Date: 2025-12-13

-- User experience level
CREATE TABLE IF NOT EXISTS user_profiles (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    experience_level VARCHAR(20) NOT NULL CHECK (experience_level IN ('Beginner', 'Intermediate', 'Advanced', 'Expert')),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

COMMENT ON TABLE user_profiles IS 'User experience level for personalized recommendations';
COMMENT ON COLUMN user_profiles.experience_level IS 'Self-reported AI/robotics experience level';

-- User software background
CREATE TABLE IF NOT EXISTS user_software_background (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    programming_languages JSONB NOT NULL DEFAULT '[]',
    frameworks JSONB NOT NULL DEFAULT '[]'
);

COMMENT ON TABLE user_software_background IS 'Programming languages and frameworks the user knows';
COMMENT ON COLUMN user_software_background.programming_languages IS 'Array of programming language strings (e.g., ["Python", "C++"])';
COMMENT ON COLUMN user_software_background.frameworks IS 'Array of framework strings (e.g., ["ROS 2", "PyTorch"])';

-- User hardware background
CREATE TABLE IF NOT EXISTS user_hardware_background (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    available_hardware JSONB NOT NULL DEFAULT '[]',
    robotics_hardware JSONB NOT NULL DEFAULT '[]'
);

COMMENT ON TABLE user_hardware_background IS 'Available compute and robotics hardware';
COMMENT ON COLUMN user_hardware_background.available_hardware IS 'Array of compute hardware (e.g., ["NVIDIA GPU", "CPU only"])';
COMMENT ON COLUMN user_hardware_background.robotics_hardware IS 'Array of robotics hardware (e.g., ["None (simulation only)", "Mobile robot"])';
