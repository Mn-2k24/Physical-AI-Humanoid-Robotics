# Data Model: Integrated RAG Chatbot & Authentication System

**Feature**: 001-rag-chatbot-auth
**Date**: 2025-12-13
**Purpose**: Define database schemas, entity relationships, validation rules, and state transitions

## Overview

This document defines the complete data model for the RAG chatbot and authentication system, covering:
- **Neon Serverless Postgres**: User accounts, conversations, progress, recommendations
- **Qdrant Cloud**: Book chapter embeddings and metadata

---

## Entity Relationship Diagram

```
┌─────────────┐
│    User     │
└──────┬──────┘
       │ 1
       │
       ├──────1:1──────► UserProfile
       ├──────1:1──────► UserSoftwareBackground
       ├──────1:1──────► UserHardwareBackground
       ├──────1:N──────► AuthSession
       ├──────1:N──────► Conversation
       ├──────1:N──────► ReadingProgress
       └──────1:N──────► Recommendation

┌──────────────┐
│ Conversation │
└──────┬───────┘
       │ 1
       │
       └──────1:N──────► ChatInteraction

┌──────────────┐
│  BookChunk   │ (Qdrant only)
└──────────────┘
```

---

## Neon Postgres Entities

### 1. User

**Purpose**: Core user account information

**Schema**:
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    full_name VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_users_email ON users(email);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Auto-generated | Unique user identifier |
| email | VARCHAR(255) | Unique, Not Null, Email format | User's email address (used for signin) |
| hashed_password | VARCHAR(255) | Not Null | Bcrypt/Argon2 hashed password (never store plaintext) |
| full_name | VARCHAR(255) | Not Null | User's display name |
| created_at | TIMESTAMP | Default CURRENT_TIMESTAMP | Account creation timestamp |
| updated_at | TIMESTAMP | Default CURRENT_TIMESTAMP | Last profile update timestamp |

**Validation Rules**:
- Email must match regex: `^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$`
- Password must be at least 8 characters (validated before hashing)
- full_name must be at least 2 characters

**State Transitions**:
```
Created → Active (default state after signup)
Active → Suspended (admin action, not implemented in MVP)
Suspended → Active (admin action, not implemented in MVP)
```

**Relationships**:
- 1:1 with UserProfile
- 1:1 with UserSoftwareBackground
- 1:1 with UserHardwareBackground
- 1:N with AuthSession
- 1:N with Conversation
- 1:N with ReadingProgress
- 1:N with Recommendation

---

### 2. UserProfile

**Purpose**: Store user experience level for personalized recommendations

**Schema**:
```sql
CREATE TABLE user_profiles (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    experience_level VARCHAR(20) NOT NULL CHECK (experience_level IN ('Beginner', 'Intermediate', 'Advanced', 'Expert')),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| user_id | UUID | Primary Key, Foreign Key → users(id), ON DELETE CASCADE | References parent user |
| experience_level | VARCHAR(20) | Not Null, CHECK constraint | User's self-reported AI/robotics experience level |
| created_at | TIMESTAMP | Default CURRENT_TIMESTAMP | Profile creation timestamp |
| updated_at | TIMESTAMP | Default CURRENT_TIMESTAMP | Last update timestamp |

**Valid Values**:
- `Beginner`: New to AI/robotics, learning fundamentals
- `Intermediate`: Some AI/robotics experience, building projects
- `Advanced`: Professional experience, advanced topics
- `Expert`: Research/industry expert, cutting-edge topics

**Relationships**:
- 1:1 with User (one profile per user)

---

### 3. UserSoftwareBackground

**Purpose**: Track user's programming languages and frameworks for recommendations

**Schema**:
```sql
CREATE TABLE user_software_background (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    programming_languages JSONB NOT NULL DEFAULT '[]',
    frameworks JSONB NOT NULL DEFAULT '[]'
);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| user_id | UUID | Primary Key, Foreign Key → users(id), ON DELETE CASCADE | References parent user |
| programming_languages | JSONB | Not Null, Must be array | Array of programming language strings |
| frameworks | JSONB | Not Null, Must be array | Array of framework strings |

**Valid Programming Languages** (predefined options):
```json
["Python", "C++", "JavaScript", "Rust", "Go", "Java", "MATLAB", "Julia", "Other"]
```

**Valid Frameworks** (predefined options):
```json
["ROS", "ROS 2", "PyTorch", "TensorFlow", "Unity", "Unreal Engine", "Gazebo", "Mujoco", "Isaac Sim", "Other"]
```

**Example**:
```json
{
  "programming_languages": ["Python", "C++"],
  "frameworks": ["ROS 2", "PyTorch"]
}
```

**Validation Rules**:
- Arrays must contain at least one element
- All values must be from predefined options (validated at application layer)

**Relationships**:
- 1:1 with User

---

### 4. UserHardwareBackground

**Purpose**: Track user's available hardware for recommendations

**Schema**:
```sql
CREATE TABLE user_hardware_background (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    available_hardware JSONB NOT NULL DEFAULT '[]',
    robotics_hardware JSONB NOT NULL DEFAULT '[]'
);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| user_id | UUID | Primary Key, Foreign Key → users(id), ON DELETE CASCADE | References parent user |
| available_hardware | JSONB | Not Null, Must be array | Array of compute hardware strings |
| robotics_hardware | JSONB | Not Null, Must be array | Array of robotics hardware strings |

**Valid Available Hardware** (predefined options):
```json
["CPU only", "NVIDIA GPU", "AMD GPU", "Apple Silicon (M1/M2)", "NVIDIA Jetson", "Google TPU", "Other"]
```

**Valid Robotics Hardware** (predefined options):
```json
["None (simulation only)", "Educational robot kit", "Research robotic arm", "Mobile robot", "Humanoid robot", "Custom hardware", "Other"]
```

**Example**:
```json
{
  "available_hardware": ["NVIDIA GPU", "CPU only"],
  "robotics_hardware": ["None (simulation only)"]
}
```

**Validation Rules**:
- Arrays can be empty (user may have no hardware)
- All values must be from predefined options (validated at application layer)

**Relationships**:
- 1:1 with User

---

### 5. AuthSession

**Purpose**: Manage active user sessions for authentication

**Schema**:
```sql
CREATE TABLE auth_sessions (
    session_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token_hash VARCHAR(255) NOT NULL,
    expires_at TIMESTAMP NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_sessions_user_id ON auth_sessions(user_id);
CREATE INDEX idx_sessions_expires_at ON auth_sessions(expires_at);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| session_id | UUID | Primary Key, Auto-generated | Unique session identifier |
| user_id | UUID | Foreign Key → users(id), ON DELETE CASCADE, Not Null | User who owns this session |
| token_hash | VARCHAR(255) | Not Null | SHA-256 hash of session token (never store plaintext tokens) |
| expires_at | TIMESTAMP | Not Null | Session expiration timestamp (24 hours from creation) |
| created_at | TIMESTAMP | Default CURRENT_TIMESTAMP | Session creation timestamp |

**State Transitions**:
```
Active (created_at < NOW < expires_at)
    ↓
Expired (NOW > expires_at)
    ↓
Deleted (cleanup job removes expired sessions)
```

**Validation Rules**:
- expires_at must be > created_at
- Token hash must be 64 characters (SHA-256 hex output)

**Cleanup Strategy**:
- Daily cron job deletes sessions where expires_at < NOW() - 7 days

**Relationships**:
- N:1 with User (user can have multiple active sessions across devices)

---

### 6. Conversation

**Purpose**: Group related chat interactions into conversation threads

**Schema**:
```sql
CREATE TABLE conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    title VARCHAR(255),
    archived BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_conversations_user_id ON conversations(user_id);
CREATE INDEX idx_conversations_archived ON conversations(archived);
CREATE INDEX idx_conversations_user_archived_created ON conversations(user_id, archived, created_at DESC);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Auto-generated | Unique conversation identifier |
| user_id | UUID | Foreign Key → users(id), ON DELETE CASCADE, Not Null | User who owns this conversation |
| title | VARCHAR(255) | Nullable | Auto-generated from first query (e.g., "What is ROS 2?") |
| archived | BOOLEAN | Default FALSE | Whether conversation is archived (>30 days old) |
| created_at | TIMESTAMP | Default CURRENT_TIMESTAMP | Conversation start timestamp |
| updated_at | TIMESTAMP | Default CURRENT_TIMESTAMP | Last message timestamp |

**State Transitions**:
```
Active (archived = FALSE, age < 30 days)
    ↓
Archived (archived = TRUE, age >= 30 days)
    ↓
(Optional) Deleted (admin action, not implemented in MVP)
```

**Title Generation**:
- Automatically set to first 50 characters of first query
- Example: "What is ROS 2?" → title = "What is ROS 2?"
- If query > 50 chars, truncate with "..."

**Archival Strategy**:
- Daily cron job sets archived = TRUE for conversations where created_at < NOW() - 30 days

**Relationships**:
- N:1 with User
- 1:N with ChatInteraction

---

### 7. ChatInteraction

**Purpose**: Store individual chat messages (query + answer pairs)

**Schema**:
```sql
CREATE TABLE chat_interactions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conversation_id UUID NOT NULL REFERENCES conversations(id) ON DELETE CASCADE,
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    query_text TEXT NOT NULL,
    answer_text TEXT NOT NULL,
    query_mode VARCHAR(10) NOT NULL CHECK (query_mode IN ('global', 'local')),
    source_chunks JSONB NOT NULL DEFAULT '[]',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_chat_interactions_conversation_id ON chat_interactions(conversation_id);
CREATE INDEX idx_chat_interactions_created_at ON chat_interactions(created_at);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Auto-generated | Unique interaction identifier |
| conversation_id | UUID | Foreign Key → conversations(id), ON DELETE CASCADE, Not Null | Parent conversation |
| user_id | UUID | Foreign Key → users(id), ON DELETE CASCADE, Not Null | User who asked the question |
| query_text | TEXT | Not Null | User's original question |
| answer_text | TEXT | Not Null | Chatbot's answer (grounded in book content) |
| query_mode | VARCHAR(10) | Not Null, CHECK constraint | Query mode: 'global' or 'local' |
| source_chunks | JSONB | Not Null, Must be array | Array of retrieved chunk references |
| created_at | TIMESTAMP | Default CURRENT_TIMESTAMP | Interaction timestamp |

**Query Modes**:
- `global`: Full-book search (all chunks eligible)
- `local`: Selected-text only (filtered by file_path + chunk_index range)

**Source Chunks Format**:
```json
[
  {
    "file_path": "docs/module-1-ros2/introduction.md",
    "section_heading": "What is ROS 2?",
    "chunk_index": 0,
    "similarity_score": 0.87
  }
]
```

**Validation Rules**:
- query_text must be at least 3 characters
- answer_text must not be empty
- source_chunks must be array with 1-5 elements

**Relationships**:
- N:1 with Conversation
- N:1 with User (denormalized for easier user-level queries)

---

### 8. ReadingProgress

**Purpose**: Track user's progress through book chapters

**Schema**:
```sql
CREATE TABLE reading_progress (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    chapter_id VARCHAR(255) NOT NULL,
    completion_percentage INTEGER NOT NULL CHECK (completion_percentage >= 0 AND completion_percentage <= 100),
    time_spent_seconds INTEGER DEFAULT 0,
    last_accessed TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    completed BOOLEAN DEFAULT FALSE,
    CONSTRAINT unique_user_chapter UNIQUE (user_id, chapter_id)
);

CREATE INDEX idx_reading_progress_user_id ON reading_progress(user_id);
CREATE INDEX idx_reading_progress_completed ON reading_progress(completed);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Auto-generated | Unique progress record identifier |
| user_id | UUID | Foreign Key → users(id), ON DELETE CASCADE, Not Null | User who is reading |
| chapter_id | VARCHAR(255) | Not Null, Unique constraint with user_id | Chapter identifier (e.g., "module-1-ros2/intro") |
| completion_percentage | INTEGER | Not Null, CHECK 0-100 | Scroll progress (0% = not started, 100% = fully scrolled) |
| time_spent_seconds | INTEGER | Default 0 | Total time spent on chapter (tracked via timer) |
| last_accessed | TIMESTAMP | Default CURRENT_TIMESTAMP | Last time user viewed this chapter |
| completed | BOOLEAN | Default FALSE | Whether user completed chapter (completion_percentage >= 90) |

**Update Strategy**:
- Frontend sends progress updates every 30 seconds or on chapter exit
- `completion_percentage` = MAX(current, new_value) (never decrease)
- `time_spent_seconds` += new_time_delta (cumulative)
- `completed` = TRUE if completion_percentage >= 90

**Validation Rules**:
- completion_percentage must be 0-100
- time_spent_seconds must be >= 0
- Only one record per (user_id, chapter_id) pair

**Relationships**:
- N:1 with User

---

### 9. Recommendation

**Purpose**: Store generated personalized chapter recommendations

**Schema**:
```sql
CREATE TABLE recommendations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    recommended_chapter_id VARCHAR(255) NOT NULL,
    score FLOAT NOT NULL CHECK (score >= 0 AND score <= 1),
    reason TEXT,
    dismissed BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_recommendations_user_id ON recommendations(user_id);
CREATE INDEX idx_recommendations_dismissed ON recommendations(dismissed);
CREATE INDEX idx_recommendations_score ON recommendations(score DESC);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Auto-generated | Unique recommendation identifier |
| user_id | UUID | Foreign Key → users(id), ON DELETE CASCADE, Not Null | User receiving recommendation |
| recommended_chapter_id | VARCHAR(255) | Not Null | Chapter being recommended (e.g., "module-2-simulation/isaac-sim") |
| score | FLOAT | Not Null, CHECK 0-1 | Recommendation score (0.0 = poor match, 1.0 = perfect match) |
| reason | TEXT | Nullable | Human-readable explanation (e.g., "Matches your NVIDIA GPU") |
| dismissed | BOOLEAN | Default FALSE | Whether user dismissed this recommendation |
| created_at | TIMESTAMP | Default CURRENT_TIMESTAMP | Recommendation generation timestamp |

**Score Calculation** (see research.md for algorithm details):
- Experience level match: 0.4 weight
- Software background match: 0.3 weight
- Hardware availability match: 0.2 weight
- Sequential progression: 0.1 weight

**Reason Examples**:
- "Matches your Python background and NVIDIA GPU"
- "Next chapter in your learning path"
- "Beginner-friendly introduction to simulation"

**Cleanup Strategy**:
- Keep last 10 recommendations per user
- Delete old recommendations when generating new ones (FIFO)

**Relationships**:
- N:1 with User

---

### 10. AuditLog

**Purpose**: Track authentication events for security auditing

**Schema**:
```sql
CREATE TABLE audit_logs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,
    event_type VARCHAR(50) NOT NULL,
    details JSONB,
    ip_address INET,
    user_agent TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_audit_logs_user_id ON audit_logs(user_id);
CREATE INDEX idx_audit_logs_event_type ON audit_logs(event_type);
CREATE INDEX idx_audit_logs_created_at ON audit_logs(created_at DESC);
```

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Auto-generated | Unique log entry identifier |
| user_id | UUID | Foreign Key → users(id), ON DELETE SET NULL, Nullable | User who triggered event (null for failed logins) |
| event_type | VARCHAR(50) | Not Null | Event category |
| details | JSONB | Nullable | Additional event-specific data |
| ip_address | INET | Nullable | Client IP address |
| user_agent | TEXT | Nullable | Client user agent string |
| created_at | TIMESTAMP | Default CURRENT_TIMESTAMP | Event timestamp |

**Event Types**:
- `signup_success`
- `signup_failed`
- `signin_success`
- `signin_failed`
- `signout`
- `password_reset_requested`
- `password_reset_completed`
- `session_expired`

**Example Details**:
```json
{
  "email": "user@example.com",
  "failure_reason": "Invalid password",
  "attempts_count": 3
}
```

**Retention Policy**:
- Keep logs for 90 days
- Daily cron job deletes logs where created_at < NOW() - 90 days

**Relationships**:
- N:1 with User (nullable, ON DELETE SET NULL preserves logs after user deletion)

---

## Qdrant Entities

### 11. BookChunk (Vector Store)

**Purpose**: Store book chapter embeddings for RAG retrieval

**Collection Configuration**:
```python
{
  "collection_name": "physical_ai_book",
  "vectors": {
    "size": 768,  # text-embedding-004 dimension
    "distance": "Cosine"
  },
  "payload_schema": {
    "file_path": "keyword",
    "section_heading": "text",
    "chunk_index": "integer",
    "raw_text": "text"
  }
}
```

**Fields**:
| Field | Type | Indexed | Description |
|-------|------|---------|-------------|
| id | UUID | Yes (primary) | Unique chunk identifier |
| vector | float[] (768-dim) | Yes (HNSW) | Gemini embedding vector |
| file_path | string | Yes (keyword) | Source file path (e.g., "docs/module-1-ros2/intro.md") |
| section_heading | string | No | Section/chapter heading (e.g., "What is ROS 2?") |
| chunk_index | integer | Yes | Chunk sequence number within file (0, 1, 2...) |
| raw_text | string | No | Original text content (for answer generation) |

**Vector Distance Metric**:
- **Cosine Similarity**: Best for semantic similarity of embeddings
- Range: [-1, 1] where 1 = identical, 0 = orthogonal, -1 = opposite

**Indexing Strategy**:
- HNSW (Hierarchical Navigable Small World) for fast approximate nearest neighbor search
- m=16 (neighbors per node), ef_construct=100 (construction depth)

**Filtering Patterns**:

**Global Search** (all chunks):
```python
client.search(
    collection_name="physical_ai_book",
    query_vector=query_embedding,
    limit=5
)
```

**Local Search** (selected text only):
```python
client.search(
    collection_name="physical_ai_book",
    query_vector=query_embedding,
    query_filter={
        "must": [
            {"key": "file_path", "match": {"value": "docs/module-1-ros2/intro.md"}},
            {"key": "chunk_index", "range": {"gte": 0, "lte": 5}}
        ]
    },
    limit=5
)
```

**Storage Estimate**:
- 1000 chunks × 768 dimensions × 4 bytes (float32) = ~3 MB vectors
- 1000 chunks × 500 bytes payload avg = ~0.5 MB payloads
- HNSW index overhead (3x) = ~10.5 MB total
- **Well within 1GB free tier limit**

---

## Data Integrity Constraints

### Foreign Key Cascades

| Parent Table | Child Table | On Delete Action | Rationale |
|--------------|-------------|------------------|-----------|
| users | user_profiles | CASCADE | Profile meaningless without user |
| users | user_software_background | CASCADE | Background meaningless without user |
| users | user_hardware_background | CASCADE | Background meaningless without user |
| users | auth_sessions | CASCADE | Sessions should be deleted with user |
| users | conversations | CASCADE | Conversations owned by user |
| users | reading_progress | CASCADE | Progress tied to user |
| users | recommendations | CASCADE | Recommendations specific to user |
| users | audit_logs | SET NULL | Preserve logs for security audit |
| conversations | chat_interactions | CASCADE | Interactions meaningless without conversation |

### Unique Constraints

| Table | Columns | Purpose |
|-------|---------|---------|
| users | email | One account per email |
| reading_progress | user_id, chapter_id | One progress record per user per chapter |

### Check Constraints

| Table | Column | Constraint | Purpose |
|-------|--------|------------|---------|
| user_profiles | experience_level | IN ('Beginner', 'Intermediate', 'Advanced', 'Expert') | Enforce valid levels |
| chat_interactions | query_mode | IN ('global', 'local') | Enforce valid modes |
| reading_progress | completion_percentage | >= 0 AND <= 100 | Valid percentage range |
| recommendations | score | >= 0 AND <= 1 | Valid score range |

---

## Validation Logic (Application Layer)

### User Registration
```python
def validate_signup(email: str, password: str, full_name: str) -> dict:
    errors = {}

    # Email validation
    if not re.match(r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$', email):
        errors['email'] = 'Invalid email format'

    # Password validation
    if len(password) < 8:
        errors['password'] = 'Password must be at least 8 characters'

    # Name validation
    if len(full_name) < 2:
        errors['full_name'] = 'Name must be at least 2 characters'

    return errors
```

### Background Questionnaire
```python
VALID_LANGUAGES = ["Python", "C++", "JavaScript", "Rust", "Go", "Java", "MATLAB", "Julia", "Other"]
VALID_FRAMEWORKS = ["ROS", "ROS 2", "PyTorch", "TensorFlow", "Unity", "Unreal Engine", "Gazebo", "Mujoco", "Isaac Sim", "Other"]

def validate_software_background(languages: list[str], frameworks: list[str]) -> dict:
    errors = {}

    if not languages:
        errors['languages'] = 'Select at least one programming language'

    if any(lang not in VALID_LANGUAGES for lang in languages):
        errors['languages'] = 'Invalid programming language selection'

    if frameworks and any(fw not in VALID_FRAMEWORKS for fw in frameworks):
        errors['frameworks'] = 'Invalid framework selection'

    return errors
```

---

## Migration Strategy

### Initial Schema Creation
```bash
# Run SQL scripts in order:
1. backend/scripts/migrations/001_create_users.sql
2. backend/scripts/migrations/002_create_profiles.sql
3. backend/scripts/migrations/003_create_conversations.sql
4. backend/scripts/migrations/004_create_progress.sql
5. backend/scripts/migrations/005_create_recommendations.sql
6. backend/scripts/migrations/006_create_audit_logs.sql
```

### Version Control with Alembic
```python
# alembic/versions/001_initial_schema.py
def upgrade():
    op.create_table('users', ...)
    op.create_table('user_profiles', ...)
    # ...

def downgrade():
    op.drop_table('audit_logs')
    op.drop_table('recommendations')
    # ... (reverse order)
```

---

## Performance Optimization

### Indexes

**Critical Indexes** (already defined):
- `idx_users_email`: Speeds up signin lookup
- `idx_conversations_user_archived_created`: Composite index for conversation list queries
- `idx_chat_interactions_conversation_id`: Speeds up conversation detail queries
- `idx_reading_progress_user_id`: Speeds up progress dashboard queries
- `idx_recommendations_user_id`, `idx_recommendations_score`: Speeds up recommendation retrieval

**Query Patterns**:
```sql
-- Fast query: Get user's active conversations (uses composite index)
SELECT * FROM conversations
WHERE user_id = ? AND archived = FALSE
ORDER BY created_at DESC
LIMIT 20;

-- Fast query: Get conversation messages (uses conversation_id index)
SELECT * FROM chat_interactions
WHERE conversation_id = ?
ORDER BY created_at ASC;
```

### Connection Pooling
- Min connections: 2
- Max connections: 10
- Connection timeout: 30s
- Idle timeout: 600s (10 min)

---

## Summary

**Total Entities**: 10 Postgres tables + 1 Qdrant collection

**Storage Estimates**:
- Neon Postgres: ~50 MB for 1000 users (with archival)
- Qdrant: ~11 MB for 1000 book chunks
- **Both well within free tier limits**

**Key Design Decisions**:
- Denormalized `user_id` in `chat_interactions` for faster user-level queries
- JSONB arrays for flexible background questionnaire options
- Composite index on conversations for optimal query performance
- CASCADE deletes for data consistency
- Audit logs with SET NULL for security compliance
