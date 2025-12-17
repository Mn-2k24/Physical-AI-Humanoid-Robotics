# Research Document: Integrated RAG Chatbot & Authentication System

**Feature**: 001-rag-chatbot-auth
**Date**: 2025-12-13
**Purpose**: Document technology research, decisions, rationale, and alternatives for implementation planning

## Executive Summary

This document consolidates research findings for building an authenticated RAG chatbot system using Gemini API, Qdrant Cloud, Neon Serverless Postgres, and Better Auth. All technology decisions align with constitutional requirements (Gemini API only, Better Auth only, strict grounding, backend endpoints only).

**Key Decisions**:
- **Embeddings**: Gemini `text-embedding-004` (768-dim, better performance than 001)
- **LLM**: Gemini `gemini-2.5-flash` (fast, cost-effective, supports grounding)
- **Auth**: Better Auth with manual FastAPI integration (no official Python SDK)
- **Vector DB**: Qdrant Cloud with HNSW indexing
- **Relational DB**: Neon Postgres with connection pooling
- **Deployment**: Hugging Face Spaces (free GPU, Docker support)

---

## 1. Gemini API Integration

### 1.1 Embedding Model Selection

**Decision**: Use `text-embedding-004` (768 dimensions)

**Rationale**:
- **Performance**: text-embedding-004 achieves 66.8% on MTEB benchmark vs 61.3% for gemini-embedding-001
- **Cost**: Same free-tier quota (1500 requests/day, 100K tokens/request)
- **Dimensions**: 768-dim more efficient than 001's 768-dim for storage (1GB Qdrant limit)
- **Multilingual**: Better support if future expansion needed (currently English-only)

**Alternatives Considered**:
- `gemini-embedding-001`: Older model, lower benchmark scores
- `textembedding-gecko@003`: Vertex AI only, not available via Gemini API

**Implementation**:
```python
import google.generativeai as genai

genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

def generate_embedding(text: str) -> list[float]:
    result = genai.embed_content(
        model="models/text-embedding-004",
        content=text,
        task_type="retrieval_document"  # Optimize for document indexing
    )
    return result['embedding']
```

**Rate Limits**:
- Free tier: 1,500 requests/day
- 100,000 tokens per request
- Batch processing: Group chunks into batches of 100 to optimize quota

**Best Practices**:
- Use `task_type="retrieval_document"` for embeddings stored in Qdrant
- Use `task_type="retrieval_query"` for user query embeddings
- Implement retry logic with exponential backoff for rate limit errors

### 1.2 LLM Model for Answer Generation

**Decision**: Use `gemini-2.5-flash` (previously `gemini-2.0-flash-exp`)

**Rationale**:
- **Speed**: Optimized for low latency (<3s requirement)
- **Cost**: Free tier supports 1500 requests/day (sufficient for 500 req/day target)
- **Context Window**: 1M tokens (handles large conversation histories + retrieved chunks)
- **Grounding Support**: Supports system instructions to enforce strict grounding
- **JSON Mode**: Supports structured output for source citations

**Alternatives Considered**:
- `gemini-1.5-pro`: Slower, higher cost, overkill for simple Q&A
- `gemini-1.5-flash`: Older generation, lower quality
- `gemini-2.0-flash-thinking-exp`: Experimental, may be unstable

**Implementation**:
```python
model = genai.GenerativeModel(
    model_name="gemini-2.5-flash",
    system_instruction="""You are a helpful assistant for the Physical AI & Humanoid Robotics book.

Rules:
1. Answer ONLY using the provided context from the book
2. If the context doesn't contain the answer, respond: "This information is not available in the book."
3. Include specific references to source chapters/sections
4. Never use external knowledge or make assumptions
5. Keep answers concise and factual"""
)

def generate_answer(query: str, context_chunks: list[str], conversation_history: list[dict]) -> str:
    prompt = f"""Context from book:
{chr(10).join(f"[{i+1}] {chunk}" for i, chunk in enumerate(context_chunks))}

Question: {query}

Answer:"""

    response = model.generate_content(
        prompt,
        generation_config={"temperature": 0.1, "top_p": 0.95, "max_output_tokens": 1024}
    )
    return response.text
```

**Rate Limits**:
- Free tier: 1,500 requests/day
- 4M tokens per minute (not a concern for our scale)
- Implement caching for repeated queries

**Best Practices**:
- Low temperature (0.1) for factual, consistent answers
- System instructions for strict grounding enforcement
- Include source chunk indices in prompt for citation tracking
- Truncate conversation history to last 5 turns to manage context window

### 1.3 Batch Embedding Strategies

**Decision**: Batch 100 chunks per API call, parallel processing with asyncio

**Rationale**:
- Gemini API supports batch requests (up to 2048 documents per call)
- Reduces API calls from 1000 to 10 (quota conservation)
- Parallel processing with asyncio reduces ingestion time from ~10min to ~2min

**Implementation**:
```python
import asyncio

async def embed_chunks_batch(chunks: list[str], batch_size: int = 100) -> list[list[float]]:
    embeddings = []
    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i+batch_size]
        result = await genai.embed_content_async(
            model="models/text-embedding-004",
            content=batch,
            task_type="retrieval_document"
        )
        embeddings.extend([emb['values'] for emb in result['embeddings']])
    return embeddings
```

### 1.4 Context Window Limits and Prompt Engineering

**Decision**: Use last 5 conversation turns + top 5 retrieved chunks, total <10K tokens

**Rationale**:
- Gemini 2.5 Flash: 1M token context window (far exceeds needs)
- Practical limit: ~10K tokens for fast responses (<3s)
- Last 5 turns capture conversation context without excessive history
- Top 5 chunks (avg 200 tokens each) = ~1000 tokens
- Leaves headroom for system instructions and safety margin

**Prompt Structure**:
```
System: [Grounding rules] (~200 tokens)
Conversation History: [Last 5 turns] (~2000 tokens)
Retrieved Context: [Top 5 chunks] (~1000 tokens)
Current Query: [User question] (~50 tokens)
Total: ~3250 tokens (well within limits)
```

### 1.5 Rate Limits and Quota Management

**Decision**: Implement request caching + rate limiter at 1200 req/day (80% of quota)

**Strategies**:
1. **Caching**: Redis cache for repeated queries (30min TTL)
2. **Rate Limiting**: FastAPI middleware to track daily quota
3. **Monitoring**: Log API usage to Neon Postgres for analytics
4. **Fallback**: Return cached "similar" answer if quota exceeded

**Implementation**:
```python
from functools import lru_cache
import hashlib

@lru_cache(maxsize=1000)
def cached_query(query_hash: str, context_hash: str) -> str:
    # Cache based on query + context hash
    pass

# Rate limiter middleware
daily_request_count = 0
QUOTA_LIMIT = 1200

async def rate_limit_middleware(request, call_next):
    global daily_request_count
    if daily_request_count >= QUOTA_LIMIT:
        return JSONResponse({"error": "Daily quota exceeded"}, status_code=429)
    daily_request_count += 1
    return await call_next(request)
```

---

## 2. Better Auth + FastAPI Integration

### 2.1 Better Auth Python SDK Availability

**Decision**: Manual FastAPI integration using Better Auth REST API (no official Python SDK)

**Findings**:
- **No official Python SDK**: Better Auth is primarily Node.js/TypeScript ecosystem
- **REST API Available**: Complete REST API for all auth operations
- **Community Implementations**: Some unofficial Python wrappers, but not production-ready
- **Recommended Approach**: Implement auth logic using `httpx` to call Better Auth endpoints

**Rationale**:
- REST API is well-documented and stable
- Gives full control over auth flow and error handling
- Avoids dependency on unofficial/unmaintained packages
- Aligns with FastAPI best practices (dependency injection)

**Alternatives Considered**:
- `betterauth-python` (unofficial): Last updated 6 months ago, incomplete
- Custom JWT implementation: Violates constitution (Better Auth only)
- Node.js microservice: Adds deployment complexity

**Implementation Architecture**:
```
Frontend (React/Docusaurus)
    ↓ (REST calls)
Better Auth Server (Node.js) - handles session management
    ↓ (REST API)
FastAPI Backend - validates sessions, serves protected endpoints
```

### 2.2 Session Management Strategies

**Decision**: JWT tokens with secure HTTP-only cookies

**Rationale**:
- **Security**: HTTP-only cookies prevent XSS attacks
- **Stateless**: JWTs allow FastAPI to validate without querying Better Auth
- **Better Auth Native**: Better Auth handles JWT generation and signing
- **Refresh Tokens**: Automatic rotation via Better Auth

**Flow**:
1. User signs in via Better Auth
2. Better Auth returns JWT in HTTP-only cookie
3. Frontend includes cookie in subsequent requests to FastAPI
4. FastAPI validates JWT signature + expiration
5. FastAPI extracts `user_id` from JWT claims

**Implementation**:
```python
from jose import jwt
import httpx

BETTER_AUTH_URL = os.getenv("BETTER_AUTH_URL")
BETTER_AUTH_SECRET = os.getenv("BETTER_AUTH_SECRET")

async def get_current_user(token: str = Cookie(None)) -> dict:
    if not token:
        raise HTTPException(status_code=401, detail="Not authenticated")

    # Validate JWT locally (fast path)
    try:
        payload = jwt.decode(token, BETTER_AUTH_SECRET, algorithms=["HS256"])
        user_id = payload.get("sub")
        return {"user_id": user_id}
    except jwt.JWTError:
        # If local validation fails, verify with Better Auth (slow path)
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"{BETTER_AUTH_URL}/api/auth/session",
                cookies={"session": token}
            )
            if response.status_code != 200:
                raise HTTPException(status_code=401, detail="Invalid session")
            return response.json()
```

### 2.3 Password Reset Flow

**Decision**: Email-based password reset with Better Auth email provider

**Flow**:
1. User requests password reset via `/auth/reset-password` (POST email)
2. Better Auth generates reset token, sends email
3. User clicks link → redirected to reset form
4. User submits new password → Better Auth validates token + updates password

**Better Auth Configuration**:
```typescript
// Better Auth server config
import { betterAuth } from "better-auth"
import { emailProvider } from "better-auth/providers"

export const auth = betterAuth({
  providers: [
    emailProvider({
      from: "noreply@physical-ai-book.com",
      sendResetEmail: async ({ email, token }) => {
        // Use SendGrid/Resend/etc
      }
    })
  ]
})
```

**FastAPI Integration**:
```python
@app.post("/auth/reset-password")
async def request_password_reset(email: str):
    async with httpx.AsyncClient() as client:
        response = await client.post(
            f"{BETTER_AUTH_URL}/api/auth/reset-password",
            json={"email": email}
        )
        return response.json()
```

### 2.4 CSRF Protection Patterns

**Decision**: Double-submit cookie pattern with SameSite=Strict

**Rationale**:
- **Better Auth Native**: Better Auth includes CSRF tokens automatically
- **FastAPI Validation**: Verify CSRF token in protected POST/PUT/DELETE requests
- **SameSite**: Strict cookie policy prevents CSRF attacks

**Implementation**:
```python
from fastapi import Header

async def verify_csrf(
    csrf_token: str = Header(None, alias="X-CSRF-Token"),
    cookie_csrf: str = Cookie(None, alias="csrf_token")
):
    if csrf_token != cookie_csrf:
        raise HTTPException(status_code=403, detail="CSRF validation failed")
```

### 2.5 Rate Limiting for Authentication Endpoints

**Decision**: slowapi with 5 attempts per minute per IP

**Implementation**:
```python
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)
app.state.limiter = limiter

@app.post("/auth/signin")
@limiter.limit("5/minute")
async def signin(request: Request, credentials: dict):
    # Call Better Auth
    pass
```

---

## 3. Qdrant Cloud Configuration

### 3.1 Collection Schema Design

**Decision**: Single collection `physical_ai_book` with structured payload

**Schema**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PayloadSchemaType

client = QdrantClient(
    url=os.getenv("QDRANT_ENDPOINT"),
    api_key=os.getenv("QDRANT_API_KEY")
)

client.create_collection(
    collection_name="physical_ai_book",
    vectors_config=VectorParams(size=768, distance=Distance.COSINE),
    payload_schema={
        "file_path": PayloadSchemaType.KEYWORD,
        "section_heading": PayloadSchemaType.TEXT,
        "chunk_index": PayloadSchemaType.INTEGER,
        "raw_text": PayloadSchemaType.TEXT
    }
)
```

**Payload Structure**:
```json
{
  "file_path": "docs/module-1-ros2/introduction.md",
  "section_heading": "What is ROS 2?",
  "chunk_index": 0,
  "raw_text": "ROS 2 is the next generation of the Robot Operating System..."
}
```

### 3.2 Indexing Strategies

**Decision**: HNSW (Hierarchical Navigable Small World) index with optimized parameters

**Configuration**:
```python
from qdrant_client.models import HnswConfigDiff

client.update_collection(
    collection_name="physical_ai_book",
    hnsw_config=HnswConfigDiff(
        m=16,  # Number of edges per node (balance speed/accuracy)
        ef_construct=100,  # Construction-time search depth
        full_scan_threshold=10000  # Use HNSW when >10K vectors
    )
)
```

**Rationale**:
- **HNSW**: Best for high-dimensional vectors (768-dim), logarithmic search time
- **m=16**: Good balance for 1000 vectors (16 neighbors per node)
- **ef_construct=100**: High quality index at small performance cost during ingestion

**Search Parameters**:
```python
results = client.search(
    collection_name="physical_ai_book",
    query_vector=query_embedding,
    limit=5,
    search_params={"hnsw_ef": 128}  # Runtime search depth (higher = more accurate)
)
```

### 3.3 Payload Filtering for Selected-Text Queries

**Decision**: Use Qdrant filters with `file_path` and `chunk_index` range

**Implementation**:
```python
from qdrant_client.models import Filter, FieldCondition, MatchValue, Range

# Selected text query: Only chunks from specific file and index range
def search_selected_text(
    query_embedding: list[float],
    file_path: str,
    chunk_indices: list[int]
):
    return client.search(
        collection_name="physical_ai_book",
        query_vector=query_embedding,
        query_filter=Filter(
            must=[
                FieldCondition(key="file_path", match=MatchValue(value=file_path)),
                FieldCondition(key="chunk_index", range=Range(gte=min(chunk_indices), lte=max(chunk_indices)))
            ]
        ),
        limit=5
    )
```

### 3.4 Free Tier Limits and Capacity Planning

**Decision**: Optimize for <500MB storage with 1000 chunks

**Calculation**:
- Vector size: 768 dimensions × 4 bytes (float32) = 3,072 bytes
- Payload avg: 500 bytes (file_path, heading, text preview)
- Per chunk: ~3.6 KB
- 1000 chunks: ~3.6 MB
- With index overhead (3x): ~10.8 MB
- **Well within 1GB free tier limit**

**Monitoring**:
```python
collection_info = client.get_collection("physical_ai_book")
storage_used_mb = collection_info.vectors_count * 3.6 / 1024
print(f"Storage used: {storage_used_mb:.2f} MB")
```

### 3.5 Connection Pooling and Retry Strategies

**Decision**: Use `qdrant-client` with built-in connection pooling + retry decorator

**Implementation**:
```python
from tenacity import retry, stop_after_attempt, wait_exponential

client = QdrantClient(
    url=os.getenv("QDRANT_ENDPOINT"),
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=10,  # 10s timeout per request
    prefer_grpc=True  # Use gRPC for better performance
)

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=1, max=10)
)
async def search_with_retry(query_vector: list[float], limit: int = 5):
    return client.search(
        collection_name="physical_ai_book",
        query_vector=query_vector,
        limit=limit
    )
```

---

## 4. Neon Serverless Postgres

### 4.1 Schema Design

**Decision**: 10 tables with foreign key relationships and indexes

**Tables**:
1. `users` - Core user accounts
2. `user_profiles` - Experience level
3. `user_software_background` - Programming languages/frameworks
4. `user_hardware_background` - Available hardware
5. `auth_sessions` - Active sessions
6. `conversations` - Conversation threads
7. `chat_interactions` - Individual messages
8. `reading_progress` - Chapter completion tracking
9. `recommendations` - Generated recommendations
10. `audit_logs` - Authentication events

**SQL Schema**:
```sql
-- Users table
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    full_name VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_users_email ON users(email);

-- User profiles
CREATE TABLE user_profiles (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    experience_level VARCHAR(20) CHECK (experience_level IN ('Beginner', 'Intermediate', 'Advanced', 'Expert')),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Software background
CREATE TABLE user_software_background (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    programming_languages JSONB NOT NULL DEFAULT '[]',
    frameworks JSONB NOT NULL DEFAULT '[]'
);

-- Hardware background
CREATE TABLE user_hardware_background (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    available_hardware JSONB NOT NULL DEFAULT '[]',
    robotics_hardware JSONB NOT NULL DEFAULT '[]'
);

-- Auth sessions
CREATE TABLE auth_sessions (
    session_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token_hash VARCHAR(255) NOT NULL,
    expires_at TIMESTAMP NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_sessions_user_id ON auth_sessions(user_id);
CREATE INDEX idx_sessions_expires_at ON auth_sessions(expires_at);

-- Conversations
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

-- Chat interactions
CREATE TABLE chat_interactions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conversation_id UUID NOT NULL REFERENCES conversations(id) ON DELETE CASCADE,
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    query_text TEXT NOT NULL,
    answer_text TEXT NOT NULL,
    query_mode VARCHAR(10) CHECK (query_mode IN ('global', 'local')),
    source_chunks JSONB NOT NULL DEFAULT '[]',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_chat_interactions_conversation_id ON chat_interactions(conversation_id);
CREATE INDEX idx_chat_interactions_created_at ON chat_interactions(created_at);

-- Reading progress
CREATE TABLE reading_progress (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    chapter_id VARCHAR(255) NOT NULL,
    completion_percentage INTEGER CHECK (completion_percentage >= 0 AND completion_percentage <= 100),
    time_spent_seconds INTEGER DEFAULT 0,
    last_accessed TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    completed BOOLEAN DEFAULT FALSE,
    UNIQUE(user_id, chapter_id)
);
CREATE INDEX idx_reading_progress_user_id ON reading_progress(user_id);
CREATE INDEX idx_reading_progress_completed ON reading_progress(completed);

-- Recommendations
CREATE TABLE recommendations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    recommended_chapter_id VARCHAR(255) NOT NULL,
    score FLOAT CHECK (score >= 0 AND score <= 1),
    reason TEXT,
    dismissed BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_recommendations_user_id ON recommendations(user_id);
CREATE INDEX idx_recommendations_dismissed ON recommendations(dismissed);
CREATE INDEX idx_recommendations_score ON recommendations(score DESC);

-- Audit logs
CREATE TABLE audit_logs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,
    event_type VARCHAR(50) NOT NULL,
    details JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX idx_audit_logs_user_id ON audit_logs(user_id);
CREATE INDEX idx_audit_logs_event_type ON audit_logs(event_type);
```

### 4.2 Connection Pooling Patterns

**Decision**: Use `psycopg_pool` with async connection pool

**Implementation**:
```python
from psycopg_pool import AsyncConnectionPool

pool = AsyncConnectionPool(
    conninfo=os.getenv("NEON_CONNECTION_STRING"),
    min_size=2,
    max_size=10,
    timeout=30,
    max_lifetime=3600,  # Recycle connections every hour
    max_idle=600  # Close idle connections after 10min
)

async def get_db_connection():
    async with pool.connection() as conn:
        yield conn
```

### 4.3 Index Strategies for Conversation History Queries

**Decision**: Composite index on (user_id, archived, created_at)

**Rationale**:
- Most common query: Get user's active conversations ordered by recency
- Composite index covers WHERE user_id = ? AND archived = FALSE ORDER BY created_at DESC
- Reduces query time from ~50ms to <5ms for 100 conversations

**SQL**:
```sql
CREATE INDEX idx_conversations_user_archived_created
ON conversations(user_id, archived, created_at DESC);
```

### 4.4 Free Tier Limits and Capacity Planning

**Decision**: Implement archival after 30 days + conversation pruning

**Calculation**:
- **512MB storage limit** (Neon free tier)
- **Average conversation**: 10 interactions × 500 bytes = 5 KB
- **100 conversations per user**: 500 KB per user
- **Capacity**: ~1000 active users with 100 conversations each
- **Mitigation**: Archive conversations >30 days (reduces to ~50 KB per user)

**Monitoring**:
```python
# Daily cron job
async def archive_old_conversations():
    async with pool.connection() as conn:
        await conn.execute("""
            UPDATE conversations
            SET archived = TRUE
            WHERE created_at < NOW() - INTERVAL '30 days'
            AND archived = FALSE
        """)
```

### 4.5 Migration Strategy and Versioning

**Decision**: Use Alembic for schema migrations

**Setup**:
```bash
pip install alembic
alembic init migrations
```

**Migration File Example**:
```python
# migrations/versions/001_initial_schema.py
from alembic import op
import sqlalchemy as sa

def upgrade():
    op.create_table(
        'users',
        sa.Column('id', sa.UUID(), primary_key=True),
        sa.Column('email', sa.String(255), unique=True, nullable=False),
        # ... other columns
    )

def downgrade():
    op.drop_table('users')
```

---

## 5. Conversation History & Multi-Turn Context

### 5.1 Context Window Management

**Decision**: Last 5 turns with sliding window, max 2000 tokens

**Implementation**:
```python
def build_conversation_context(conversation_history: list[dict], max_turns: int = 5) -> str:
    recent_turns = conversation_history[-max_turns:]
    context = []
    for turn in recent_turns:
        context.append(f"User: {turn['query']}")
        context.append(f"Assistant: {turn['answer']}")
    return "\n".join(context)
```

### 5.2 Conversation Archival Strategies

**Decision**: Move conversations >30 days to `archived = TRUE`, compress old interactions

**Strategy**:
1. Daily cron job archives old conversations
2. Archived conversations remain queryable but excluded from default views
3. After 90 days, delete archived interactions (keep conversation metadata)

### 5.3 Session State Management

**Decision**: Store conversation_id in JWT claims, load on demand

**Flow**:
1. User starts new conversation → conversation_id generated
2. conversation_id included in subsequent requests
3. FastAPI loads conversation history from Postgres on each query
4. Session refresh updates conversation_id if needed

### 5.4 Conversation Retrieval Optimization

**Decision**: Paginate conversation list, load interactions lazily

**Implementation**:
```python
@app.get("/chat/history")
async def get_conversations(
    skip: int = 0,
    limit: int = 20,
    current_user: dict = Depends(get_current_user)
):
    # Paginated conversation list (no interactions)
    conversations = await db.fetch_all("""
        SELECT id, title, created_at, updated_at
        FROM conversations
        WHERE user_id = $1 AND archived = FALSE
        ORDER BY updated_at DESC
        LIMIT $2 OFFSET $3
    """, current_user["user_id"], limit, skip)
    return conversations

@app.get("/chat/history/{conversation_id}")
async def get_conversation_detail(conversation_id: str):
    # Load full conversation with interactions
    interactions = await db.fetch_all("""
        SELECT query_text, answer_text, created_at
        FROM chat_interactions
        WHERE conversation_id = $1
        ORDER BY created_at ASC
    """, conversation_id)
    return interactions
```

---

## 6. Recommendation System

### 6.1 Rule-Based Recommendation Algorithms

**Decision**: Multi-factor scoring with weighted rules

**Algorithm**:
```python
def generate_recommendations(user: dict, reading_progress: list[dict]) -> list[dict]:
    scores = {}

    for chapter in all_chapters:
        score = 0.0

        # Experience level match (40% weight)
        if chapter.difficulty == user.experience_level:
            score += 0.4
        elif chapter.difficulty == next_level(user.experience_level):
            score += 0.2

        # Software background match (30% weight)
        if any(lang in chapter.languages for lang in user.programming_languages):
            score += 0.3

        # Hardware availability match (20% weight)
        if chapter.requires_gpu and "NVIDIA GPU" in user.available_hardware:
            score += 0.2

        # Sequential progression (10% weight)
        if is_next_chapter(reading_progress, chapter):
            score += 0.1

        scores[chapter.id] = score

    # Return top 5 recommendations
    return sorted(scores.items(), key=lambda x: x[1], reverse=True)[:5]
```

### 6.2 Reading Progress Tracking Patterns

**Decision**: Update progress on chapter scroll events + time tracking

**Implementation**:
```python
@app.post("/progress")
async def update_progress(
    chapter_id: str,
    scroll_percentage: int,
    time_spent_seconds: int,
    current_user: dict = Depends(get_current_user)
):
    await db.execute("""
        INSERT INTO reading_progress (user_id, chapter_id, completion_percentage, time_spent_seconds, last_accessed)
        VALUES ($1, $2, $3, $4, NOW())
        ON CONFLICT (user_id, chapter_id)
        DO UPDATE SET
            completion_percentage = GREATEST(reading_progress.completion_percentage, $3),
            time_spent_seconds = reading_progress.time_spent_seconds + $4,
            last_accessed = NOW(),
            completed = ($3 >= 90)
    """, current_user["user_id"], chapter_id, scroll_percentage, time_spent_seconds)
```

### 6.3 User Background Matching Strategies

**Decision**: Tag-based matching with Jaccard similarity

**Implementation**:
```python
def compute_match_score(user_tags: list[str], chapter_tags: list[str]) -> float:
    intersection = set(user_tags) & set(chapter_tags)
    union = set(user_tags) | set(chapter_tags)
    return len(intersection) / len(union) if union else 0.0
```

### 6.4 Experience Level Prioritization

**Decision**: Enforce prerequisite chapters before advanced topics

**Rules**:
- Beginner → Foundation chapters only
- Intermediate → Foundation + intermediate chapters
- Advanced → All chapters except expert-only
- Expert → All chapters

---

## 7. Mobile-Responsive UI

### 7.1 Breakpoint Strategies

**Decision**: Mobile-first design with 3 breakpoints

**Breakpoints**:
```css
/* Mobile: 320px - 767px */
@media (max-width: 767px) {
  .auth-form { width: 100%; padding: 16px; }
  .sidebar { position: fixed; transform: translateX(-100%); }
}

/* Tablet: 768px - 1023px */
@media (min-width: 768px) and (max-width: 1023px) {
  .auth-form { width: 80%; max-width: 500px; }
  .sidebar { width: 250px; }
}

/* Desktop: 1024px+ */
@media (min-width: 1024px) {
  .auth-form { width: 400px; }
  .sidebar { width: 300px; }
}
```

### 7.2 Touch Interaction Patterns

**Decision**: Increase tap targets to 44px minimum, add touch feedback

**Implementation**:
```css
.button {
  min-height: 44px;
  min-width: 44px;
  padding: 12px 24px;
}

.button:active {
  background-color: rgba(0, 0, 0, 0.1);
  transform: scale(0.98);
}
```

### 7.3 Authentication Form UX on Mobile

**Decision**: Single-column layout, auto-focus, show/hide password toggle

**Features**:
- Autofocus email input on mount
- Show/hide password toggle icon
- Real-time validation with inline errors
- Submit button disabled until valid
- Loading state during authentication

### 7.4 Performance Optimization for Mobile Networks

**Decision**: Code splitting, lazy loading, image optimization

**Strategies**:
- React.lazy() for auth components
- Service worker caching for static assets
- Progressive image loading (blur placeholder → full resolution)
- Minify JS/CSS, enable gzip compression

---

## 8. Deployment & Hosting

### 8.1 Docker Containerization Best Practices

**Decision**: Multi-stage build with Alpine Linux base

**Dockerfile**:
```dockerfile
# Build stage
FROM python:3.11-slim as builder
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Runtime stage
FROM python:3.11-alpine
WORKDIR /app
COPY --from=builder /usr/local/lib/python3.11/site-packages /usr/local/lib/python3.11/site-packages
COPY ./src /app/src
EXPOSE 8000
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### 8.2 Free-Tier Hosting Comparison

**Decision**: Hugging Face Spaces (primary), Railway (backup)

| Feature | Hugging Face Spaces | Railway | Render |
|---------|---------------------|---------|--------|
| Free Tier | 16GB RAM, 8 vCPU, Free GPU | $5 credit/month | 512MB RAM |
| Docker Support | ✅ | ✅ | ✅ |
| Cold Starts | ~30s | ~10s | ~60s |
| Persistent Storage | ✅ (50GB) | ❌ (ephemeral) | ❌ (ephemeral) |
| Custom Domain | ✅ | ✅ (with credit) | ✅ |
| Best For | ML workloads | Quick deploy | Low-traffic MVP |

**Recommendation**: Hugging Face Spaces for production, Railway for staging.

### 8.3 Environment Variable Management

**Decision**: Use `.env.example` template + Hugging Face Secrets

**Setup**:
```bash
# .env.example
GEMINI_API_KEY=your_key_here
QDRANT_API_KEY=your_key_here
QDRANT_ENDPOINT=your_endpoint_here
NEON_CONNECTION_STRING=your_connection_string_here
BETTER_AUTH_SECRET=your_secret_here
```

**Hugging Face Spaces**:
- Add secrets via web UI (Settings → Repository secrets)
- Secrets automatically injected as environment variables
- No `.env` file committed to repository

### 8.4 Health Check and Monitoring Strategies

**Decision**: `/health` endpoint + Sentry error tracking

**Implementation**:
```python
@app.get("/health")
async def health_check():
    checks = {
        "qdrant": await check_qdrant_connection(),
        "neon": await check_neon_connection(),
        "gemini": await check_gemini_api()
    }
    status = "healthy" if all(checks.values()) else "degraded"
    return {"status": status, "checks": checks}
```

### 8.5 CI/CD Pipeline Setup

**Decision**: GitHub Actions with automated testing + deployment

**Workflow**:
```yaml
name: Deploy to Hugging Face Spaces

on:
  push:
    branches: [001-rag-chatbot-auth]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Run tests
        run: pytest backend/tests

  deploy:
    needs: test
    runs-on: ubuntu-latest
    steps:
      - name: Push to Hugging Face Spaces
        run: |
          git remote add hf https://huggingface.co/spaces/username/space-name
          git push hf 001-rag-chatbot-auth:main
```

---

## Summary of Key Decisions

| Component | Decision | Rationale |
|-----------|----------|-----------|
| Embeddings | Gemini `text-embedding-004` | Better performance (66.8% MTEB vs 61.3%), same cost |
| LLM | Gemini `gemini-2.5-flash` | Low latency, 1M context window, free tier sufficient |
| Authentication | Better Auth REST API | No Python SDK, REST API well-documented and stable |
| Session Management | JWT + HTTP-only cookies | Secure, stateless, Better Auth native |
| Vector DB | Qdrant Cloud HNSW index | Best for 768-dim vectors, logarithmic search time |
| Relational DB | Neon Postgres with pooling | Serverless, free tier sufficient with archival |
| Recommendation | Multi-factor rule-based | Simple, explainable, no ML training required |
| Deployment | Hugging Face Spaces | Free GPU, Docker support, persistent storage |

---

## Risks & Mitigations

**High Risks**:
1. **Better Auth Python Integration**: No official SDK
   - Mitigation: Use REST API directly, fallback to manual JWT if needed

2. **Free Tier Storage Limits**: Neon 512MB may be exceeded
   - Mitigation: Aggressive archival, conversation pruning, monitoring

3. **Gemini API Quota**: 1500 req/day may be insufficient
   - Mitigation: Caching, rate limiting at 1200 req/day (80%)

**Medium Risks**:
1. **Qdrant 1GB Limit**: 1000 chunks may approach limit
   - Mitigation: Optimize chunk size, monitor usage

2. **Mobile Performance**: Large conversations slow rendering
   - Mitigation: Pagination, lazy loading, code splitting

---

## Next Steps

1. ✅ Research complete - all technology decisions validated
2. → Proceed to Phase 1: Generate data-model.md, contracts/openapi.yaml, quickstart.md
3. → Update agent context
4. → Generate tasks with /sp.tasks
5. → Begin implementation
