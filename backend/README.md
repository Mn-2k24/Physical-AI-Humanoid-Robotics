# Backend - RAG Chatbot & Authentication System

FastAPI backend for an integrated RAG (Retrieval-Augmented Generation) chatbot with user authentication and personalized recommendations.

## 🏗️ Architecture

- **Framework**: FastAPI (Python 3.11+)
- **LLM**: Google Gemini API (`gemini-pro`, `text-embedding-004`)
- **Vector Database**: Qdrant Cloud (HNSW index, 768-dim embeddings)
- **Relational Database**: Neon Serverless Postgres
- **Authentication**: JWT-based sessions with HTTP-only cookies
- **Rate Limiting**: SlowAPI (5 requests/minute for auth endpoints)

---

## 📁 Project Structure

```
backend/
├── src/
│   ├── api/              # REST API endpoints
│   │   ├── auth.py       # Authentication endpoints
│   │   ├── chat.py       # Chatbot endpoints (global/local RAG)
│   │   ├── conversations.py  # Conversation history
│   │   ├── progress.py   # Reading progress tracking
│   │   └── recommendations.py  # Personalized recommendations
│   ├── core/             # Core functionality
│   │   ├── config.py     # Configuration management
│   │   ├── middleware.py # Auth & rate limiting middleware
│   │   └── security.py   # Password hashing, JWT tokens
│   ├── models/           # Pydantic schemas
│   │   ├── auth.py       # AuthSession schema
│   │   ├── conversation.py  # Conversation & ChatInteraction
│   │   ├── progress.py   # ReadingProgress schema
│   │   ├── recommendation.py  # Recommendation schema
│   │   └── user.py       # User, UserProfile schemas
│   ├── services/         # Business logic
│   │   ├── auth_service.py  # Signup, signin, session management
│   │   ├── conversation.py  # Conversation history service
│   │   ├── embedding.py  # Gemini text embeddings
│   │   ├── neon.py       # Postgres connection pool
│   │   ├── qdrant.py     # Qdrant vector store
│   │   ├── rag.py        # RAG retrieval & generation
│   │   └── recommendation_engine.py  # Recommendation algorithm
│   ├── utils/            # Utilities
│   │   ├── chunking.py   # Semantic text chunking
│   │   └── validation.py # Input validation schemas
│   └── main.py           # FastAPI app initialization
├── scripts/
│   ├── migrations/       # Database migrations (SQL)
│   ├── init_db.py        # Database initialization script
│   └── ingest.py         # Book content ingestion script
├── tests/                # Pytest test suite
├── requirements.txt      # Python dependencies
├── pyproject.toml        # Project metadata
└── README.md             # This file
```

---

## 🚀 Quick Start

### Prerequisites

- **Python**: 3.11 or higher
- **Pip**: Latest version
- **Virtual Environment**: Recommended (`venv` or `conda`)
- **Environment Variables**: Copy `.env.example` to `.env` and fill in values

### 1. Clone Repository

```bash
git clone https://github.com/Mn-2k24/Physical-AI-Humanoid-Robotics.git
cd Physical-AI-Humanoid-Robotics/backend
```

### 2. Create Virtual Environment

```bash
python3.11 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### 4. Configure Environment Variables

Copy the `.env.example` file from the project root to `.env` and update the values:

```bash
cp ../.env.example .env
```

**Required Environment Variables**:

| Variable | Description | Example |
|----------|-------------|---------|
| `GEMINI_API_KEY` | Google Gemini API key | `AIzaSy...` |
| `QDRANT_API_KEY` | Qdrant Cloud API key | `eyJhbGci...` |
| `QDRANT_ENDPOINT` | Qdrant Cloud endpoint URL | `https://...qdrant.io` |
| `QDRANT_COLLECTION` | Collection name for book embeddings | `physical_ai_book` |
| `NEON_CONNECTION_STRING` | Neon Postgres connection string | `postgresql://...` |
| `BETTER_AUTH_SECRET` | Secret for JWT signing (≥32 chars) | `your-secret-key...` |
| `FRONTEND_URL` | Frontend URL for CORS | `http://localhost:3000` |

**Optional Variables**:

| Variable | Description | Default |
|----------|-------------|---------|
| `BACKEND_HOST` | Host to bind server | `0.0.0.0` |
| `BACKEND_PORT` | Port to bind server | `8000` |
| `ENVIRONMENT` | Environment mode | `development` |
| `LOG_LEVEL` | Logging level | `INFO` |
| `RATE_LIMIT_PER_MINUTE` | Rate limit for auth endpoints | `5` |
| `SESSION_EXPIRATION_HOURS` | JWT session expiration | `24` |

### 5. Initialize Database

Run the database initialization script to create all tables:

```bash
python scripts/init_db.py
```

This will:
- Create `users`, `user_profiles`, `user_software_background`, `user_hardware_background` tables
- Create `auth_sessions` table
- Create `conversations`, `chat_interactions` tables
- Create `reading_progress`, `recommendations` tables
- Create `audit_logs` table
- Create all necessary indexes

### 6. Ingest Book Content (Optional)

If you have book content in `/docs` directory as Markdown files, ingest it into Qdrant:

```bash
python scripts/ingest.py
```

This will:
- Read all Markdown files from `/docs`
- Chunk content semantically (max 500 tokens per chunk)
- Generate embeddings using Gemini `text-embedding-004`
- Upload chunks to Qdrant with metadata (chapter, section, page)

### 7. Start Development Server

```bash
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

Server will start at: **http://localhost:8000**

- **Interactive API Docs**: http://localhost:8000/docs (Swagger UI)
- **Alternative API Docs**: http://localhost:8000/redoc (ReDoc)
- **Health Check**: http://localhost:8000/health

---

## 📚 API Documentation

### Base URL

```
http://localhost:8000
```

### Endpoints

#### Authentication (`/auth`)

| Method | Endpoint | Description | Rate Limit |
|--------|----------|-------------|------------|
| `POST` | `/auth/signup` | Create new user account | 5/min |
| `POST` | `/auth/signin` | Sign in existing user | 5/min |
| `POST` | `/auth/signout` | Sign out current user | - |
| `GET` | `/auth/me` | Get current user profile | - |
| `POST` | `/auth/refresh` | Refresh JWT token | - |

#### Chat (`/chat`)

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/chat/global` | Ask question with global context (RAG over entire book) |
| `POST` | `/chat/local` | Ask question with local context (RAG over selected text only) |

#### Conversations (`/conversations`)

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/conversations` | Get user's conversation history (last 10 messages) |
| `GET` | `/conversations/{id}` | Get specific conversation by ID |
| `DELETE` | `/conversations/{id}` | Delete conversation |

#### Progress (`/progress`)

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/progress` | Get user's reading progress across all chapters |
| `POST` | `/progress` | Update reading progress for a chapter |

#### Recommendations (`/recommendations`)

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/recommendations` | Get personalized chapter recommendations (3-5 items) |
| `PUT` | `/recommendations/{id}/dismiss` | Dismiss a recommendation |

#### Health (`/health`)

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/health` | Check backend health status |

---

## 🔒 Authentication Flow

### 1. User Signup

```bash
POST /auth/signup
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "securepassword123",
  "full_name": "John Doe",
  "experience_level": "Intermediate",
  "programming_languages": ["Python", "JavaScript"],
  "frameworks": ["React", "FastAPI"],
  "available_hardware": ["NVIDIA Jetson Nano"],
  "robotics_hardware": ["ROS 2", "Isaac Sim"]
}
```

**Response**:
```json
{
  "user_id": "123e4567-e89b-12d3-a456-426614174000",
  "email": "user@example.com",
  "full_name": "John Doe"
}
```

**Cookies Set**:
- `session_token`: HTTP-only, Secure, SameSite=Lax

### 2. User Signin

```bash
POST /auth/signin
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "securepassword123",
  "remember_me": true
}
```

**Response**:
```json
{
  "user_id": "123e4567-e89b-12d3-a456-426614174000",
  "email": "user@example.com",
  "full_name": "John Doe"
}
```

### 3. Protected Endpoints

All endpoints except `/auth/signup` and `/auth/signin` require authentication.

Include the session cookie in requests:
```bash
GET /auth/me
Cookie: session_token=eyJhbGci...
```

---

## 💬 Chat Examples

### Global RAG Query

Ask a question with context from the entire book:

```bash
POST /chat/global
Content-Type: application/json
Cookie: session_token=eyJhbGci...

{
  "query": "What are the main components of ROS 2?",
  "conversation_id": null  # Optional: link to existing conversation
}
```

**Response**:
```json
{
  "answer": "ROS 2 has several main components: 1) Nodes - fundamental building blocks...",
  "sources": [
    {"chapter": "Module 1", "section": "ROS 2 Architecture", "page": 15},
    {"chapter": "Module 1", "section": "Nodes and Topics", "page": 22}
  ],
  "conversation_id": "conv-123"
}
```

### Local RAG Query (Selected Text)

Ask a question about specific selected text:

```bash
POST /chat/local
Content-Type: application/json
Cookie: session_token=eyJhbGci...

{
  "query": "What does this mean?",
  "selected_text": "The DDS middleware provides publish-subscribe messaging...",
  "conversation_id": null
}
```

---

## 🧪 Development

### Run Tests

```bash
pytest
```

### Run Tests with Coverage

```bash
pytest --cov=src --cov-report=html
```

### Code Formatting

```bash
# Format with black
black src/

# Sort imports with isort
isort src/
```

### Type Checking

```bash
mypy src/
```

---

## 🐳 Docker Deployment

See `Dockerfile` and `docker-compose.yml` in the project root.

### Build Docker Image

```bash
docker build -t rag-chatbot-backend .
```

### Run Container

```bash
docker run -p 8000:8000 --env-file .env rag-chatbot-backend
```

---

## 📊 Database Schema

### Users & Profiles

- `users`: User accounts (email, hashed_password, full_name)
- `user_profiles`: User experience level
- `user_software_background`: Programming languages, frameworks
- `user_hardware_background`: Available hardware, robotics platforms

### Authentication

- `auth_sessions`: JWT session tokens (session_id, user_id, token_hash, expires_at)

### Chat & Progress

- `conversations`: Chat conversations (conversation_id, user_id, created_at)
- `chat_interactions`: Individual messages (query, answer, sources, timestamp)
- `reading_progress`: Chapter completion tracking (chapter_id, completion_percentage, time_spent)

### Recommendations

- `recommendations`: Personalized chapter recommendations (recommendation_id, chapter_id, score, reason, dismissed)

### Audit

- `audit_logs`: Authentication events (event_type, user_id, timestamp, ip_address)

---

## 🔧 Troubleshooting

### Database Connection Errors

**Error**: `connection to server failed`

**Solution**:
1. Verify `NEON_CONNECTION_STRING` in `.env`
2. Ensure Neon database is active (check Neon console)
3. Check firewall/network settings

### Qdrant Connection Errors

**Error**: `Unable to connect to Qdrant`

**Solution**:
1. Verify `QDRANT_API_KEY` and `QDRANT_ENDPOINT` in `.env`
2. Ensure Qdrant Cloud cluster is running
3. Check collection name matches `QDRANT_COLLECTION`

### Gemini API Errors

**Error**: `Invalid API key` or `Quota exceeded`

**Solution**:
1. Verify `GEMINI_API_KEY` in `.env`
2. Check API quota in Google Cloud console
3. Ensure billing is enabled for Gemini API

### CORS Errors

**Error**: `Access to fetch ... has been blocked by CORS policy`

**Solution**:
1. Add frontend URL to `CORS_ORIGINS` in `.env`
2. Restart backend server
3. Clear browser cache

---

## 📝 Environment Setup Checklist

Before starting the server, ensure:

- [ ] Python 3.11+ installed
- [ ] Virtual environment created and activated
- [ ] All dependencies installed (`pip install -r requirements.txt`)
- [ ] `.env` file created with all required variables
- [ ] Gemini API key is valid and has quota
- [ ] Qdrant Cloud cluster is running
- [ ] Neon Postgres database is accessible
- [ ] Database tables initialized (`python scripts/init_db.py`)
- [ ] Book content ingested (optional: `python scripts/ingest.py`)

---

## 📖 Additional Resources

- **API Documentation**: http://localhost:8000/docs (when server is running)
- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **Gemini API Docs**: https://ai.google.dev/docs
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **Neon Docs**: https://neon.tech/docs/introduction

---

## 📄 License

See project root LICENSE file.

---

**Last Updated**: 2025-12-14
