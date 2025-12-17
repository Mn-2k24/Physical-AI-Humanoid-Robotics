# Quickstart Guide - RAG Chatbot & Authentication System

**Feature**: 001-rag-chatbot-auth
**Version**: 1.0.0
**Last Updated**: 2025-12-13

This guide will help you set up and run the integrated RAG chatbot and authentication system locally in under 15 minutes.

---

## Prerequisites

Before you begin, ensure you have the following installed:

- **Python 3.11+** ([Download](https://www.python.org/downloads/))
- **Node.js 18+** and npm ([Download](https://nodejs.org/))
- **Git** ([Download](https://git-scm.com/downloads))
- **Docker** (optional, for containerized deployment) ([Download](https://www.docker.com/))

**Required Accounts** (Free Tier):
- [Google AI Studio](https://ai.google.dev/) - for Gemini API key
- [Qdrant Cloud](https://cloud.qdrant.io/) - for vector database
- [Neon](https://neon.tech/) - for serverless Postgres

---

## Step 1: Clone Repository

```bash
git clone <repository-url>
cd Physical-AI-Humanoid-Robotics
git checkout 001-rag-chatbot-auth
```

---

## Step 2: Environment Setup

### 2.1 Create Environment File

Create a `.env` file in the project root:

```bash
# Backend/.env
GEMINI_API_KEY=AIzaSyBckRsTQQUF_WFB0lQ6RfOQocuPRBgNQrw

# Qdrant Configuration
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJodHRwczovL2NvcmUucWRyYW50LmlvIiwic3ViIjoiZmZlNzc5ZmQtNTVhZi00MGJlLWJiOTItMjE2NjNkMWQ3MTYyIiwiZXhwIjoxNzY4MTI3MzE1LCJpYXQiOjE3MzY1OTEzMTV9.pNbXhc3MvvTbE01yGEVbI4cDHLlTJeZrpjYvgqKpbTw
QDRANT_ENDPOINT=https://0de0ce6a-6b92-487d-9d50-52a358d48738.europe-west3-0.gcp.cloud.qdrant.io

# Neon Postgres Configuration
NEON_CONNECTION_STRING=postgresql://neondb_owner:npg_ktgZ4va0zuPF@ep-crimson-king-a5z9l9l0.us-east-2.aws.neon.tech/neondb?sslmode=require

# Better Auth Configuration
BETTER_AUTH_SECRET=your-secret-key-minimum-32-characters-long
BETTER_AUTH_URL=http://localhost:8000/auth
FRONTEND_URL=http://localhost:3000

# Application Configuration
ENVIRONMENT=development
LOG_LEVEL=INFO
MAX_CONVERSATION_MESSAGES=10
RECOMMENDATION_LIMIT=5
```

**IMPORTANT**: Generate a secure `BETTER_AUTH_SECRET` using:
```bash
python -c "import secrets; print(secrets.token_urlsafe(32))"
```

### 2.2 Verify Credentials

Test your configuration:

```bash
# Test Gemini API
curl -X POST "https://generativelanguage.googleapis.com/v1beta/models/text-embedding-004:embedContent?key=$GEMINI_API_KEY" \
  -H 'Content-Type: application/json' \
  -d '{"content": {"parts": [{"text": "Hello"}]}}'

# Test Qdrant (should return cluster info)
curl -X GET "$QDRANT_ENDPOINT/collections" \
  -H "api-key: $QDRANT_API_KEY"

# Test Neon Postgres
psql "$NEON_CONNECTION_STRING" -c "SELECT version();"
```

---

## Step 3: Backend Setup

### 3.1 Install Dependencies

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

**requirements.txt**:
```txt
fastapi==0.104.1
uvicorn[standard]==0.24.0
google-generativeai==0.3.0
qdrant-client==1.7.0
psycopg2-binary==2.9.9
pydantic==2.5.0
pydantic-settings==2.1.0
python-jose[cryptography]==3.3.0
passlib[bcrypt]==1.7.4
python-multipart==0.0.6
aiohttp==3.9.1
pytest==7.4.3
pytest-asyncio==0.21.1
httpx==0.25.2
```

### 3.2 Initialize Database

```bash
# Run database migrations
python scripts/init_db.py

# Verify tables created
psql "$NEON_CONNECTION_STRING" -c "\dt"
```

**Expected Output**:
```
             List of relations
 Schema |         Name          | Type  |    Owner
--------+-----------------------+-------+--------------
 public | users                 | table | neondb_owner
 public | conversations         | table | neondb_owner
 public | messages              | table | neondb_owner
 public | reading_progress      | table | neondb_owner
 public | recommendations       | table | neondb_owner
 public | user_profiles         | table | neondb_owner
```

### 3.3 Ingest Book Content

```bash
# Ingest book chapters into Qdrant
python scripts/ingest_book.py --book-path ../book.epub

# Verify ingestion (should show book_chapters collection)
curl -X GET "$QDRANT_ENDPOINT/collections/book_chapters" \
  -H "api-key: $QDRANT_API_KEY"
```

**Expected Output**:
```json
{
  "result": {
    "status": "green",
    "vectors_count": 250,
    "points_count": 250
  }
}
```

### 3.4 Run Backend Server

```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

**Verify**: Open http://localhost:8000/docs to see Swagger UI with all 18 API endpoints.

---

## Step 4: Frontend Setup

### 4.1 Install Dependencies

```bash
cd frontend
npm install
```

**Key Dependencies**:
- React 18
- Better Auth React SDK
- TailwindCSS (for mobile responsiveness)
- React Query (for API state management)

### 4.2 Configure Better Auth

Create `frontend/src/lib/auth.ts`:

```typescript
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_BACKEND_URL || "http://localhost:8000",
});

export const { signIn, signUp, signOut, useSession } = authClient;
```

### 4.3 Run Frontend Server

```bash
npm start
```

**Verify**: Open http://localhost:3000 to see the application.

---

## Step 5: Test the System

### 5.1 Create Test User

```bash
curl -X POST http://localhost:8000/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "Test123!@#",
    "full_name": "Test User",
    "experience_level": "beginner",
    "primary_goals": ["understanding_fundamentals"],
    "learning_pace": "moderate",
    "background": ["cs_fundamentals"]
  }'
```

### 5.2 Test Authentication

```bash
curl -X POST http://localhost:8000/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "Test123!@#"
  }'
```

**Expected Response**:
```json
{
  "user": {
    "id": "uuid-here",
    "email": "test@example.com",
    "full_name": "Test User"
  },
  "session": {
    "token": "jwt-token-here",
    "expires_at": "2025-12-14T00:00:00Z"
  }
}
```

### 5.3 Test RAG Query (Global Search)

```bash
curl -X POST http://localhost:8000/chat/global \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <jwt-token>" \
  -d '{
    "query": "What are the main challenges in physical AI?",
    "conversation_id": null
  }'
```

**Expected Response**:
```json
{
  "answer": "The main challenges in physical AI include...",
  "sources": [
    {
      "chapter": "Chapter 1: Introduction to Physical AI",
      "page_range": "12-15",
      "relevance_score": 0.89
    }
  ],
  "conversation_id": "uuid-here"
}
```

### 5.4 Test Recommendations

```bash
curl -X GET http://localhost:8000/recommendations \
  -H "Authorization: Bearer <jwt-token>"
```

**Expected Response**:
```json
{
  "recommendations": [
    {
      "chapter_id": "ch03",
      "title": "Neural Networks for Robotics",
      "reason": "Matches your background in CS fundamentals",
      "priority": 0.95
    }
  ]
}
```

---

## Step 6: Run Tests

### 6.1 Backend Tests

```bash
cd backend
pytest tests/ -v --cov=app --cov-report=html
```

**Expected Coverage**: >80% for all modules

### 6.2 Contract Tests

```bash
pytest tests/contract/ -v
```

**Tests**:
- OpenAPI schema validation
- Request/response format verification
- Authentication flow validation

### 6.3 Integration Tests

```bash
pytest tests/integration/ -v
```

**Tests**:
- End-to-end user registration → query → recommendation flow
- Conversation history persistence
- Reading progress tracking

---

## Step 7: Deployment (Optional)

### 7.1 Docker Build

```bash
# Build backend image
docker build -t rag-chatbot-backend:latest -f backend/Dockerfile .

# Build frontend image
docker build -t rag-chatbot-frontend:latest -f frontend/Dockerfile .
```

### 7.2 Deploy to Hugging Face Spaces

```bash
# Install Hugging Face CLI
pip install huggingface_hub

# Login
huggingface-cli login

# Push Docker image
docker tag rag-chatbot-backend:latest <your-username>/rag-chatbot:latest
docker push <your-username>/rag-chatbot:latest

# Create Space
huggingface-cli repo create --type=space --space_sdk=docker rag-chatbot

# Configure secrets in Space settings
GEMINI_API_KEY
QDRANT_API_KEY
QDRANT_ENDPOINT
NEON_CONNECTION_STRING
BETTER_AUTH_SECRET
```

### 7.3 Alternative: Railway Deployment

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login and deploy
railway login
railway init
railway up
```

---

## Troubleshooting

### Issue: Gemini API 429 (Rate Limit)

**Solution**: Free tier has 1500 requests/day. Implement caching:
```python
# Add to backend/app/services/llm.py
from functools import lru_cache

@lru_cache(maxsize=100)
def cached_embedding(text: str) -> list[float]:
    return generate_embedding(text)
```

### Issue: Qdrant Connection Timeout

**Solution**: Check firewall rules and verify endpoint URL:
```bash
# Test connection
curl -X GET "$QDRANT_ENDPOINT/health" \
  -H "api-key: $QDRANT_API_KEY"
```

### Issue: Neon Database Connection Pooling

**Solution**: Add `pgbouncer=true` to connection string:
```
postgresql://neondb_owner:npg_ktgZ4va0zuPF@ep-crimson-king-a5z9l9l0.us-east-2.aws.neon.tech/neondb?sslmode=require&pgbouncer=true
```

### Issue: Better Auth Session Not Persisting

**Solution**: Ensure HTTP-only cookies are enabled:
```typescript
// frontend/src/lib/auth.ts
export const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_BACKEND_URL,
  credentials: "include", // Enable cookies
});
```

### Issue: Mobile UI Not Responsive

**Solution**: Verify TailwindCSS breakpoints:
```css
/* Ensure these breakpoints are configured */
sm: 640px  /* Mobile */
md: 768px  /* Tablet */
lg: 1024px /* Desktop */
```

### Issue: Recommendations Always Empty

**Solution**: Ensure user profile is complete and reading progress is tracked:
```bash
# Verify user profile
curl -X GET http://localhost:8000/auth/me \
  -H "Authorization: Bearer <jwt-token>"

# Track reading progress manually
curl -X POST http://localhost:8000/progress \
  -H "Authorization: Bearer <jwt-token>" \
  -H "Content-Type: application/json" \
  -d '{
    "chapter_id": "ch01",
    "completed": true,
    "time_spent_seconds": 300
  }'
```

---

## Next Steps

1. **Explore the API**: Open http://localhost:8000/docs for full API documentation
2. **Test Conversation History**: Create a conversation and ask follow-up questions
3. **Test Mobile UI**: Resize browser to mobile viewport (375px width)
4. **Review Logs**: Check `backend/logs/app.log` for debugging information
5. **Read Architecture**: See `specs/001-rag-chatbot-auth/plan.md` for technical details

---

## Useful Commands

```bash
# View backend logs
tail -f backend/logs/app.log

# Reset database (WARNING: deletes all data)
python backend/scripts/reset_db.py

# Re-ingest book (updates embeddings)
python backend/scripts/ingest_book.py --book-path ../book.epub --force

# Export user data (GDPR compliance)
python backend/scripts/export_user_data.py --user-id <uuid>

# Health check
curl http://localhost:8000/health
```

---

## Support

- **Documentation**: See `specs/001-rag-chatbot-auth/` for detailed specifications
- **Issues**: Report bugs in GitHub Issues
- **Logs**: Check `backend/logs/` for error details
- **Constitution**: See `.specify/memory/constitution.md` for project principles

---

**Estimated Setup Time**: 12-15 minutes
**Difficulty**: Intermediate
**Last Tested**: 2025-12-13
