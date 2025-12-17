---
title: Physical AI Humanoid Robotics Backend
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
license: mit
---

# Physical AI Humanoid Robotics - Backend API

Backend API for the Physical AI Humanoid Robotics chatbot system. Provides RAG (Retrieval-Augmented Generation) functionality with strict grounding, user authentication, and conversation history.

## Features

- **RAG Chatbot**: Retrieval-augmented generation with Gemini API and Qdrant vector database
- **User Authentication**: JWT-based authentication with session management
- **Conversation History**: Multi-turn conversation context preservation
- **Vector Search**: Semantic search using Qdrant Cloud for book content retrieval
- **Strict Grounding**: All answers sourced exclusively from book content (zero hallucinations)

## Tech Stack

- **Framework**: FastAPI (Python 3.11)
- **LLM**: Google Gemini API (gemini-2.0-flash-exp)
- **Embeddings**: text-embedding-004
- **Vector DB**: Qdrant Cloud
- **Database**: Neon Serverless Postgres
- **Authentication**: JWT with HTTP-only cookies

## API Endpoints

### Health Check
- `GET /health` - Check service health status

### Authentication
- `POST /auth/signup` - Create new user account with background questionnaire
- `POST /auth/signin` - Sign in with email/password
- `POST /auth/signout` - Sign out and clear session
- `GET /auth/me` - Get current user profile
- `POST /auth/refresh` - Refresh JWT token

### Chatbot
- `POST /chat/global` - Ask questions about book content (global search)
- `POST /chat/local` - Ask questions about selected text only
- `GET /chat/history` - Get user's conversation history
- `GET /chat/history/{conversation_id}` - Get specific conversation with all interactions
- `POST /chat/new` - Create new conversation

## Environment Variables

The following environment variables must be configured in Hugging Face Space secrets:

```bash
# Gemini API
GEMINI_API_KEY=your_gemini_api_key

# Qdrant Cloud
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_ENDPOINT=your_qdrant_endpoint
QDRANT_COLLECTION=physical-ai-humanoid-robotics

# Neon Postgres
NEON_CONNECTION_STRING=postgresql://user:pass@host/db

# Authentication
SESSION_SECRET=your_session_secret_key
SESSION_EXPIRATION_HOURS=24

# Frontend
FRONTEND_URL=https://physical-ai-humanoid-robotics-zeta.vercel.app
```

## Setup Instructions

### Local Development

1. **Clone the repository**:
```bash
git clone https://huggingface.co/spaces/Mn-2k24/physical-ai-humanoid-robotics-backend
cd physical-ai-humanoid-robotics-backend
```

2. **Install dependencies**:
```bash
pip install -r requirements.txt
```

3. **Set up environment variables**:
```bash
cp .env.example .env
# Edit .env with your API keys and configuration
```

4. **Run the server**:
```bash
uvicorn app:app --host 0.0.0.0 --port 7860 --reload
```

### Hugging Face Spaces Deployment

This backend is automatically deployed on Hugging Face Spaces using Docker. The Dockerfile handles all setup and configuration.

**Port**: 7860 (Hugging Face Spaces default)

## Database Schema

### Users
- `users` - User accounts (email, password hash, full name)
- `user_profiles` - User experience levels
- `user_software_background` - Programming languages and frameworks
- `user_hardware_background` - Available hardware and robotics equipment

### Conversations
- `conversations` - Conversation metadata
- `chat_interactions` - Individual chat messages with source references

### Sessions
- `auth_sessions` - JWT session tokens
- `audit_logs` - Authentication event logs

## Architecture

```
┌─────────────┐
│   Vercel    │
│  Frontend   │ (React/Docusaurus)
└──────┬──────┘
       │ HTTPS
       ▼
┌─────────────────┐
│ Hugging Face    │
│ Spaces Backend  │ (FastAPI)
└────┬────────┬───┘
     │        │
     │        ▼
     │   ┌──────────┐
     │   │  Qdrant  │ (Vector embeddings)
     │   │  Cloud   │
     │   └──────────┘
     │
     ▼
┌──────────────┐
│    Neon      │ (User data, conversations)
│  Postgres    │
└──────────────┘
     │
     ▼
┌──────────────┐
│   Gemini     │ (Embeddings + LLM)
│     API      │
└──────────────┘
```

## Security

- **Password Hashing**: bcrypt with salt
- **JWT Tokens**: HTTP-only cookies with secure flag
- **Rate Limiting**: 5 requests/minute for auth endpoints
- **CORS**: Configured for Vercel frontend only
- **Input Validation**: Pydantic schemas for all requests
- **SQL Injection**: Parameterized queries with psycopg3

## Rate Limits

- **Authentication**: 5 attempts/minute per IP
- **Gemini API**: 1200 requests/day (free tier)
- **Qdrant**: 1GB vector storage (free tier)
- **Neon**: 512MB database storage (free tier)

## Troubleshooting

### Common Issues

1. **Health check fails**: Verify all environment variables are set correctly
2. **Qdrant connection error**: Check QDRANT_API_KEY and QDRANT_ENDPOINT
3. **Neon Postgres timeout**: Verify NEON_CONNECTION_STRING format
4. **Gemini quota exceeded**: Wait for daily quota reset or upgrade plan

### Logs

Check Hugging Face Space logs for detailed error messages:
- Navigate to your Space → Settings → Logs
- Look for startup errors, database connection issues, or API failures

## Support

For issues or questions:
- **GitHub Issues**: [Report issues](https://github.com/your-repo/issues)
- **Documentation**: [Full documentation](https://physical-ai-humanoid-robotics-zeta.vercel.app)

## License

MIT License - see LICENSE file for details

---

**Status**: ✅ Production-ready | **Last Updated**: 2025-12-18
