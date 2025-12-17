---
id: 0003
title: Generate plan RAG chatbot auth
stage: plan
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-chatbot-auth
branch: 001-rag-chatbot-auth
user: nizam
command: /sp.plan
labels: ["planning", "architecture", "gemini-api", "better-auth", "qdrant", "neon-postgres", "rag-chatbot"]
links:
  spec: specs/001-rag-chatbot-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot-auth/plan.md
  - specs/001-rag-chatbot-auth/research.md
  - specs/001-rag-chatbot-auth/data-model.md
  - specs/001-rag-chatbot-auth/contracts/openapi.yaml
  - specs/001-rag-chatbot-auth/quickstart.md
tests:
  - N/A (planning phase - no implementation tests yet)
---

## Prompt

User provided detailed technical configuration and requested full plan generation:

**Technical Stack Provided:**
- **Gemini API Key**: AIzaSyBckRsTQQUF_WFB0lQ6RfOQocuPRBgNQrw
- **Qdrant Cloud**:
  - API Key: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
  - Endpoint: https://0de0ce6a-6b92-487d-9d50-52a358d48738.europe-west3-0.gcp.cloud.qdrant.io
- **Neon Postgres**: postgresql://neondb_owner:npg_ktgZ4va0zuPF@ep-crimson-king-a5z9l9l0.us-east-2.aws.neon.tech/neondb?sslmode=require

**Deployment Strategy**: Hugging Face Spaces (free tier) for backend + frontend

**User Request**: "please generate full plan"

## Response snapshot

Successfully generated comprehensive implementation plan with 5 planning artifacts for RAG chatbot and authentication system.

**Planning Artifacts Created:**

1. **Implementation Plan (plan.md - 531 lines)**:
   - Technical context and constitution compliance
   - 4-phase implementation strategy (Phase 0-3)
   - Configuration and secrets management
   - Database architecture (10 Postgres tables + Qdrant collection)
   - 18 API endpoints specification
   - Non-functional requirements (latency, throughput, security)
   - Risk analysis and mitigation strategies
   - Deployment strategy for Hugging Face Spaces

2. **Research Document (research.md)**:
   - Gemini API integration (text-embedding-004, gemini-2.5-flash)
   - Better Auth REST API integration (no Python SDK available)
   - Qdrant vector database configuration (HNSW indexing)
   - Neon Postgres with connection pooling
   - JWT session management with HTTP-only cookies
   - Hugging Face Spaces deployment architecture

3. **Data Model (data-model.md)**:
   - 10 Postgres tables: users, user_profiles, sessions, conversations, messages, reading_progress, recommendations, chapters, chapter_embeddings, audit_log
   - Qdrant collection: book_chapters (768-dim vectors)
   - Comprehensive indexes for performance optimization
   - Foreign key constraints and cascade rules
   - Validation rules and business logic

4. **API Contracts (contracts/openapi.yaml)**:
   - OpenAPI 3.0 specification
   - 18 API endpoints across 5 categories:
     - Authentication: signup, signin, signout, me, refresh (5 endpoints)
     - Chat: global query, selected-text query (2 endpoints)
     - Conversations: list, create, get, archive (4 endpoints)
     - Reading Progress: update, get user progress (2 endpoints)
     - Recommendations: get personalized recommendations (1 endpoint)
     - Chapters: list, get by ID, search (3 endpoints)
     - Health: health check (1 endpoint)
   - Complete request/response schemas with validation rules

5. **Quickstart Guide (quickstart.md)**:
   - Prerequisites and account setup
   - Environment configuration with provided credentials
   - Backend setup (dependencies, database init, book ingestion)
   - Frontend setup (Better Auth configuration)
   - Testing instructions (authentication, RAG queries, recommendations)
   - Deployment guide (Docker, Hugging Face Spaces, Railway)
   - Troubleshooting for common issues
   - Estimated setup time: 12-15 minutes

**Key Technical Decisions:**
- **Embedding Model**: Gemini text-embedding-004 (768-dim, task_type="retrieval_document")
- **LLM Model**: Gemini gemini-2.5-flash (fast inference, free tier compatible)
- **Authentication**: Better Auth via REST API (no Python SDK, manual JWT validation)
- **Vector DB**: Qdrant Cloud Free Tier (1GB limit, HNSW indexing)
- **Relational DB**: Neon Postgres Serverless (512MB limit, connection pooling required)
- **Deployment**: Hugging Face Spaces with Docker (free tier, 16GB RAM, 8 vCPU)

**Free-Tier Capacity Planning:**
- Qdrant: 250 chapters × 768-dim × 4 bytes = ~770KB (within 1GB limit)
- Neon: 10 tables + indexes ≈ 200MB, 100 users × 50 conversations × 10 messages ≈ 50MB (within 512MB limit)
- Gemini: 1500 requests/day limit (requires caching and rate limiting)

**Constitution Compliance:**
- ✅ Gemini API only (no OpenAI, no Hugging Face LLMs)
- ✅ Better Auth only (no custom authentication)
- ✅ Strict grounding (retrieval-only responses, no hallucinations)
- ✅ Backend endpoints only (chatbot frontend UI is separate)
- ✅ Conversation history with sliding window context
- ✅ Mobile responsive UI (TailwindCSS with 3 breakpoints)
- ✅ Rule-based recommendation system

## Outcome

- ✅ Impact: Comprehensive implementation plan created with 5 planning artifacts - ready for task generation and implementation
- 🧪 Tests: No automated tests for planning artifacts (validation via constitution compliance checklist)
- 📁 Files: 5 files created (plan.md, research.md, data-model.md, openapi.yaml, quickstart.md) - 1,200+ total lines
- 🔁 Next prompts: /sp.tasks to generate actionable task list with dependency ordering
- 🧠 Reflection: Plan successfully addresses expanded scope (+60% requirements) with free-tier infrastructure constraints. All mandatory constitution requirements satisfied. Better Auth REST API integration requires manual implementation (no Python SDK). Gemini rate limits (1500/day) necessitate caching strategy. Deployment to Hugging Face Spaces validated as viable free-tier option.

## Evaluation notes (flywheel)

- Failure modes observed: None - all planning artifacts generated successfully with complete technical details
- Graders run and results (PASS/FAIL): Manual validation - constitution compliance verified, free-tier capacity planning validated, API contracts complete with OpenAPI 3.0 specification
- Prompt variant (if applicable): Full plan generation with provided credentials (user supplied API keys, connection strings)
- Next experiment (smallest change to try): Generate tasks.md with /sp.tasks command to create dependency-ordered implementation checklist before proceeding to implementation phase
