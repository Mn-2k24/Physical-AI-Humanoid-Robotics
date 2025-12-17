# Implementation Plan: Integrated RAG Chatbot & Authentication System

**Branch**: `001-rag-chatbot-auth` | **Date**: 2025-12-13 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot-auth/spec.md`

## Summary

Build an integrated RAG (Retrieval-Augmented Generation) chatbot system embedded in a Docusaurus-based book with mandatory user authentication. The system provides grounded answers from book content using Gemini API, stores conversation history, offers personalized chapter recommendations, and features a fully responsive mobile UI.

**Primary Requirements**:
- **Authentication-gated access**: All book and chatbot features require signin/signup via Better Auth
- **RAG chatbot**: Global and selected-text query modes with strict grounding (zero hallucinations)
- **Conversation history**: Multi-turn context preservation across sessions
- **Recommendations**: Personalized chapter suggestions based on user background and reading progress
- **Mobile-responsive UI**: All components optimized for mobile, tablet, and desktop

**Technical Approach**:
- Backend: FastAPI (Python 3.11) with Gemini API for embeddings and generation
- Storage: Qdrant Cloud (vectors), Neon Serverless Postgres (users, conversations, progress)
- Frontend: React/Docusaurus with Better Auth integration
- Deployment: Docker containerization on free-tier hosting (Hugging Face Spaces/Railway/Render)

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, google-generativeai (Gemini SDK), qdrant-client, psycopg2-binary, better-auth, pydantic, uvicorn
**Storage**: Qdrant Cloud (Free Tier) for vectors, Neon Serverless Postgres for relational data
**Testing**: pytest, pytest-asyncio, httpx (API testing)
**Target Platform**: Linux server (Docker container), browser-based frontend
**Project Type**: Web application (backend + frontend auth UI)
**Performance Goals**: <3s query latency, <1s recommendation generation, <500ms auth validation
**Constraints**: Free-tier limits (Qdrant: 1GB vectors, Neon: 512MB storage), <200ms p95 API response time, HTTPS-only auth
**Scale/Scope**: 100 concurrent users, ~1000 text chunks embedded, 34 book chapters, 100 conversations per user

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Mandatory Requirements (From Constitution v4.0.0)

- ✅ **Gemini API Only**: Use Gemini API for embeddings (`gemini-embedding-001`) and LLM (`gemini-2.5-flash`) - NO OpenAI, NO Hugging Face
- ✅ **Better Auth Only**: Use Better Auth for authentication - NO custom auth logic
- ✅ **Strict Grounding**: All chatbot answers MUST be sourced from book content only (zero hallucinations, zero external knowledge)
- ✅ **Context Isolation**: Selected-text queries MUST only use selected content (no cross-chapter retrieval)
- ✅ **Backend Endpoints Only**: Frontend chatbot UI out of scope (only auth UI required)
- ✅ **Qdrant for Embeddings**: Embeddings stored in Qdrant only (NOT in Postgres)
- ✅ **Neon Postgres for User Data**: User accounts, conversations, progress, recommendations in Postgres (NO embeddings)
- ✅ **Security**: API keys server-side only, password hashing (bcrypt/Argon2), rate limiting, CSRF protection, HTTPS-only auth

### Non-Negotiable Constraints Verified

1. ✅ No OpenAI APIs - using Gemini API
2. ✅ No Hugging Face APIs - using Gemini API
3. ✅ No chatbot frontend implementation - auth UI only
4. ✅ No ungrounded answers - retrieval-only responses
5. ✅ No external knowledge usage - book content only
6. ✅ No custom authentication - Better Auth mandator

y
7. ✅ No skipped signup background - collecting software/hardware data
8. ✅ No frontend exclusion for auth - providing signup/signin UI
9. ✅ No embeddings in Postgres - Qdrant only
10. ✅ No secrets in frontend - server-side only

**GATE STATUS**: ✅ PASSED - All constitutional requirements satisfied

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot-auth/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - technology research and decisions
├── data-model.md        # Phase 1 output - database schemas and entities
├── quickstart.md        # Phase 1 output - setup and deployment guide
├── contracts/           # Phase 1 output - API specifications
│   ├── openapi.yaml     # OpenAPI 3.0 specification for all endpoints
│   └── schemas.json     # JSON schemas for request/response validation
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api/
│   │   ├── auth.py           # Better Auth integration endpoints
│   │   ├── chat.py           # RAG chatbot endpoints (global/local)
│   │   ├── conversations.py  # Conversation history management
│   │   ├── recommendations.py # Personalized chapter recommendations
│   │   └── progress.py       # Reading progress tracking
│   ├── models/
│   │   ├── user.py           # User, UserProfile, UserSoftwareBackground, UserHardwareBackground
│   │   ├── conversation.py   # Conversation, ChatInteraction
│   │   ├── progress.py       # ReadingProgress
│   │   └── recommendation.py # Recommendation
│   ├── services/
│   │   ├── embedding.py      # Gemini embedding generation
│   │   ├── qdrant.py         # Qdrant vector operations
│   │   ├── neon.py           # Neon Postgres operations
│   │   ├── rag.py            # RAG pipeline (retrieve + generate)
│   │   ├── recommendation_engine.py # Rule-based recommendation logic
│   │   └── auth_service.py   # Better Auth service wrapper
│   ├── core/
│   │   ├── config.py         # Environment variables and configuration
│   │   ├── security.py       # Password hashing, rate limiting, CSRF
│   │   └── middleware.py     # Authentication middleware
│   ├── utils/
│   │   ├── chunking.py       # Semantic text chunking
│   │   └── validation.py     # Request/response validation
│   └── main.py               # FastAPI app initialization
├── scripts/
│   ├── ingest.py             # Embed /docs chapters into Qdrant
│   ├── init_db.py            # Initialize Neon Postgres schema
│   └── migrate.py            # Database migration utility
├── tests/
│   ├── contract/
│   │   ├── test_auth_api.py
│   │   ├── test_chat_api.py
│   │   └── test_recommendations_api.py
│   ├── integration/
│   │   ├── test_rag_pipeline.py
│   │   └── test_auth_flow.py
│   └── unit/
│       ├── test_embedding.py
│       ├── test_chunking.py
│       └── test_recommendation_engine.py
├── requirements.txt
├── Dockerfile
└── README.md

src/
├── components/
│   ├── auth/
│   │   ├── SignupForm.tsx     # Multi-step signup with background questionnaire
│   │   ├── SigninForm.tsx     # Email/password signin with "remember me"
│   │   ├── PasswordReset.tsx  # Password reset flow
│   │   └── AuthProvider.tsx   # Better Auth React context
│   ├── layout/
│   │   ├── Header.tsx         # Updated to show auth state
│   │   └── Sidebar.tsx        # Updated to show auth state
│   └── chatbot/               # Pre-existing chatbot UI (no modifications)
├── pages/
│   ├── signup.tsx             # Signup page
│   ├── signin.tsx             # Signin page
│   └── reset-password.tsx     # Password reset page
└── hooks/
    └── useAuth.ts             # Authentication hooks

docs/
└── [34 existing book chapters] # Unchanged - source for embeddings
```

**Structure Decision**: Web application structure selected due to separation of FastAPI backend and React/Docusaurus frontend. Backend handles all data processing, authentication, and RAG logic. Frontend handles auth UI and integrates with existing chatbot component. This separation enables independent deployment and scaling of backend services.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations detected. All constitutional requirements satisfied:
- Using Gemini API exclusively (no OpenAI/Hugging Face)
- Using Better Auth exclusively (no custom auth)
- Strict grounding and context isolation enforced in RAG pipeline
- Proper data separation (Qdrant for vectors, Neon for users)
- Security requirements met (server-side secrets, password hashing, rate limiting)
- Auth UI provided (signup/signin components)
- Backend endpoints only (no chatbot UI implementation)

## Phase 0: Research & Technology Validation

**Objective**: Resolve all technology decisions, validate integrations, and document best practices for Gemini API, Better Auth, Qdrant, and Neon Postgres.

### Research Topics

1. **Gemini API Integration**
   - Research: Embedding model selection (`gemini-embedding-001` vs alternatives)
   - Research: LLM model for answer generation (`gemini-2.5-flash` performance and cost)
   - Research: Batch embedding strategies for 1000+ chunks
   - Research: Context window limits and prompt engineering for grounded responses
   - Research: Rate limits and quota management on free tier

2. **Better Auth + FastAPI Integration**
   - Research: Better Auth Python SDK availability and FastAPI compatibility
   - Research: Session management strategies (JWT vs session tokens)
   - Research: Password reset flow implementation
   - Research: CSRF protection patterns for FastAPI
   - Research: Rate limiting for authentication endpoints

3. **Qdrant Cloud Configuration**
   - Research: Collection schema design for book chunks
   - Research: Indexing strategies for similarity search performance
   - Research: Payload filtering for selected-text queries
   - Research: Free tier limits (1GB storage capacity planning)
   - Research: Connection pooling and retry strategies

4. **Neon Serverless Postgres**
   - Research: Schema design for users, conversations, progress, recommendations
   - Research: Connection pooling patterns for serverless functions
   - Research: Index strategies for conversation history queries
   - Research: Free tier limits (512MB storage capacity planning)
   - Research: Migration strategy and versioning

5. **Conversation History & Multi-Turn Context**
   - Research: Context window management for multi-turn conversations
   - Research: Conversation archival strategies (>30 days)
   - Research: Session state management across page refreshes
   - Research: Conversation retrieval optimization

6. **Recommendation System**
   - Research: Rule-based recommendation algorithms for chapter suggestions
   - Research: Reading progress tracking patterns (completion percentage, time spent)
   - Research: User background matching strategies (software/hardware)
   - Research: Experience level prioritization (beginner → advanced)

7. **Mobile-Responsive UI**
   - Research: Breakpoint strategies for mobile (<768px), tablet (768-1024px), desktop (>1024px)
   - Research: Touch interaction patterns for mobile chatbot
   - Research: Authentication form UX on mobile devices
   - Research: Performance optimization for mobile networks

8. **Deployment & Hosting**
   - Research: Docker containerization best practices for FastAPI
   - Research: Free-tier hosting comparison (Hugging Face Spaces, Railway, Render)
   - Research: Environment variable management
   - Research: Health check and monitoring strategies
   - Research: CI/CD pipeline setup

**Output**: `research.md` documenting all decisions, rationale, alternatives considered, and implementation patterns

## Phase 1: Design & Contracts

**Prerequisites**: `research.md` complete

### Data Model Design (`data-model.md`)

**Entities** (from spec.md Key Entities):

1. **User** (Neon Postgres)
   - Fields: id (UUID primary key), email (unique, indexed), hashed_password, full_name, created_at, updated_at
   - Relationships: 1-to-1 with UserProfile, 1-to-many with Conversation, 1-to-many with ReadingProgress
   - Validation: Email format, password min 8 characters
   - State transitions: Created → Active → (optional) Suspended

2. **UserProfile** (Neon Postgres)
   - Fields: user_id (foreign key), experience_level (enum: Beginner/Intermediate/Advanced/Expert), created_at, updated_at
   - Relationships: 1-to-1 with User, 1-to-1 with UserSoftwareBackground, 1-to-1 with UserHardwareBackground

3. **UserSoftwareBackground** (Neon Postgres)
   - Fields: user_id (foreign key), programming_languages (JSONB array), frameworks (JSONB array)
   - Validation: Arrays must contain valid predefined options

4. **UserHardwareBackground** (Neon Postgres)
   - Fields: user_id (foreign key), available_hardware (JSONB array), robotics_hardware (JSONB array)
   - Validation: Arrays must contain valid predefined options

5. **AuthSession** (Neon Postgres)
   - Fields: session_id (UUID primary key), user_id (foreign key), token (hashed), expires_at, created_at
   - State transitions: Active → Expired → Deleted
   - Index: user_id, expires_at for cleanup queries

6. **Conversation** (Neon Postgres)
   - Fields: id (UUID primary key), user_id (foreign key), title (auto-generated from first query), created_at, updated_at, archived (boolean, default false)
   - Relationships: 1-to-many with ChatInteraction
   - State transitions: Active → Archived (after 30 days)
   - Index: user_id, archived, created_at

7. **ChatInteraction** (Neon Postgres)
   - Fields: id (UUID primary key), conversation_id (foreign key), user_id (foreign key), query_text, answer_text, query_mode (enum: global/local), source_chunks (JSONB array), created_at
   - Relationships: Many-to-1 with Conversation
   - Index: conversation_id, created_at

8. **BookChunk** (Qdrant only - not in Postgres)
   - Payload: chunk_id, file_path, section_heading, chunk_index, raw_text
   - Vector: embedding_vector (Gemini embedding, dimension based on model)
   - Index: Vector similarity index

9. **ReadingProgress** (Neon Postgres)
   - Fields: id (UUID primary key), user_id (foreign key), chapter_id (string, e.g., "module-1-ros2/intro"), completion_percentage (0-100), time_spent_seconds, last_accessed, completed (boolean)
   - Relationships: Many-to-1 with User
   - Index: user_id, chapter_id (unique constraint), completed

10. **Recommendation** (Neon Postgres)
    - Fields: id (UUID primary key), user_id (foreign key), recommended_chapter_id (string), score (float, 0-1), reason (text), dismissed (boolean, default false), created_at
    - Relationships: Many-to-1 with User
    - Index: user_id, dismissed, score

### API Contracts (`contracts/openapi.yaml`)

**Authentication Endpoints**:
- `POST /auth/signup` - Create user account with background questionnaire
- `POST /auth/signin` - Authenticate user (email/password)
- `POST /auth/signout` - Terminate session
- `GET /auth/me` - Get current user profile
- `POST /auth/reset-password` - Initiate password reset
- `PUT /auth/reset-password` - Complete password reset

**RAG Chatbot Endpoints**:
- `POST /chat/global` - Global book query (requires auth)
- `POST /chat/local` - Selected-text query (requires auth)

**Conversation History Endpoints**:
- `GET /chat/history` - Retrieve user's conversations (requires auth)
- `GET /chat/history/{conversation_id}` - Retrieve specific conversation (requires auth)
- `POST /chat/new` - Start new conversation (requires auth)
- `DELETE /chat/history/{conversation_id}` - Delete conversation (requires auth)

**Recommendation Endpoints**:
- `GET /recommendations` - Get personalized chapter recommendations (requires auth)
- `PUT /recommendations/{recommendation_id}/dismiss` - Dismiss recommendation (requires auth)

**Reading Progress Endpoints**:
- `GET /progress` - Get user's reading progress (requires auth)
- `POST /progress` - Update reading progress (requires auth)

**Internal Endpoints**:
- `POST /ingest` - Embed /docs chapters into Qdrant (admin only)

### Quickstart Guide (`quickstart.md`)

**Contents**:
1. Prerequisites (Python 3.11, Docker, Node.js)
2. Environment variable setup (Gemini API key, Qdrant credentials, Neon connection string)
3. Backend setup (install dependencies, initialize database, run ingestion)
4. Frontend setup (install dependencies, configure Better Auth)
5. Local development (run backend and frontend servers)
6. Testing (run contract, integration, and unit tests)
7. Deployment (Docker build, deploy to Hugging Face Spaces/Railway/Render)
8. Troubleshooting common issues

## Phase 2: Tasks Generation (Not Executed in /sp.plan)

**Note**: Tasks are generated by the `/sp.tasks` command, not during planning. This phase is documented here for completeness but will be executed in a separate workflow.

**Expected Task Categories**:
1. Setup tasks (project initialization, dependency installation)
2. Foundational tasks (database schema, authentication framework, embedding pipeline)
3. User Story 1: Authentication (P1 - blocking all other features)
4. User Story 2: RAG with conversation history (P2)
5. User Story 3: Selected-text queries (P3)
6. User Story 4: Auth UI in header/sidebar (P4)
7. User Story 5: Personalized recommendations (P5)
8. Polish & testing tasks

## Configuration & Secrets Management

### Environment Variables (`.env`)

```bash
# Gemini API
GEMINI_API_KEY=AIzaSyBckRsTQQUF_WFB0lQ6RfOQocuPRBgNQrw

# Qdrant Cloud
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.yzJCwNt_iaudrD1FYIFfx0W_rwyLB9DmgB_aGZSjv7M
QDRANT_ENDPOINT=https://0de0ce6a-6b92-487d-9d50-52a358d48738.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_COLLECTION=physical_ai_book

# Neon Postgres
NEON_CONNECTION_STRING=postgresql://neondb_owner:npg_ktgZ4va0zuPF@ep-still-wildflower-abp6qoo7-pooler.eu-west-2.aws.neon.tech/neondb?sslmode=require&channel_binding=require

# Better Auth
BETTER_AUTH_SECRET=<generate-secure-random-string>
BETTER_AUTH_URL=<backend-url>

# Backend
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000
CORS_ORIGINS=http://localhost:3000,https://physical-ai-humanoid-robotics-zeta.vercel.app

# Security
RATE_LIMIT_PER_MINUTE=5
SESSION_EXPIRATION_HOURS=24
CSRF_SECRET=<generate-secure-random-string>
```

### MCP Server for Better Auth (Claude CLI Integration)

```bash
# Setup Better Auth MCP server for Claude Code
npx @better-auth/cli mcp --claude-code
```

This enables Better Auth autocomplete and integration directly in Claude Code during development.

## Deployment Architecture

### Docker Container

```yaml
services:
  backend:
    build: ./backend
    ports:
      - "8000:8000"
    environment:
      - GEMINI_API_KEY=${GEMINI_API_KEY}
      - QDRANT_API_KEY=${QDRANT_API_KEY}
      - QDRANT_ENDPOINT=${QDRANT_ENDPOINT}
      - NEON_CONNECTION_STRING=${NEON_CONNECTION_STRING}
      - BETTER_AUTH_SECRET=${BETTER_AUTH_SECRET}
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8000/health"]
      interval: 30s
      timeout: 10s
      retries: 3
```

### Free-Tier Hosting Options

1. **Hugging Face Spaces** (Recommended)
   - Pros: Free GPU access, Docker support, persistent storage
   - Cons: 16GB RAM limit, may have cold starts
   - Best for: FastAPI backend with Gemini API integration

2. **Railway**
   - Pros: Easy deployment, free tier with $5 credit/month
   - Cons: Limited free tier, may require credit card
   - Best for: Quick deployment and testing

3. **Render**
   - Pros: Free web services with 512MB RAM
   - Cons: Spins down after inactivity, slower cold starts
   - Best for: Low-traffic MVP deployment

**Recommendation**: Hugging Face Spaces for primary deployment, Railway as backup.

## Acceptance Criteria Checklist

From specification success criteria:

**Authentication & Access Control**:
- [ ] SC-001: 100% of unauthenticated access blocked and redirected
- [ ] SC-002: Signup completes in <3 minutes
- [ ] SC-003: Signin completes in <10 seconds
- [ ] SC-004: Sessions persist across page refreshes
- [ ] SC-005: Session expiration triggers signout in <1 second

**Chatbot Functionality**:
- [ ] SC-006: Answers delivered in <3 seconds
- [ ] SC-007: 95% of queries return relevant answers
- [ ] SC-008: Selected-text queries 100% context-isolated
- [ ] SC-009: All answers include source references
- [ ] SC-010: Zero hallucinations (100% verifiable)

**Conversation History**:
- [ ] SC-011: History preserved with 100% accuracy
- [ ] SC-012: Multi-turn context maintained for 10+ turns
- [ ] SC-013: Conversation retrieval in <2 seconds
- [ ] SC-014: Archival reduces storage by 70%

**Mobile Responsive UI**:
- [ ] SC-015: Renders correctly on mobile (320px-480px)
- [ ] SC-016: Renders correctly on tablet (481px-1024px)
- [ ] SC-017: Renders correctly on desktop (>1024px)
- [ ] SC-018: Touch interactions work with 95% success

**Recommendation System**:
- [ ] SC-019: Recommendations generated in <1 second
- [ ] SC-020: 80% match software background
- [ ] SC-021: 80% match hardware availability
- [ ] SC-022: 100% beginner prioritization correct
- [ ] SC-023: 100% dismiss success rate

**User Experience**:
- [ ] SC-024: 90% first query success rate
- [ ] SC-025: Validation errors in <500ms
- [ ] SC-026: Auth state updates in <1 second
- [ ] SC-027: 100% background data capture

**System Performance**:
- [ ] SC-028: 500 requests/day without degradation
- [ ] SC-029: All 34 chapters ingested successfully
- [ ] SC-030: Vector search in <1 second
- [ ] SC-031: 100 conversations per user supported

**Security and Privacy**:
- [ ] SC-032: Zero API keys exposed
- [ ] SC-033: Rate limiting blocks >5 attempts/minute
- [ ] SC-034: Passwords hashed with bcrypt/Argon2
- [ ] SC-035: User background only in recommendations

## Risk Assessment & Mitigation

### High-Risk Items

1. **Free-Tier Storage Limits**
   - Risk: Conversation history + recommendations may exceed Neon 512MB limit
   - Mitigation: Implement aggressive archival (>30 days), conversation pruning, monitoring alerts

2. **Gemini API Rate Limits**
   - Risk: Free-tier quota exhaustion with 500+ requests/day
   - Mitigation: Implement request caching, rate limiting, quota monitoring

3. **Better Auth + FastAPI Integration**
   - Risk: Better Auth primarily designed for Node.js, Python SDK may be limited
   - Mitigation: Research phase validation, fallback to manual JWT implementation if needed

4. **Mobile UI Performance**
   - Risk: Large conversation histories may slow mobile rendering
   - Mitigation: Pagination, lazy loading, conversation summary views

### Medium-Risk Items

1. **Qdrant 1GB Vector Limit**
   - Risk: 1000 chunks may approach limit with metadata
   - Mitigation: Optimize chunk size, compress metadata, monitor usage

2. **Recommendation Algorithm Complexity**
   - Risk: Rule-based recommendations may not scale or provide value
   - Mitigation: Start simple (experience level matching), iterate based on user feedback

3. **Session Management Complexity**
   - Risk: Session expiration edge cases (mid-query, mid-form submission)
   - Mitigation: Graceful error handling, session refresh logic, destination preservation

## Next Steps

**After Plan Approval**:
1. Execute Phase 0: Run research tasks and generate `research.md`
2. Execute Phase 1: Generate `data-model.md`, `contracts/openapi.yaml`, `quickstart.md`
3. Update agent context: Run `.specify/scripts/bash/update-agent-context.sh claude`
4. Generate tasks: Run `/sp.tasks` to create implementation task list
5. Begin implementation: Start with foundational tasks (database, auth framework)

**Immediate Actions**:
- Validate Gemini API access and quota limits
- Validate Better Auth Python SDK availability
- Verify Qdrant and Neon connection credentials
- Set up local development environment per `quickstart.md` (when generated)
