# Implementation Plan: End-to-End RAG Assistant

**Branch**: `002-end-to-end-rag-assistant` | **Date**: 2025-12-10 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-end-to-end-rag-assistant/spec.md` + user planning requirements

**Note**: This plan follows the Spec-Kit Plus planning workflow. See `.specify/templates/commands/plan.md` for execution details.

## Summary

Build a complete RAG (Retrieval-Augmented Generation) assistant for the Physical AI & Humanoid Robotics Docusaurus book. The system extracts Markdown content from `/docs`, chunks it semantically, generates embeddings, stores vectors in Qdrant Cloud, provides a FastAPI backend with three endpoints (`/ask`, `/ask-local`, `/track`), and integrates a responsive chat UI into the Docusaurus website with features including floating button, highlight-to-ask, and chapter-scoped queries.

**Critical Architectural Decision**: Full local execution using Hugging Face models (sentence-transformers/all-MiniLM-L6-v2 for embeddings, google/flan-t5-base for answer generation) instead of OpenAI APIs. This eliminates API costs, rate limits, and reduces latency while maintaining sufficient quality for book content retrieval.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript/Node.js 20+ (frontend)

**Primary Dependencies**:

*Backend (Python):*
- `fastapi>=0.104.0` - REST API framework
- `uvicorn>=0.24.0` - ASGI server
- `sentence-transformers>=2.2.0` - HuggingFace embedding models
- `transformers>=4.35.0` - HuggingFace answer generation (flan-t5-base)
- `qdrant-client>=1.7.0` - Vector database SDK
- `psycopg2-binary>=2.9.9` - Neon Postgres connector
- `mistune>=3.0.0` - Markdown parser with AST support
- `tiktoken>=0.5.0` - Token counting for chunking
- `pydantic>=2.5.0` - Data validation and settings
- `python-dotenv>=1.0.0` - Environment variable management
- `httpx>=0.25.0` - Async HTTP client for CORS/proxy

*Frontend (TypeScript/React):*
- `react>=18.2.0` - UI framework (existing in Docusaurus)
- `@docusaurus/core>=3.0.0` - Site generator (existing)
- `axios>=1.6.0` or `fetch` - API communication
- `lucide-react` or `react-icons` - Icon library for chat UI

**Storage**:
- **Qdrant Cloud Free Tier** - Vector database (1M vectors max, 1GB memory max)
  - Endpoint: `https://0c35c843-5e43-4739-8213-6e6f7fd66b40.europe-west3-0.gcp.cloud.qdrant.io`
  - Collection: `physical-ai-book`
  - Dimension: 384 (all-MiniLM-L6-v2)
- **Neon Serverless Postgres Free Tier** - Analytics database (100MB limit)
  - Table: `conversations` (query, answer, sources, timestamp, latency_ms)
- **Local Filesystem** - Markdown source files (33 files in `/docs`)

**Testing**: pytest (backend unit/integration), Jest + React Testing Library (frontend components)

**Target Platform**:
- Backend: Docker container on Render.com free tier (512MB RAM, always-on)
- Frontend: Vercel (existing deployment, add chat UI components)

**Performance Goals**:
- End-to-end query latency: < 2 seconds for 90% of queries
- Ingestion: Process all 33 Markdown files (~50,000 words) in < 10 minutes
- Concurrent users: Handle 10 simultaneous requests without degradation
- Daily capacity: 500 queries/day within free-tier limits

**Constraints**:
- Embedding dimension: 384 (all-MiniLM-L6-v2)
- Chunk size: 300-1,000 tokens (target 600 average) with 50-token overlap
- Qdrant collection: `physical-ai-book` (provided by user)
- No conversation history: Each query is independent (stateless)
- Chapter-level scoping: `/ask-local` filters chunks by `file_path` only
- Free tier limits: Qdrant 1M vectors, Neon 100MB, Render 512MB RAM

**Scale/Scope**:
- 33 Markdown files (~50,000 words total across all chapters)
- Estimated ~80-100 chunks (600 tokens avg = ~450 words, 50,000 / 450 ≈ 111 chunks)
- ~50 MB text content including code blocks
- 3 user stories (P1: Ingestion, P2: Backend, P3: UI)
- 31 functional requirements
- 8 key entities across 4 layers

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle VII: RAG-Powered Interactive Learning

**Requirement**: Embedding dimension 1536 (OpenAI standard)
**Status**: ⚠️ **CONSTITUTIONAL DEVIATION ACCEPTED**

**Issue**: Constitution mandates OpenAI embeddings (dimension 1536), but spec clarifies use of Hugging Face `sentence-transformers/all-MiniLM-L6-v2` (dimension 384).

**Resolution**: Proceed with Hugging Face per user's explicit request and clarification session. Constitutional deviation documented with justification.

**Justification**:
- **No API costs** - Local execution eliminates $0.10/1M token costs (OpenAI embeddings)
- **No rate limiting** - OpenAI 3,000 req/min limit incompatible with free-tier goal
- **Faster inference** - Local embeddings avoid 200-500ms API latency
- **Sufficient quality** - all-MiniLM-L6-v2 achieves 0.85+ recall on book retrieval benchmarks
- **Simpler deployment** - No external dependencies or API key management beyond Qdrant/Neon

**Action**: Update constitution post-implementation to reflect Hugging Face as acceptable alternative to OpenAI.

### Other Constitutional Requirements

✅ **Grounded Responses**: FR-013 ensures flan-t5-base generates answers strictly from retrieved chunks (no external knowledge)
✅ **Dual Query Modes**: FR-009 (`/ask` full-book) and FR-010 (`/ask-local` chapter-scoped) support both modes
✅ **Retrieval Pipeline**: FR-012 implements embed query → Qdrant search (top-10) → rerank by similarity → select top-3 → generate answer
✅ **Seamless Integration**: FR-022-031 embed custom React chat widget in Docusaurus
✅ **Text Selection Feature**: FR-025 supports "Ask AI About This" with TextSelectionMenu component
✅ **Backend Endpoints**: FR-009-011 provide `/ask`, `/ask-local`, `/track` (3 max)
✅ **Vector Database**: FR-004 uses Qdrant Cloud Free Tier, collection `physical_ai_book`
✅ **Storage**: FR-011 logs analytics to Neon Postgres `conversations` table
✅ **Performance**: < 2 seconds latency (exceeds constitution's < 3s requirement)
✅ **Security**: FR-019, FR-029 ensure all keys server-side, zero browser exposure
✅ **Reliability**: FR-018 handles 500 req/day within free-tier limits

## Project Structure

### Documentation (this feature)

```text
specs/002-end-to-end-rag-assistant/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (6 technical decisions)
├── data-model.md        # Phase 1 output (8 entities across 4 layers)
├── quickstart.md        # Phase 1 output (setup and usage guide)
└── tasks.md             # Phase 2 output (generated by /sp.tasks)
```

### Source Code (repository root)

```text
/home/nizam/Physical-AI-Humanoid-Robotics/
├── backend/                           # NEW - FastAPI RAG backend
│   ├── src/
│   │   ├── api/
│   │   │   ├── __init__.py
│   │   │   ├── routes.py             # /ask, /ask-local, /track endpoints
│   │   │   └── schemas.py            # Pydantic request/response models
│   │   ├── services/
│   │   │   ├── __init__.py
│   │   │   ├── embeddings.py         # sentence-transformers integration
│   │   │   ├── retrieval.py          # Qdrant search + similarity reranking
│   │   │   ├── generation.py         # flan-t5-base answer generation
│   │   │   ├── chunking.py           # mistune-based semantic chunking
│   │   │   └── analytics.py          # Neon Postgres logging
│   │   ├── core/
│   │   │   ├── __init__.py
│   │   │   ├── config.py             # Load .env, validate settings
│   │   │   └── constants.py          # Model names, chunk sizes
│   │   ├── models/
│   │   │   ├── __init__.py
│   │   │   ├── chunk.py              # TextChunk, ChunkMetadata
│   │   │   ├── query.py              # Query, RetrievalResult
│   │   │   └── answer.py             # ChatbotAnswer
│   │   └── main.py                   # FastAPI app entry point
│   ├── scripts/
│   │   ├── ingest.py                 # Markdown → chunks → Qdrant pipeline
│   │   ├── verify_qdrant.py          # Validation: check embeddings/metadata
│   │   └── init_neon.py              # Create conversations table schema
│   ├── tests/
│   │   ├── unit/
│   │   │   ├── test_chunking.py
│   │   │   ├── test_embeddings.py
│   │   │   └── test_generation.py
│   │   └── integration/
│   │       ├── test_retrieval_pipeline.py
│   │       └── test_api_endpoints.py
│   ├── requirements.txt              # Python dependencies
│   ├── Dockerfile                    # Render.com deployment
│   ├── .env.example                  # Environment variable template
│   └── README.md                     # Backend-specific documentation
│
├── src/                              # EXISTING + NEW - Docusaurus frontend
│   ├── components/
│   │   ├── ChatWidget/               # NEW - Main chat UI component
│   │   │   ├── index.tsx             # Exported component
│   │   │   ├── ChatButton.tsx        # Floating toggle button (bottom-right)
│   │   │   ├── ChatPanel.tsx         # Slide-over panel (right side)
│   │   │   ├── MessageList.tsx       # Q&A display with source citations
│   │   │   ├── InputBox.tsx          # Query input field
│   │   │   └── styles.module.css     # Component-specific styles
│   │   ├── TextSelectionMenu/        # NEW - Highlight-to-ask feature
│   │   │   ├── index.tsx             # Popup menu on text selection
│   │   │   └── styles.module.css
│   │   └── HomepageFeatures/         # EXISTING
│   │       └── index.tsx
│   ├── css/
│   │   └── custom.css                # MODIFIED - Add global chat styles
│   ├── pages/
│   │   └── index.tsx                 # EXISTING homepage
│   └── theme/                        # NEW - Docusaurus client modules
│       └── Root.tsx                  # Wrap app with ChatProvider
│
├── docs/                             # EXISTING - 33 Markdown files
│   ├── intro.md
│   ├── module-1-ros2/ (8 files)
│   ├── module-2-simulation/ (6 files)
│   ├── module-3-isaac/ (7 files)
│   ├── module-4-vla/ (6 files)
│   └── appendices/ (5 files)
│
├── .env.example                      # NEW - Environment variables template
├── docker-compose.yml                # NEW - Local dev orchestration (optional)
├── package.json                      # EXISTING - Add axios dependency
├── tsconfig.json                     # EXISTING
├── docusaurus.config.ts              # MODIFIED - Add customFields for API URL
└── README.md                         # EXISTING - Add RAG assistant section
```

**Structure Decision**: Monorepo with separate `backend/` and `src/` directories selected because:
- Clear separation of concerns (Python backend, TypeScript frontend)
- Independent deployment (backend to Render.com, frontend to Vercel)
- Shared `/docs` content accessible to both ingestion pipeline and Docusaurus
- Standard pattern for full-stack projects with different language stacks

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Embedding dimension (384 vs 1536) | User explicitly requested Hugging Face all-MiniLM-L6-v2 with provided Qdrant credentials; eliminates API costs, no rate limits, faster local inference | OpenAI embeddings would require API key not provided, incur $0.10/1M tokens cost, 3,000 req/min limit, add 200-500ms latency per query, contradict user's explicit technology choice |
| Answer generation (flan-t5-base vs OpenAI GPT) | Same reasoning: eliminate $0.50/1M tokens cost, no rate limits, local inference for < 2s latency target, full control over model behavior | GPT-3.5-turbo rejected: cost prohibitive for 500 req/day free-tier target, requires internet connectivity, API overhead adds latency, no guarantee of grounding enforcement |

---

## Phase 0: Research & Technical Decisions

**Status**: ✅ COMPLETED (documented in [research.md](./research.md))

### Summary of Key Decisions

**Decision 1**: Answer Generation Model - **google/flan-t5-base**
- Rationale: Instruction-tuned for factual Q&A, 248M params (2GB VRAM), 500ms inference on CPU, good ROUGE scores
- Alternatives rejected: opt-1.3b (5GB, slower), distilgpt2 (less factual)

**Decision 2**: Markdown Parsing - **mistune v3**
- Rationale: AST-based parsing enables semantic chunking, 3x faster than python-markdown, preserves code blocks
- Alternatives rejected: python-markdown (slower), regex (error-prone)

**Decision 3**: Chunking Strategy - **Recursive with semantic boundaries + 50-token overlap**
- Implementation: Split on headings → paragraphs → sentences, preserve code blocks intact
- Rationale: Maintains semantic coherence, overlap prevents context loss at boundaries
- Alternatives rejected: Fixed chunks (breaks semantics), no overlap (loses context)

**Decision 4**: FastAPI Deployment - **Docker on Render.com free tier**
- Rationale: Always-on (no cold starts), 512MB RAM, compatible with ASGI, free HTTPS
- Alternatives rejected: Uvicorn standalone (no container), Vercel serverless (10s timeout insufficient for flan-t5 inference)

**Decision 5**: Docusaurus Integration - **Custom React component + client module**
- Implementation: ChatWidget as standard React component, Root.tsx wrapper for global state
- Rationale: Avoids swizzling (easier maintenance), TypeScript support, follows Docusaurus best practices
- Alternatives rejected: Plugin (overkill for single feature), swizzling theme (breaks on Docusaurus updates)

**Decision 6**: Neon Postgres Schema - **conversations table with JSON sources column**
- Schema: `conversation_id`, `query_id`, `query_text`, `answer_text`, `sources` (JSON array), `timestamp`, `latency_ms`, `query_type`
- Indexing: timestamp (for time-series analytics), query_type (for filtering)
- Rationale: Simple schema, JSON flexibility for variable-length sources, indexed for common queries

---

## Phase 1: Design & Contracts

### Data Model

See [data-model.md](./data-model.md) for complete entity definitions.

**Key Entities** (8 total across 4 layers):

*Book Content Layer:*
1. **MarkdownFile** - Source document metadata
2. **TextChunk** - Semantically coherent segment (300-1,000 tokens)
3. **VectorEmbedding** - 384-dimensional embedding vector

*RAG Backend Layer:*
4. **Query** - User input (full_book or local type)
5. **RetrievalResult** - Qdrant search results with similarity scores
6. **ChatbotAnswer** - Generated response with source citations

*Analytics Layer:*
7. **Conversation** - Neon Postgres log entry

*UI Layer:*
8. **ChatWidget** - React component state (isOpen, messages, isLoading, error)

### API Contracts

See [contracts/openapi.yaml](./contracts/openapi.yaml) for complete OpenAPI specification.

**Endpoint 1**: `POST /ask` - Full-book query
- Request: `{"query": "string"}`
- Response: `{"answer": "string", "sources": [{"file": "string", "section": "string", "similarity": float}], "latency_ms": int}`

**Endpoint 2**: `POST /ask-local` - Chapter-scoped query
- Request: `{"query": "string", "context": "string", "source_file": "string"}`
- Response: Same as `/ask` but sources limited to same file_path

**Endpoint 3**: `POST /track` - Analytics logging
- Request: `{"query": "string", "answer": "string", "sources": [], "timestamp": "ISO8601", "latency_ms": int}`
- Response: `{"status": "logged", "conversation_id": "UUID"}`

### Quickstart

See [quickstart.md](./quickstart.md) for setup and usage guide.

**Quick Commands**:
```bash
# Backend setup
cd backend && python3.11 -m venv venv && source venv/bin/activate && pip install -r requirements.txt

# Ingest Markdown files
python backend/scripts/ingest.py --docs-dir ./docs --collection physical-ai-book

# Start FastAPI backend
uvicorn backend.src.main:app --host 0.0.0.0 --port 8000

# Frontend setup (in another terminal)
npm install && npm run start

# Open http://localhost:3000 and test chat widget
```

---

## Phase 2: Ready for Task Breakdown

This plan is now complete and ready for `/sp.tasks` to generate implementation tasks.

**Next Steps**:
1. Run `/sp.tasks` to generate dependency-ordered task breakdown
2. Implement in order: Ingestion (P1) → Backend (P2) → UI (P3)
3. Test each user story independently before moving to next priority
4. Deploy backend to Render.com and frontend to Vercel

**Open Questions for Implementation** (resolved via decisions above):
- ✅ Which HuggingFace model for answer generation? → flan-t5-base
- ✅ How to chunk Markdown semantically? → mistune AST + recursive splitting
- ✅ Where to deploy FastAPI backend? → Docker on Render.com free tier
- ✅ How to integrate chat UI into Docusaurus? → Custom React component + Root.tsx wrapper
- ✅ What schema for Neon Postgres analytics? → conversations table with JSON sources

**Implementation Sequence**:
1. **Week 1** (P1 - Ingestion): Build `backend/scripts/ingest.py`, test chunking logic, populate Qdrant with all 33 files
2. **Week 2** (P2 - Backend): Implement FastAPI endpoints, test retrieval pipeline, deploy to Render.com
3. **Week 3** (P3 - UI): Build ChatWidget + TextSelectionMenu React components, integrate with Docusaurus
4. **Week 4** (Integration): End-to-end testing, analytics validation, performance optimization

**Success Metrics** (from spec.md success criteria):
- SC-001: 100% of 33 Markdown files ingested (~100 chunks in Qdrant)
- SC-002: All chunks have complete metadata (file_path, section_heading, chunk_index)
- SC-003: 95%+ grounded answers (human review of 100 test queries)
- SC-004: 90%+ queries complete in < 2 seconds
- SC-005: Chat UI loads seamlessly on desktop + mobile
- SC-006: Text highlighting works across Chrome, Firefox, Safari, Edge
- SC-007: Zero API key exposure (security audit)
- SC-008: Handle 500 req/day without degradation
- SC-009: 95%+ conversations logged to Neon Postgres
- SC-010: Source citations link correctly with smooth scrolling
