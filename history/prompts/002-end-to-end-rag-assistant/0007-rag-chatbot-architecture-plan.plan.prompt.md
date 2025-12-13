---
id: 0007
title: RAG Chatbot Architecture Plan
stage: plan
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-end-to-end-rag-assistant
branch: 002-end-to-end-rag-assistant
user: nizam
command: /sp.plan
labels: ["plan", "rag-chatbot", "architecture", "technical-design"]
links:
  spec: specs/002-end-to-end-rag-assistant/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-end-to-end-rag-assistant/plan.md
tests:
  - N/A (planning phase)
---

## Prompt

User provided comprehensive system plan for Gemini-based RAG chatbot with detailed technology stack, deployment strategy, and architecture specifications.

**Key User Requirements**:
- Gemini models via Hugging Face for embeddings and generation
- FastAPI backend with existing infrastructure
- Three query modes: book-related, selected-text, general conversation
- Qdrant Cloud for vector storage
- Neon Serverless Postgres for analytics
- Deployment on Hugging Face Spaces (Container)
- Integration with existing `/src/components` frontend

## Response snapshot

Successfully created comprehensive implementation plan for RAG Chatbot Behavioral System (feature 002-end-to-end-rag-assistant).

**Plan Highlights**:

**Technical Context**:
- Python 3.10+ with FastAPI framework
- Hugging Face Transformers + sentence-transformers/all-MiniLM-L6-v2 (384-dim embeddings)
- Gemini generation models via Hugging Face Inference API
- Qdrant Cloud (vectors) + Neon Postgres (analytics)
- Target: <3s p95 latency, 500 queries/day, 95% classification accuracy

**Architecture Design**:
1. **Query Classification**: Heuristic-based classifier for book|selection|general modes
2. **Retrieval Service**: Qdrant similarity search (0.65 threshold, top-5 chunks)
3. **Generation Service**: Gemini via HF with mode-specific system prompts
4. **Session Management**: 10-turn history, 30-min timeout, 10 queries/min rate limit
5. **Analytics**: Neon Postgres logging with 100% capture rate

**API Endpoints**:
- `POST /api/v1/chat` - Main chatbot endpoint (all 3 modes)
- `POST /api/v1/embed` - Embedding generation (internal)
- `GET /api/v1/health` - Dependency health check
- `POST /api/v1/sessions/{id}/clear` - Clear conversation history

**Data Models**:
- UserQuery (query_id, session_id, text, query_type, selected_text)
- RetrievedChunk (chunk_id, text, metadata, similarity_score >=0.65)
- ChatbotResponse (response_id, text, source_chunks, latency_ms)
- ConversationSession (session_id, history[10 turns], timeouts)
- BookContent (Qdrant: 384-dim vectors, metadata)

**Constitution Check**: ✅ PASS
- All Principle VII requirements addressed
- Zero hallucination enforcement via retrieval-only prompts
- Security: API keys server-side only
- Performance: <3s latency architecture
- Analytics: 100% query logging to Neon

**Project Structure**:
- Uses existing `backend/` directory
- Enhanced services: retrieval.py, generation.py, routes.py
- New services: embedding.py, classification.py, session.py, rate_limit.py
- New models: query.py, session.py, response.py
- New core modules: prompts.py (system prompts for each mode)

**Implementation Priorities** (for /sp.tasks):
1. Phase 1 (MVP): Book content Q&A (P1 user story)
2. Phase 2: Selected text queries (P2 user story)
3. Phase 3: General conversation (P3 user story)
4. Phase 4: Session mgmt, rate limiting, analytics

**Next Steps**:
- Phase 0: Execute research tasks (Gemini via HF validation, query classification strategies, Qdrant optimization, session management patterns, Neon schema design)
- Phase 1: Generate data-model.md, contracts/openapi.yaml, contracts/prompts.md, quickstart.md
- Run `/sp.tasks` to create actionable implementation tasks

## Outcome

- ✅ Impact: Comprehensive technical plan ready for implementation
- 🧪 Tests: N/A (planning phase - test strategy defined in plan)
- 📁 Files: specs/002-end-to-end-rag-assistant/plan.md
- 🔁 Next prompts: Execute Phase 0 research, then Phase 1 design artifacts, then `/sp.tasks`
- 🧠 Reflection: Plan successfully integrates user's Gemini-based architecture with existing backend while maintaining constitution compliance

## Evaluation notes (flywheel)

- Failure modes observed: None - plan aligns with spec and user input
- Graders run and results (PASS/FAIL): PASS - Constitution Check passed, all technical requirements addressed
- Prompt variant (if applicable): Standard /sp.plan workflow with user-provided architecture
- Next experiment (smallest change to try): Validate Gemini model access via Hugging Face Inference API in Phase 0 research
