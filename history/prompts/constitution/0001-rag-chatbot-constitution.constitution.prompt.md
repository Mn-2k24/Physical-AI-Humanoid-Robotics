---
id: 0001
title: RAG Chatbot Constitution
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4.5
feature: none
branch: main
user: nizam
command: /sp.constitution
labels: ["constitution", "rag", "chatbot", "interactive-learning", "infrastructure"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - Constitution validation: All principles defined
  - Template alignment: plan-template.md, spec-template.md, tasks-template.md reviewed
  - Version increment: MAJOR bump from 1.0.0 to 2.0.0
---

## Prompt

Project: Integrated RAG Chatbot Development for Docusaurus Book

Core principles:
- Reliability through retrieval-augmented generation (RAG)
- Consistency in responses sourced strictly from book content
- Security of API keys and user data
- Efficiency in vector search and database operations
- Seamless integration with Docusaurus front-end

Key standards:
- All chatbot answers must be grounded in stored book content (no hallucinations)
- Retrieval pipeline: OpenAI embeddings → Qdrant similarity search → rerank → answer
- Chatbot must support two modes: full-book queries & user-selected-text queries

Must use the following technologies:
- OpenAI Agents / ChatKit SDKs
- FastAPI backend
- Neon Serverless Postgres (for logs, conversations, analytics)
- Qdrant Cloud Free Tier (vector DB)
- Frontend must integrate directly into Docusaurus using custom React components
- UI must support text selection → "Ask AI About This" action
- Logging, embeddings, and metadata must follow strict schema stored in Neon Postgres

Constraints:
- Vector collection name: physical_ai_book
- Minimum 1,000 text chunks embedded for full-book coverage
- Embedding dimension: 1536
- Latency requirement: < 3 seconds per query
- All environment variables stored server-side only

Backend must expose a maximum of three endpoints:
- /ask (full-book RAG)
- /ask-local (selected-text only)
- /track (conversation analytics → Neon Postgres)

Success criteria:
- Chatbot functions fully inside the Docusaurus book UI
- Responses are accurate and traceable to the book sections used
- Selected-text mode always ignores all unrelated content
- End-to-end pipeline deployed and publicly accessible
- Backend stable under 500 requests/day free-tier limits
- All logs/analytics visible in Neon dashboard
- Zero leakage of protected API keys in the frontend

## Response snapshot

Constitution successfully amended to version 2.0.0 with comprehensive RAG chatbot integration requirements.

**Key Changes**:

1. **New Principle VII: RAG-Powered Interactive Learning**
   - Comprehensive chatbot requirements covering grounded responses, dual query modes, and retrieval pipeline
   - Technical standards specifying FastAPI backend, Qdrant vector DB, and Neon Postgres
   - Security requirements for API key management (server-side only)
   - Performance requirements (< 3 seconds latency, 500 requests/day stability)

2. **Updated Project Overview**
   - Added "Interactive Features: RAG-powered AI chatbot for content exploration"
   - Updated publishing platform to "Vercel (primary), GitHub Pages"

3. **Enhanced Core Purpose**
   - Included "enhanced with an intelligent chatbot for personalized learning assistance"

4. **Expanded Focus Areas**
   - Added "Interactive learning through RAG-powered AI assistance"

5. **Updated Technical Workflow**
   - Added RAG Infrastructure section with FastAPI, OpenAI API, Qdrant Cloud, Neon Postgres
   - Added Development Rule #6 for RAG chatbot testing
   - Added Workflow Step #8 for RAG integration validation

6. **Enhanced Constraints**
   - Added comprehensive RAG Infrastructure constraints
   - Added Quality Constraints for chatbot accuracy and security

7. **Expanded Success Criteria**
   - Added 7 new criteria (#10-16) for chatbot functionality, accuracy, security, and performance

8. **Updated Book Versioning**
   - Added v2.0+ versions with RAG chatbot milestones

9. **Enhanced Review Checkpoints**
   - Added RAG-specific validation steps for v2.0+ content

**Version Bump Rationale**: MAJOR (1.0.0 → 2.0.0)
- Backward-incompatible change introducing mandatory interactive features
- New technical infrastructure requirements (FastAPI, Qdrant, Neon Postgres)
- Fundamental scope change from static documentation to interactive learning platform
- New mandatory success criteria that alter project definition

**Sync Impact Report** (in constitution file):
- Templates reviewed and confirmed compatible: plan-template.md, spec-template.md, tasks-template.md
- Follow-up TODOs identified: Backend implementation, UI integration, Postgres setup, embedding pipeline

## Outcome

- ✅ Impact: Constitution v2.0.0 ratified with comprehensive RAG chatbot governance
- 🧪 Tests: All validation checks passed - no unresolved placeholders, dates correct (ISO 8601), version coherent
- 📁 Files: 1 file modified (.specify/memory/constitution.md)
- 🔁 Next prompts: Use `/sp.specify` to create RAG chatbot feature specification, then `/sp.plan` for architecture design
- 🧠 Reflection: This is a significant project evolution. The constitution now mandates interactive learning features while maintaining the original commitment to technical accuracy, reproducibility, and academic rigor. The RAG requirements are specific enough to guide implementation while remaining technology-appropriate for the educational context.

## Evaluation notes (flywheel)

- Failure modes observed: None - all constitutional requirements clearly defined and validated
- Graders run and results (PASS/FAIL): PASS - Constitution template fully populated, no placeholder tokens remaining, version semantics correct, Sync Impact Report complete
- Prompt variant (if applicable): N/A (constitution amendment, not iterative prompt optimization)
- Next experiment (smallest change to try): Validate that existing Docusaurus content structure supports effective RAG chunking (may need to adjust heading hierarchy or add section metadata for optimal retrieval)
