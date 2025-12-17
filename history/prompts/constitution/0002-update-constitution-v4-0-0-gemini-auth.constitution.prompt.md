---
id: 0002
title: Update constitution v4.0.0 gemini auth
stage: constitution
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: nizam
command: /sp.constitution
labels: ["constitution", "major-version", "gemini-api", "better-auth", "breaking-change"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A
---

## Prompt

# Constitution: Integrated RAG Chatbot & Authentication System

## 1. Purpose and Scope

This system defines the mandatory architectural, behavioral, and implementation rules for building:

1. A **Retrieval-Augmented Generation (RAG) chatbot** embedded inside a Docusaurus-based published book.
2. A **user authentication system (Signup & Signin)** implemented using https://www.better-auth.com, including frontend UI and backend integration.

This constitution is authoritative and must be followed by all generated specifications, plans, and implementations.

---

## 2. Core Principles

- The chatbot must be **strictly grounded in book content**.
- The system must be **secure, auditable, and scalable**.
- Authentication must enable **future personalization** based on user background.
- Backend services must be **API-driven and decoupled from frontend logic**.

---

## 3. RAG Chatbot Constraints (Critical)

### 3.1 Knowledge Boundaries

The chatbot MUST:

- Answer questions **only using retrieved book content**.
- Refuse or safely respond with:
  > "This information is not available in the book."

When no relevant content is found.

The chatbot MUST NOT:
- Hallucinate
- Use external or general world knowledge
- Answer speculative or unrelated questions

---

### 3.2 Text-Selection-Based Queries (Local RAG)

The system MUST support a mode where:

- Users select a portion of text from the book
- The chatbot answers questions **only from that selected text**
- No other book chapters or documents are allowed in context

This rule is absolute.

---

## 4. System Architecture Awareness

The system exists in a backend-driven pipeline consisting of:

- **FastAPI** as the backend framework
- **Qdrant Cloud (Free Tier)** as the vector database
- **Neon Serverless Postgres** for persistence and analytics
- **LLM & Embeddings via Gemini API** (not OpenAI, not Hugging Face)

Frontend chatbot implementation details are **explicitly excluded** from this constitution.

---

## 5. Embeddings and Retrieval Rules

- Book content MUST be chunked into semantically meaningful sections.
- Chunks must preserve:
  - Paragraph boundaries
  - Section headings
- Each chunk must be embedded and stored in Qdrant with metadata:
  - file_path
  - section_heading
  - chunk_index

Query flow:
1. User query → embedding
2. Vector similarity search (top-k)
3. Context filtering (global or selected-text only)
4. Answer generation using retrieved chunks

---

## 6. Answer Generation Rules

The answer model MUST:

- Use retrieved chunks as the **only source of truth**
- Generate concise, factual answers
- Include implicit grounding to the source text
- Never fabricate missing information

If similarity scores are below threshold, respond with:
> "No relevant information found in this book."

---

## 7. Authentication System (Mandatory UI + Backend)

### 7.1 Authentication Provider

- Authentication MUST be implemented using:
  https://www.better-auth.com/
- Custom authentication logic is NOT allowed.

---

### 7.2 Signup Requirements

During signup, the system MUST collect:

- Software background:
  - Programming languages
  - Frameworks
  - Experience level
- Hardware background:
  - CPU / GPU availability
  - Robotics / AI hardware (if any)

This data will be stored in Neon Postgres for future personalization.

---

### 7.3 Signin Requirements

- Secure signin using Better Auth
- Session-based or token-based authentication
- Backend must validate authentication state for protected endpoints

---

### 7.4 Frontend Requirements (Auth ONLY)

Frontend implementation MUST be provided for:

- Signup UI
- Signin UI
- Background questionnaire UI

Frontend implementation MUST NOT include:
- Chatbot UI logic
- Chat rendering logic
- Message streaming logic

---

## 8. Data Persistence Rules

Neon Serverless Postgres MUST store:

- User accounts
- Signup background questionnaire responses
- Chat interaction metadata (query, timestamps, sources)

No embeddings are stored in Postgres.

---

## 9. Security and Privacy

- API keys must be stored only in environment variables
- No secrets may be exposed to frontend
- Authentication endpoints must be rate-limited
- User background data must not be exposed to the chatbot directly

---

## 10. Non-Negotiable Constraints

- No OpenAI APIs
- No Hugging Face APIs
- No chatbot frontend implementation
- No ungrounded answers
- No external knowledge usage

---

## 11. Acceptance Criteria

The system is considered valid ONLY if:

- The chatbot answers strictly from book content
- Selected-text queries are fully isolated
- Authentication works end-to-end
- Signup collects user background data
- Backend APIs are cleanly separated from frontend concerns

Failure to meet any requirement invalidates the implementation.

---

## Generate full constitution

## Response snapshot

Successfully updated constitution to version 4.0.0 with the following changes:

**Version Change**: 3.0.0 → 4.0.0 (MAJOR)

**Key Modifications**:
1. Migrated RAG infrastructure from OpenAI to Gemini API (BREAKING CHANGE)
   - Embeddings: Gemini API embeddings
   - Answer Generation: Gemini API LLM
2. Made authentication mandatory with Better Auth integration
3. Explicitly excluded chatbot frontend UI (backend endpoints only)
4. Added strict grounding requirements (zero hallucinations, zero external knowledge)
5. Added selected-text mode with absolute context isolation
6. Expanded Principle VIII with detailed user background collection schema
7. Added comprehensive Non-Negotiable Constraints section

**Modified Principles**:
- Principle VII: Complete RAG infrastructure overhaul (OpenAI → Gemini)
- Principle VIII: Expanded authentication requirements with detailed UI/backend specs

**Added Sections**:
- Authentication frontend UI requirements (Signup/Signin components)
- User background questionnaire schema (software/hardware)
- Neon Postgres data persistence requirements
- Security and privacy requirements
- Non-Negotiable Constraints section (10 absolute rules)

**Template Alignment**:
- plan-template.md: Constitution Check will validate Gemini usage and Better Auth
- spec-template.md: User scenarios support authenticated interactions
- tasks-template.md: Task structure accommodates auth + RAG backend tasks

**File Updated**:
- `.specify/memory/constitution.md` (601 lines)

**Follow-up Actions Required**:
- Migrate existing RAG embeddings from OpenAI to Gemini
- Implement Better Auth FastAPI integration
- Create Neon Postgres schema for user accounts
- Build Signup/Signin UI components
- Update environment variables for Gemini API keys
- Remove OpenAI dependencies from backend
- Test authentication flow end-to-end
- Validate RAG pipeline with Gemini embeddings

## Outcome

- ✅ Impact: MAJOR version bump (4.0.0) - backward-incompatible infrastructure change requiring full migration from OpenAI to Gemini API and mandatory Better Auth integration
- 🧪 Tests: No automated tests for constitution (validation via template alignment and compliance reviews)
- 📁 Files: 1 file updated (.specify/memory/constitution.md)
- 🔁 Next prompts: /sp.specify or /sp.plan to create feature specifications following new constitution requirements for Gemini API and Better Auth
- 🧠 Reflection: This is a significant architectural shift - moving from OpenAI to Gemini fundamentally changes the RAG implementation and requires careful migration planning to avoid service disruption

## Evaluation notes (flywheel)

- Failure modes observed: None - constitution update successful with complete Sync Impact Report
- Graders run and results (PASS/FAIL): Manual validation - all placeholders filled, version incremented correctly, Sync Impact Report complete
- Prompt variant (if applicable): Standard /sp.constitution workflow
- Next experiment (smallest change to try): Validate constitution compliance by running /sp.specify for authentication feature to ensure templates align with new Gemini API and Better Auth requirements
