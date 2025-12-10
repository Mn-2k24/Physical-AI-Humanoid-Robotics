# Feature Specification: End-to-End RAG Assistant

**Feature Branch**: `002-end-to-end-rag-assistant`
**Created**: 2025-12-10
**Status**: Draft
**Input**: Target audience: Developers building an end-to-end Retrieval-Augmented Generation (RAG) assistant for a digital book. Focus: Extract book content, generate embeddings and store in Qdrant, build RAG backend using FastAPI, integrate chat UI into Docusaurus book website.

## Clarifications

### Session 2025-12-10

- Q: Embedding model selection - confirm Hugging Face `sentence-transformers/all-MiniLM-L6-v2` (384 dimensions) or select alternative? → A: Confirm all-MiniLM-L6-v2 (384 dim) with constitutional deviation accepted, justified by: no API costs (local execution), no rate limiting, faster inference for < 2s latency target, sufficient quality for book content retrieval
- Q: Conversation history behavior - should each query be independent or maintain conversation context across queries? → A: Independent queries - each query retrieves fresh chunks with no conversation context between queries, simpler implementation, no state management complexity, aligns with grounding requirement
- Q: LLM service for answer generation - OpenAI API or alternative? → A: Hugging Face models (local execution, no API costs, no rate limiting, faster response times, full control over inference)
- Q: Reranking strategy in retrieval pipeline - simple similarity sorting or cross-encoder? → A: Simple similarity score sorting - use Qdrant scores directly to rerank top-10 results by descending similarity, select top-3, fast and simple, meets < 2s latency target
- Q: Context scope for /ask-local with selected text - same file, same section, or surrounding chunks? → A: Same file only - filter Qdrant search to chunks where file_path matches selected text's source file, clear chapter-level scoping, simple to implement

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Book Content Ingestion Pipeline (Priority: P1)

A developer wants to prepare the Docusaurus book for RAG by extracting all Markdown content from `/docs`, chunking it semantically, generating embeddings, and storing everything in Qdrant with accurate metadata.

**Why this priority**: This is the foundational data layer. Without embedded book content, the RAG assistant cannot answer queries. All other features (backend, UI) depend on having searchable vector embeddings of the book.

**Independent Test**: Can be fully tested by running the ingestion pipeline against `/docs` and verifying that Qdrant contains all chunks with correct vectors and metadata (file name, section, chunk index). Success means querying Qdrant for any book topic returns relevant chunks.

**Acceptance Scenarios**:

1. **Given** a `/docs` directory with Markdown files, **When** the ingestion pipeline runs, **Then** all `.md` files are read, chunked into 300-1,000 token segments, and stored in Qdrant collection `physical_ai_book` with embeddings
2. **Given** a Markdown file with headings and code blocks, **When** that file is chunked, **Then** chunks preserve semantic boundaries (no mid-sentence or mid-code-block splits) and each chunk includes metadata: file path, section heading, chunk index
3. **Given** the Qdrant free tier has storage limits, **When** the pipeline approaches quota, **Then** it detects the limit and halts with clear error message before exceeding quota
4. **Given** the ingestion pipeline completes, **When** querying Qdrant for a specific chapter topic, **Then** all relevant chunks from that chapter are returned with similarity scores and complete metadata

---

### User Story 2 - RAG Backend API (Priority: P2)

A developer wants a production-ready FastAPI backend that accepts user queries, retrieves relevant book chunks from Qdrant, reranks results, generates grounded answers using an LLM, and logs all conversations to Neon Postgres.

**Why this priority**: The backend is the core intelligence layer that connects the data (P1) with the user interface (P3). Without a reliable API, the chatbot cannot function. This must be complete before frontend integration.

**Independent Test**: Can be tested by sending HTTP requests to the FastAPI endpoints (`/ask`, `/ask-local`, `/track`) and verifying that responses are grounded in book content, retrieval is accurate, and analytics are logged to Postgres. No UI needed for testing.

**Acceptance Scenarios**:

1. **Given** a user query "How does ROS 2 work?", **When** the `/ask` endpoint is called, **Then** the backend retrieves top-K relevant chunks from Qdrant, generates an answer grounded strictly in those chunks, and returns the answer with source citations (file paths, sections)
2. **Given** a user selects text from a specific chapter file, **When** the `/ask-local` endpoint is called with the selected text, query, and source file path, **Then** the backend retrieves chunks only from the same `file_path` (chapter-level scoping), generates an answer strictly from those chunks, and returns the answer with citations pointing only to that file
3. **Given** a conversation between user and chatbot, **When** the `/track` endpoint is called, **Then** the conversation (query, answer, timestamp, sources) is logged to Neon Postgres for analytics
4. **Given** the backend receives 500 requests in one day, **When** approaching free-tier rate limits, **Then** it handles requests gracefully without crashing and returns clear error messages when limits are reached
5. **Given** a query that has no relevant book content, **When** the `/ask` endpoint is called, **Then** the backend returns "I cannot answer this based on the book content" instead of hallucinating
6. **Given** a query is submitted, **When** the end-to-end pipeline executes (embedding → retrieval → reranking → generation), **Then** the total latency is under 2 seconds for 90% of queries

---

### User Story 3 - Docusaurus Chat UI Integration (Priority: P3)

A reader wants a seamless chat interface embedded directly in the Docusaurus book that allows asking questions about the book, highlighting text for contextual queries, and viewing answers with source citations—all without leaving the book layout.

**Why this priority**: The UI is the user-facing layer that delivers the RAG experience. While critical for end-user value, it depends on the backend (P2) and data layer (P1) being complete. This is the final integration step.

**Independent Test**: Can be tested by loading the Docusaurus book in a browser, using the chat widget to ask questions, highlighting text to trigger contextual queries, and verifying that answers are displayed with proper source links. Backend must be running for full test.

**Acceptance Scenarios**:

1. **Given** a reader opens the Docusaurus book, **When** the page loads, **Then** a responsive chat widget appears (collapsed by default) that works on desktop and mobile browsers
2. **Given** a reader types a question in the chat widget, **When** they submit the query, **Then** the UI calls the `/ask` endpoint, displays a loading indicator, and shows the answer with clickable source citations linking to relevant book sections
3. **Given** a reader highlights text in a book chapter, **When** they right-click or tap the selection, **Then** a "Ask AI About This" option appears, and clicking it opens the chat widget with the selected text as context
4. **Given** the reader submits a query with selected text, **When** the UI calls the `/ask-local` endpoint with the source file path, **Then** the answer is strictly based on chunks from the same chapter file (`file_path` match), and source citations point only to that file
5. **Given** the chat widget displays an answer, **When** the reader clicks a source citation, **Then** the browser scrolls smoothly to the referenced section in the book with a highlight effect
6. **Given** the backend is unreachable or returns an error, **When** a query is submitted, **Then** the UI displays a user-friendly error message ("Chatbot temporarily unavailable, please try again") instead of crashing
7. **Given** a reader uses the chat widget, **When** they interact with the UI, **Then** no API keys or environment variables are exposed in browser network requests or console logs (security validation)

---

### Edge Cases

- **Empty or minimal Markdown files**: Files with < 50 words are skipped during ingestion and logged as warnings
- **Malformed Markdown**: Files with broken syntax are processed with best-effort parsing; formatting errors are logged but do not halt the pipeline
- **Extremely long documents**: Markdown files exceeding 50,000 tokens are split using recursive chunking with 50-token overlap to preserve context
- **Qdrant quota exhaustion**: Pipeline halts gracefully with error message and suggested mitigation (upgrade tier or reduce chunk size)
- **Model inference failures**: Backend retries with exponential backoff (max 3 retries); if exhausted, returns clear error to user
- **No relevant chunks found**: When Qdrant returns no results above similarity threshold (e.g., < 0.5), backend responds with "No relevant book content found for this query"
- **Ambiguous selected text**: If user highlights < 10 words, UI prompts "Please select more text for better context" before allowing query
- **Concurrent users**: Backend handles up to 10 concurrent requests without degradation; queues additional requests with timeout warnings
- **Backend downtime**: Frontend detects failed API calls (timeout or 5xx errors) and displays maintenance message with retry button
- **Neon Postgres connection failure**: Backend logs analytics failures but does not block query responses (analytics is non-critical path)

## Requirements *(mandatory)*

### Functional Requirements

**Book Content Ingestion (User Story 1)**

- **FR-001**: System MUST read all `.md` files from `/docs` directory recursively, including nested subdirectories
- **FR-002**: System MUST chunk each Markdown file into semantically coherent segments of 300-1,000 tokens per chunk, preserving paragraph and code block boundaries
- **FR-003**: System MUST generate vector embeddings for each chunk using Hugging Face `sentence-transformers/all-MiniLM-L6-v2` model (384 dimensions)
- **FR-004**: System MUST store embeddings in Qdrant Cloud Free Tier collection named `physical_ai_book` with metadata: `file_path`, `section_heading`, `chunk_index`, `word_count`, `created_at`
- **FR-005**: System MUST validate that Qdrant storage stays within free tier limits (1M vectors, 1GB memory) and halt with error if quota is reached
- **FR-006**: System MUST skip Markdown files with < 50 words and log them as warnings without halting the pipeline
- **FR-007**: System MUST handle malformed Markdown with best-effort parsing and log formatting errors as warnings
- **FR-008**: System MUST split Markdown files exceeding 50,000 tokens using recursive chunking with 50-token overlap

**RAG Backend API (User Story 2)**

- **FR-009**: System MUST provide a FastAPI backend with endpoint `/ask` that accepts JSON: `{"query": "string"}` and returns JSON: `{"answer": "string", "sources": [{"file": "string", "section": "string"}]}`
- **FR-010**: System MUST provide endpoint `/ask-local` that accepts JSON: `{"query": "string", "context": "string", "source_file": "string"}` where `context` is user-selected text, and retrieves chunks only from the same `file_path` as the selected text (chapter-level scoping)
- **FR-011**: System MUST provide endpoint `/track` that accepts JSON: `{"query": "string", "answer": "string", "sources": [], "timestamp": "ISO8601"}` and logs it to Neon Postgres table `conversations`
- **FR-012**: System MUST implement retrieval pipeline: embed query → search Qdrant (top-K=10) → rerank by descending similarity score → select top-3 chunks → generate answer
- **FR-013**: System MUST generate answers strictly grounded in retrieved chunks with no external knowledge or hallucination
- **FR-014**: System MUST include source citations in answers with format: "According to [Chapter X - Section Y]..."
- **FR-015**: System MUST handle model inference errors with exponential backoff (3 retries max) and return error message if exhausted
- **FR-016**: System MUST respond with "No relevant book content found" when Qdrant similarity scores are all below 0.5 threshold
- **FR-017**: System MUST achieve end-to-end query latency < 2 seconds for 90% of requests
- **FR-018**: System MUST handle up to 10 concurrent requests without degradation and queue additional requests with timeout warnings
- **FR-019**: System MUST store all API keys and environment variables in `.env` file with zero exposure in logs or responses
- **FR-020**: System MUST log analytics failures separately without blocking query responses (non-critical path)
- **FR-021**: System MUST process each query independently with no conversation history - each query retrieves fresh chunks without context from previous Q&A pairs

**Docusaurus Chat UI (User Story 3)**

- **FR-022**: System MUST embed a responsive chat widget in Docusaurus layout that works on desktop and mobile browsers
- **FR-023**: System MUST display chat widget collapsed by default with expand/collapse toggle button
- **FR-024**: System MUST call `/ask` endpoint when user submits query without selected text and display answer with loading indicator
- **FR-025**: System MUST allow text highlighting in book content and show "Ask AI About This" context menu option on selection
- **FR-026**: System MUST call `/ask-local` endpoint when user submits query with selected text (> 10 words), including the source file path derived from current page URL
- **FR-027**: System MUST display answers with clickable source citations that scroll to referenced sections with highlight effect
- **FR-028**: System MUST display user-friendly error messages when backend is unreachable or returns errors
- **FR-029**: System MUST validate that no API keys or secrets are exposed in browser network requests or console logs
- **FR-030**: System MUST prompt "Please select more text for better context" when user highlights < 10 words before querying
- **FR-031**: System MUST use HTTPS only for all backend communication (no HTTP fallback)

### Key Entities

**Book Content Layer**

- **MarkdownFile**: Represents a source document from `/docs`. Attributes: `file_path` (string), `raw_content` (string), `word_count` (int), `last_modified` (datetime), `is_valid` (boolean).

- **TextChunk**: A semantically coherent segment of a Markdown file. Attributes: `chunk_id` (UUID), `file_path` (string), `section_heading` (string), `chunk_index` (int, 0-based), `text_content` (string), `token_count` (int, 300-1,000), `created_at` (datetime).

- **VectorEmbedding**: The numerical vector representation of a TextChunk. Attributes: `chunk_id` (UUID, foreign key to TextChunk), `embedding_vector` (array of 384 floats, using sentence-transformers/all-MiniLM-L6-v2), `model_name` (string), `created_at` (datetime).

**RAG Backend Layer**

- **Query**: User input to the chatbot. Attributes: `query_id` (UUID), `query_text` (string), `context_text` (optional string for selected text), `query_type` (enum: "full_book" or "local"), `timestamp` (datetime).

- **RetrievalResult**: Retrieved chunks from Qdrant. Attributes: `query_id` (UUID), `chunk_id` (UUID), `similarity_score` (float, 0-1), `rerank_score` (optional float), `final_rank` (int, 1-10).

- **ChatbotAnswer**: Generated response to user query. Attributes: `answer_id` (UUID), `query_id` (UUID), `answer_text` (string), `sources` (array of objects: `[{file_path, section_heading, chunk_index}]`), `latency_ms` (int), `timestamp` (datetime).

**Analytics Layer**

- **Conversation**: Logged interaction for analytics. Attributes: `conversation_id` (UUID), `query_id` (UUID), `answer_id` (UUID), `user_session_id` (optional UUID), `timestamp` (datetime), stored in Neon Postgres table `conversations`.

**UI Layer**

- **ChatWidget**: React component embedded in Docusaurus. State: `isOpen` (boolean), `messages` (array of `{role: "user" | "assistant", content: string, sources?: []}` for display only - not used for conversation context), `isLoading` (boolean), `error` (optional string). Note: Each query is processed independently without conversation history.

- **TextSelection**: User-highlighted text in book. Attributes: `selected_text` (string), `source_file` (string, derived from current page URL), `start_offset` (int), `end_offset` (int).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All Markdown files in `/docs` are successfully ingested, chunked, embedded, and stored in Qdrant with 100% of files processed (excluding < 50 word files)
- **SC-002**: Qdrant collection `physical_ai_book` contains accurate vectors and metadata for every chunk with validation showing no missing `file_path`, `section_heading`, or `chunk_index` fields
- **SC-003**: FastAPI backend `/ask` endpoint returns answers strictly grounded in book text for 95% of test queries with no hallucinated content (verified by human review of 100 test queries)
- **SC-004**: End-to-end retrieval pipeline (embedding → Qdrant search → reranking → answer generation) completes in under 2 seconds for 90% of queries (measured via backend latency logs)
- **SC-005**: Chat UI loads seamlessly inside Docusaurus book layout on desktop and mobile with no layout breaking or rendering issues
- **SC-006**: Text highlighting feature works across all major browsers (Chrome, Firefox, Safari, Edge) with "Ask AI About This" context menu appearing on selection
- **SC-007**: All API keys and environment variables are stored server-side in `.env` with zero exposure in frontend code, network requests, or browser console (verified by security audit)
- **SC-008**: Backend handles 500 requests/day within free-tier limits without degradation or crashes (load testing validation)
- **SC-009**: Analytics are successfully logged to Neon Postgres for 95% of conversations with complete data (query, answer, sources, timestamp)
- **SC-010**: All source citations in chatbot answers link correctly to book sections with smooth scrolling and highlight effects (manual UI testing across 50 queries)

## Assumptions

- `/docs` directory contains well-formed Markdown files following standard syntax (# headings, code blocks with triple backticks)
- Hugging Face `sentence-transformers/all-MiniLM-L6-v2` model (384 dimensions) is used for all embeddings
- Hugging Face models are used for answer generation (local execution with sufficient memory and compute resources)
- Qdrant Cloud Free Tier provides sufficient storage for all book chunks (estimated < 10,000 chunks for typical Docusaurus book)
- Neon Postgres Free Tier provides sufficient storage for conversation analytics (< 100MB for typical usage)
- Docusaurus is configured with React components support (swizzling enabled for custom widget integration)
- Users access the book via HTTPS (production deployment on Vercel or similar platform)
- English-only content (no multilingual embedding or translation requirements)
- Readers use modern browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)

## Out of Scope

- **PDF parsing or DOCX parsing**: Only Markdown (`.md`) files are supported as input
- **Model fine-tuning or custom LLM training**: Use pre-trained Hugging Face models as-is for both embeddings and answer generation
- **Analytics dashboard or admin panel**: Conversation logs stored in Postgres but no visualization UI
- **Multi-language support**: English-only for book content and UI (no translation or multilingual embeddings)
- **User authentication or payment system**: Chatbot is publicly accessible with no login or payment required
- **Advanced moderation or safety layers**: Basic prompt filtering only (no custom toxicity detection or content moderation beyond LLM's built-in safeguards)
- **Real-time collaboration or multi-user sessions**: Each reader's chat session is independent (no shared conversations)
- **Conversation history context**: Each query is processed independently - no multi-turn conversation or follow-up question handling using previous Q&A context
- **Mobile native apps**: Web-only UI (responsive design for mobile browsers, but no iOS/Android native apps)
- **A/B testing or feature flags**: Single production configuration (no experimentation framework)
- **Cross-encoder reranking models**: Use simple similarity score sorting from Qdrant (no BERT cross-encoder or learned-to-rank models for reranking)

## Constraints

### Data Constraints

- **Input Format**: Markdown (`.md`) files only
- **Source Directory**: `/docs` (relative to Docusaurus root)
- **Chunk Size**: 300-1,000 tokens per chunk
- **Chunk Overlap**: 50 tokens between adjacent chunks to preserve context

### Technology Constraints

- **Embedding Model**: Hugging Face `sentence-transformers/all-MiniLM-L6-v2` (384 dimensions)
- **Answer Generation Model**: Hugging Face models (local execution, specific model TBD during planning)
- **Vector Database**: Qdrant Cloud Free Tier (1M vectors max, 1GB memory max)
- **Backend Framework**: FastAPI with Python 3.11+
- **Frontend Framework**: Docusaurus (React-based)
- **Analytics Database**: Neon Serverless Postgres Free Tier

### Security Constraints

- **Transport**: HTTPS only for all API communication (no HTTP fallback)
- **Secrets Management**: All API keys stored in `.env` file on backend server, never in frontend code
- **Browser Session**: No personal data or conversation history stored in browser localStorage or cookies

### Performance Constraints

- **Query Latency**: < 2 seconds end-to-end for 90% of queries (including local model inference)
- **Concurrent Users**: Up to 10 concurrent requests without degradation
- **Daily Request Target**: Backend designed to handle 500 requests/day (Qdrant + Neon free tier limits, no external API rate limits for models)

### UI Constraints

- **Responsive Design**: Chat widget must work on desktop (1920x1080+) and mobile (375x667+)
- **Browser Compatibility**: Chrome 90+, Firefox 88+, Safari 14+, Edge 90+
- **Layout Integration**: Chat widget must load inside Docusaurus theme without breaking existing layout

### Operational Constraints

- **Grounding**: All chatbot answers must be strictly based on retrieved book chunks (no external knowledge)
- **Error Handling**: Backend failures must display user-friendly error messages in UI (no stack traces or technical errors)
- **Logging**: All analytics failures must not block query responses (logging is non-critical path)

## Quality Checklist *(mandatory)*

- [ ] **User Scenarios**: 3 user stories defined with clear priorities (P1, P2, P3)
- [ ] **Independent Testability**: Each user story can be tested independently (verified in acceptance scenarios)
- [ ] **Acceptance Scenarios**: 4+ scenarios per user story with Given/When/Then format
- [ ] **Edge Cases**: 10+ edge cases documented with clear handling strategies
- [ ] **Functional Requirements**: 30 functional requirements covering all user stories
- [ ] **Key Entities**: 8+ entities defined with attributes across data/backend/analytics/UI layers
- [ ] **Success Criteria**: 10 measurable outcomes with specific metrics and targets
- [ ] **Assumptions**: 10+ assumptions documented for planning phase
- [ ] **Out of Scope**: 10+ exclusions to prevent scope creep
- [ ] **Constraints**: Documented across data, technology, security, performance, UI, and operational dimensions
- [ ] **Constitution Alignment**: Aligns with Principle VII (RAG-Powered Interactive Learning) requirements
- [ ] **No Ambiguity**: All requirements use precise language (MUST/SHOULD/MAY) with measurable criteria

---

## Constitution Alignment Verification

### Principle VII: RAG-Powered Interactive Learning

✅ **Grounded Responses**: FR-013 ensures all answers are strictly from book content (no hallucinations)
✅ **Dual Query Modes**: FR-009 (`/ask` for full-book) and FR-010 (`/ask-local` for selected text) support both modes
✅ **Retrieval Pipeline**: FR-012 implements embeddings → Qdrant search → rerank → generation
✅ **Seamless Integration**: FR-021 embeds chat widget in Docusaurus UI
✅ **Text Selection Feature**: FR-024 supports "Ask AI About This" for highlighted text

**Technical Standards Alignment**:
✅ **Backend**: FastAPI with three endpoints (`/ask`, `/ask-local`, `/track`) - matches constitution
✅ **Vector Database**: Qdrant Cloud Free Tier, collection `physical_ai_book`, 1,000+ chunks - matches constitution
✅ **Embedding Dimension**: Constitutional deviation accepted - using Hugging Face all-MiniLM-L6-v2 (384 dim) instead of OpenAI (1536 dim), justified by: no API costs, no rate limiting, faster inference meeting < 2s latency target, sufficient quality for book retrieval
✅ **Storage**: Neon Postgres for analytics - matches constitution
✅ **Performance**: < 3 seconds query latency (constitution) → < 2 seconds in spec (more aggressive target)
✅ **Security**: All keys server-side (FR-019, FR-028) - matches constitution
✅ **Reliability**: 500 requests/day (FR-008) - matches constitution

---

## Notes for Planning Phase

1. **Embedding Model Confirmed**: Using `sentence-transformers/all-MiniLM-L6-v2` (384 dimensions) for all embeddings. Constitutional deviation from OpenAI (1536 dim) accepted with justification documented.

2. **Reranking Strategy Confirmed**: Using simple similarity score sorting - rerank Qdrant's top-10 results by descending similarity score, select top-3 for answer generation. No cross-encoder model needed.

3. **Answer Generation Model Selection**: Choose Hugging Face model for answer generation (e.g., `facebook/opt-1.3b`, `google/flan-t5-base`, or similar) that balances quality, memory requirements, and inference speed for < 2s latency target.

4. **Neon Postgres Schema**: Design `conversations` table schema during planning (columns: query_id, answer_id, query_text, answer_text, sources JSON, timestamp).

5. **Docusaurus Integration Strategy**: Decide between custom React component vs. Docusaurus plugin. Custom component offers more control but requires swizzling theme.

6. **Text Selection UI**: Determine browser API for text selection (Selection API + range) and context menu implementation (custom overlay vs. browser context menu).

7. **Error Handling Granularity**: Define specific error codes for different failure modes (Qdrant timeout, model inference failure, no results found, etc.) to enable better frontend UX.

8. **Analytics Schema**: Clarify what metrics to log beyond basic conversation data (e.g., retrieval scores, latency breakdown, user feedback?).
