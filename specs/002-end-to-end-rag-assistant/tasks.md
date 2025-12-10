# Tasks: End-to-End RAG Assistant

**Feature**: 002-end-to-end-rag-assistant | **Branch**: `002-end-to-end-rag-assistant`
**Input**: Design documents from `/specs/002-end-to-end-rag-assistant/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Organization**: Tasks are grouped by user story (P1: Ingestion, P2: Backend, P3: UI) to enable independent implementation and testing of each story.

**Tests**: No explicit test requirements found in spec - tests are OPTIONAL. Focus on implementation with manual validation per acceptance scenarios.

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- All file paths are absolute from repository root

## Path Conventions

Per plan.md structure:
- **Backend**: `/home/nizam/Physical-AI-Humanoid-Robotics/backend/`
- **Frontend**: `/home/nizam/Physical-AI-Humanoid-Robotics/src/`
- **Docs**: `/home/nizam/Physical-AI-Humanoid-Robotics/docs/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend directory structure: backend/src/{api,services,core,models}, backend/scripts/, backend/tests/{unit,integration}
- [X] T002 Initialize Python virtual environment and create backend/requirements.txt with dependencies: fastapi==0.104.0, uvicorn==0.24.0, sentence-transformers==2.2.2, transformers==4.35.0, qdrant-client==1.7.0, psycopg2-binary==2.9.9, mistune==3.0.2, tiktoken==0.5.1, pydantic==2.5.0, python-dotenv==1.0.0, httpx==0.25.0
- [X] T003 [P] Create backend/.env.example template with placeholders for QDRANT_API_KEY, QDRANT_ENDPOINT, QDRANT_COLLECTION_NAME, NEON_CONNECTION_STRING, EMBEDDING_MODEL_NAME, ANSWER_MODEL_NAME
- [X] T004 [P] Create backend/README.md with setup instructions referencing quickstart.md
- [X] T005 [P] Add backend/.env to .gitignore

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Implement backend/src/core/config.py: Load .env variables using python-dotenv, validate all required environment variables (QDRANT_*, NEON_*, MODEL_NAMES), raise ValueError if missing
- [X] T007 [P] Implement backend/src/core/constants.py: Define MAX_CHUNK_SIZE_TOKENS=1000, MIN_CHUNK_SIZE_TOKENS=300, CHUNK_OVERLAP_TOKENS=50, TOP_K_RETRIEVAL=10, TOP_N_RESULTS=3
- [X] T008 [P] Create backend/src/models/__init__.py with exports for all model classes
- [X] T009 Create backend/Dockerfile: Use python:3.11-slim base, install requirements.txt, pre-download sentence-transformers/all-MiniLM-L6-v2 and google/flan-t5-base models at build time, expose port 8000, CMD uvicorn

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Book Content Ingestion Pipeline (Priority: P1) 🎯 MVP

**Goal**: Extract all 33 Markdown files from /docs, chunk semantically (300-1,000 tokens with 50-token overlap), generate 384-dim embeddings using all-MiniLM-L6-v2, store in Qdrant collection `physical-ai-book` with complete metadata

**Independent Test**: Run backend/scripts/ingest.py against /docs directory, verify Qdrant contains ~100 chunks with file_path, section, chunk_index metadata, query Qdrant for any book topic and confirm relevant chunks returned with similarity scores

### Implementation for User Story 1

- [X] T010 [P] [US1] Create backend/src/models/chunk.py: Implement MarkdownFile Pydantic model with fields file_path, raw_content, ast, heading_path, file_size_bytes, last_modified, add validator for file_path (must start with 'docs/' and end with '.md')
- [X] T011 [P] [US1] Create backend/src/models/chunk.py: Implement TextChunk Pydantic model with fields chunk_id (UUID), file_path, section, text, token_count (300-1,000), chunk_index (≥0), overlap_with_prev, overlap_with_next, add validator for text (non-empty after stripping)
- [X] T012 [US1] Implement backend/src/services/chunking.py: Function chunk_section(section_text, section_heading, file_path, chunk_index_start) - recursive splitting on paragraphs with 50-token overlap using tiktoken cl100k_base encoding, returns List[TextChunk] with token_count 300-1,000
- [X] T013 [US1] Implement backend/src/services/chunking.py: Function process_markdown_file(file_path) - parse with mistune v3, extract sections by headings (h1, h2), call chunk_section for each, return List[TextChunk] for entire file
- [X] T014 [US1] Implement backend/src/services/embeddings.py: Initialize SentenceTransformer('sentence-transformers/all-MiniLM-L6-v2'), function generate_embeddings(chunks: List[TextChunk]) → List[List[float]] with 384 dimensions per vector
- [X] T015 [US1] Implement backend/scripts/verify_qdrant.py: Connect to Qdrant using QDRANT_API_KEY and QDRANT_ENDPOINT from .env, check if collection QDRANT_COLLECTION_NAME exists, if not create with VectorParams(size=384, distance=Distance.COSINE), print collection info
- [X] T016 [US1] Implement backend/scripts/ingest.py: Scan /docs directory recursively for *.md files (use pathlib.Path.rglob), skip files < 50 words (log warning), process each with process_markdown_file, generate embeddings with generate_embeddings, upload to Qdrant as PointStruct with payload {file_path, section, text, token_count, chunk_index, overlap_with_prev, overlap_with_next}
- [X] T017 [US1] Add error handling to backend/scripts/ingest.py: Catch mistune parsing errors (log warning, continue), catch Qdrant quota errors (halt with clear message), log progress (files processed, chunks created, time elapsed)

**Checkpoint**: At this point, all book content should be in Qdrant with embeddings. Test by running: `python backend/scripts/verify_qdrant.py` then `python backend/scripts/ingest.py`

---

## Phase 4: User Story 2 - RAG Backend API (Priority: P2)

**Goal**: Build FastAPI backend with 3 endpoints (/ask for full-book queries, /ask-local for chapter-scoped queries, /track for analytics), implement retrieval pipeline (embed query → Qdrant search top-10 → rerank by similarity → select top-3 → generate answer with flan-t5-base), log all conversations to Neon Postgres

**Independent Test**: Start FastAPI with `uvicorn backend.src.main:app --reload`, send POST requests to /ask and /ask-local with curl, verify responses include grounded answers with source citations (file_path, section, similarity_score), check Neon Postgres conversations table contains logged entries

### Implementation for User Story 2

- [X] T018 [P] [US2] Create backend/src/models/query.py: Implement Query Pydantic model with fields query_id (UUID), query_text (1-500 chars), query_type ('full_book' or 'local'), selected_text (optional), source_file_path (optional), timestamp, add model_validator to ensure selected_text and source_file_path required when query_type=='local'
- [X] T019 [P] [US2] Create backend/src/models/query.py: Implement RetrievedChunk Pydantic model with fields chunk_id, file_path, section, text, similarity_score (0.0-1.0), rank (1-3)
- [X] T020 [P] [US2] Create backend/src/models/query.py: Implement RetrievalResult Pydantic model with fields query_id, chunks (List[RetrievedChunk], length exactly 3), retrieval_latency_ms, add validator to ensure chunks sorted by similarity_score descending and ranks are [1,2,3]
- [X] T021 [P] [US2] Create backend/src/models/answer.py: Implement SourceCitation Pydantic model with fields file_path, section, chunk_index, similarity_score
- [X] T022 [P] [US2] Create backend/src/models/answer.py: Implement ChatbotAnswer Pydantic model with fields query_id, answer_text (1-1000 chars), sources (List[SourceCitation], length exactly 3), generation_latency_ms, total_latency_ms, timestamp, add validator to ensure total_latency_ms >= generation_latency_ms
- [X] T023 [US2] Implement backend/src/services/retrieval.py: Initialize QdrantClient from config, function search_qdrant(query_embedding: List[float], query_type: str, source_file_path: Optional[str] = None) → List[ScoredPoint] - search collection with top_k=10, if query_type=='local' add filter for file_path match, return Qdrant results
- [X] T024 [US2] Implement backend/src/services/retrieval.py: Function rerank_and_select(scored_points: List[ScoredPoint]) → List[RetrievedChunk] - sort by score descending, take top 3, map to RetrievedChunk with rank 1-3
- [X] T025 [US2] Implement backend/src/services/generation.py: Initialize AutoTokenizer and AutoModelForSeq2SeqLM for 'google/flan-t5-base', function generate_answer(query: str, chunks: List[RetrievedChunk]) → str - construct prompt "Answer the question based only on the following context:\n\nContext: {chunks}\n\nQuestion: {query}\n\nAnswer:", generate with model, return decoded answer (max 200 tokens)
- [X] T026 [US2] Implement backend/scripts/init_neon.py: Connect to Neon Postgres using NEON_CONNECTION_STRING from .env, create table conversations (conversation_id UUID PRIMARY KEY, query_id UUID, query_text TEXT, answer_text TEXT, sources JSONB, query_type VARCHAR(20) CHECK IN ('full_book','local'), latency_ms INTEGER, timestamp TIMESTAMPTZ DEFAULT NOW(), user_session_id UUID), create indexes: idx_conversations_timestamp DESC, idx_conversations_query_type, idx_conversations_sources GIN
- [X] T027 [US2] Implement backend/src/services/analytics.py: Initialize psycopg2 connection from config, function log_conversation(query_id, query_text, answer_text, sources: List[SourceCitation], query_type, latency_ms, user_session_id=None) - insert into conversations table with sources as JSONB array, handle connection failures gracefully (log error but don't raise)
- [X] T028 [P] [US2] Create backend/src/api/schemas.py: Define AskRequest(query: str), AskLocalRequest(query: str, selected_text: str, source_file_path: str), AskResponse(query_id: str, answer: str, sources: List[SourceCitation], latency_ms: int), TrackRequest(query_id: str, query_text: str, answer_text: str, sources: List[dict], query_type: str, latency_ms: int), TrackResponse(conversation_id: str, status: str), HealthResponse(status: str, qdrant_connected: bool, neon_connected: bool, model_loaded: bool)
- [X] T029 [US2] Implement backend/src/api/routes.py: POST /ask endpoint - validate AskRequest, embed query with embeddings.generate_embeddings([query_text])[0], call search_qdrant with query_type='full_book', rerank_and_select top-3, generate_answer, construct ChatbotAnswer with sources, return AskResponse, measure total latency
- [X] T030 [US2] Implement backend/src/api/routes.py: POST /ask-local endpoint - validate AskLocalRequest, embed query, call search_qdrant with query_type='local' and source_file_path filter, rerank_and_select top-3, generate_answer, return AskResponse with latency
- [X] T031 [US2] Implement backend/src/api/routes.py: POST /track endpoint - validate TrackRequest, call analytics.log_conversation, return TrackResponse with conversation_id and status='logged'
- [X] T032 [P] [US2] Implement backend/src/api/routes.py: GET /health endpoint - check Qdrant connection (list collections), check Neon connection (simple SELECT 1), check models loaded (sentence-transformers and flan-t5-base in memory), return HealthResponse
- [X] T033 [US2] Create backend/src/main.py: Initialize FastAPI app with CORS middleware (origins from API_CORS_ORIGINS env var), include routes from api.routes, add startup event to pre-load models (sentence-transformers, flan-t5-base) and connect to Qdrant/Neon, add exception handler for ValueError/HTTPException to return user-friendly error JSON
- [X] T034 [US2] Add error handling to backend/src/api/routes.py: Catch QdrantException (timeout, quota) → return 503 with message, catch model inference errors → retry with exponential backoff (max 3), if all retries fail → return 500 with message, if similarity_score < 0.5 for all chunks → return 200 with answer "No relevant book content found for this query"

**Checkpoint**: At this point, FastAPI backend should be fully functional. Test with: `uvicorn backend.src.main:app --host 0.0.0.0 --port 8000` then `curl -X POST http://localhost:8000/api/ask -d '{"query":"What is ROS 2?"}'`

---

## Phase 5: User Story 3 - Docusaurus Chat UI Integration (Priority: P3)

**Goal**: Build responsive chat UI (ChatWidget) integrated into Docusaurus with floating button (bottom-right), slide-over panel, highlight-to-ask feature, source citations with smooth scrolling, mobile support

**Independent Test**: Start Docusaurus with `npm start`, open browser to localhost:3000, click floating chat button to open panel, submit test query "What is Physical AI?", verify answer displays with source citations, highlight text in a chapter and click "Ask AI About This", verify contextual query submitted to /ask-local

### Implementation for User Story 3

- [X] T035 [P] [US3] Create src/components/ChatWidget/index.tsx: Export default ChatWidget component
- [X] T036 [P] [US3] Create src/components/ChatWidget/ChatButton.tsx: Floating button component (fixed position bottom-right: right-6, bottom-6), circular button with 💬 icon, onClick toggles isOpen state, styled with Tailwind or CSS module, z-index 1000
- [X] T037 [P] [US3] Create src/components/ChatWidget/ChatPanel.tsx: Slide-over panel component (fixed position right side, width 400px), slide-in animation (transform translateX), contains header with close button, MessageList, and InputBox, styled for desktop and mobile (@media queries for < 768px width)
- [X] T038 [P] [US3] Create src/components/ChatWidget/MessageList.tsx: Scrollable message list component, renders messages from state.messages array, displays user messages (right-aligned, blue background) and assistant messages (left-aligned, gray background with source citations as clickable links), auto-scrolls to bottom on new messages
- [X] T039 [P] [US3] Create src/components/ChatWidget/InputBox.tsx: Input field + submit button component, textarea for multi-line input, submit button disabled while isLoading, Enter key submits (Shift+Enter for new line), clears input after submit
- [X] T040 [P] [US3] Create src/components/ChatWidget/styles.module.css: Component-specific styles for ChatButton, ChatPanel, MessageList, InputBox, source citations, loading spinner, error banner
- [X] T041 [US3] Create src/theme/Root.tsx: Wrap Docusaurus app with ChatProvider context, implement useChatWidget hook with state {isOpen, messages, isLoading, error} and actions {togglePanel, sendMessage, clearError}, sendMessage function calls fetch(/api/ask or /api/ask-local) based on query type, updates messages array with user question and assistant response
- [X] T042 [US3] Update src/theme/Root.tsx sendMessage function: Detect query type (if selectedText and sourceFilePath provided → /ask-local, else → /ask), construct request body, handle fetch errors (timeout, 5xx) → set error state, parse response and append to messages array with timestamp
- [X] T043 [P] [US3] Create src/components/TextSelectionMenu/index.tsx: Component that listens for text selection events (window.getSelection()), displays "Ask AI About This" button as floating overlay near selection, onClick gets selected text + derives source_file_path from window.location.pathname, opens ChatWidget with pre-filled query context
- [X] T044 [P] [US3] Create src/components/TextSelectionMenu/styles.module.css: Floating menu styles (absolute position, z-index 999, small button with AI icon)
- [X] T045 [US3] Update docusaurus.config.ts: Add customFields.apiUrl with value `process.env.NODE_ENV === 'production' ? 'https://your-render-app.onrender.com' : 'http://localhost:8000'`, use this in Root.tsx fetch calls
- [X] T046 [US3] Update src/css/custom.css: Add global styles for chat widget (z-index layers, mobile responsive breakpoints, smooth animations for slide-in/out transitions)
- [X] T047 [US3] Add source citation click handling to MessageList.tsx: Parse source file_path to construct Docusaurus URL (e.g., docs/01-introduction/overview.md → /docs/module-1-ros2/overview#section-heading), implement smooth scroll with document.querySelector and scrollIntoView({behavior: 'smooth'}), add highlight effect (temporary yellow background for 2 seconds)
- [X] T048 [US3] Add validation to InputBox.tsx and TextSelectionMenu: If selected text < 10 words, display prompt "Please select more text for better context" and disable submit, if query_text empty → disable submit
- [X] T049 [US3] Update package.json: Add axios dependency if not using fetch, add lucide-react or react-icons for chat icons

**Checkpoint**: All user stories should now be independently functional. Test end-to-end: (1) Start backend `uvicorn backend.src.main:app`, (2) Start frontend `npm start`, (3) Open localhost:3000, click chat button, submit queries, highlight text and use "Ask AI About This"

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T050 [P] Update README.md at repository root: Add "RAG Assistant" section with links to quickstart.md, spec.md, plan.md
- [X] T051 [P] Create docker-compose.yml (optional): Orchestrate backend (uvicorn) and frontend (npm start) for local development
- [ ] T052 Validate backend/scripts/ingest.py against all 33 files in /docs: Check for edge cases (< 50 words, malformed Markdown, very long files), ensure chunking preserves semantic boundaries, verify all chunks have complete metadata [MANUAL TESTING REQUIRED]
- [ ] T053 Performance optimization: Measure end-to-end latency for 100 test queries, identify bottlenecks (embedding generation, Qdrant search, flan-t5 inference), optimize if > 10% queries exceed 2 seconds (consider GPU acceleration, reduce top_k, smaller model variant) [MANUAL TESTING REQUIRED]
- [ ] T054 Security hardening: Audit frontend code for API key exposure (check network tab, console logs), ensure backend .env file in .gitignore, validate HTTPS enforcement in production (update docusaurus.config.ts apiUrl) [MANUAL AUDIT REQUIRED]
- [ ] T055 Run quickstart.md validation: Follow setup steps from scratch (new virtualenv, fresh Qdrant collection, empty Neon database), document any missing steps or errors, update quickstart.md with corrections [MANUAL TESTING REQUIRED]

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational completion - No dependencies on other stories
- **User Story 2 (Phase 4)**: Depends on Foundational completion + User Story 1 (requires Qdrant populated with chunks)
- **User Story 3 (Phase 5)**: Depends on Foundational completion + User Story 2 (requires backend API running)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1 - Ingestion)**: Foundational → US1 ✓ (independent, can be tested standalone)
- **User Story 2 (P2 - Backend)**: Foundational → US1 → US2 (requires Qdrant chunks from US1)
- **User Story 3 (P3 - UI)**: Foundational → US2 → US3 (requires backend endpoints from US2)

**⚠️ Sequential Implementation Required**: User stories CANNOT be parallelized due to hard dependencies (US2 needs US1 data, US3 needs US2 API)

### Within Each User Story

- Models before services (e.g., T010-T011 before T012-T014 in US1)
- Services before scripts/endpoints (e.g., T012-T014 before T015-T016 in US1)
- Core implementation before error handling (e.g., T029-T031 before T034 in US2)
- Components before integration (e.g., T036-T040 before T041 in US3)

### Parallel Opportunities

**Phase 1 (Setup)**: T003, T004, T005 (all [P])

**Phase 2 (Foundational)**: T007, T008 (both [P] after T006 completes)

**Phase 3 (US1)**:
- T010, T011 (models, both [P])

**Phase 4 (US2)**:
- T018, T019, T020, T021, T022 (models, all [P])
- T028, T032 (schemas and health endpoint, both [P] after models complete)

**Phase 5 (US3)**:
- T035, T036, T037, T038, T039, T040, T043, T044 (all components, all [P])
- T050, T051 (documentation tasks in Phase 6, both [P])

---

## Parallel Example: User Story 1 (Ingestion)

```bash
# Launch model definitions together:
Task T010: "Create MarkdownFile model in backend/src/models/chunk.py"
Task T011: "Create TextChunk model in backend/src/models/chunk.py"

# After models complete, services run sequentially due to dependencies:
Task T012: "Implement chunking.py chunk_section function" (uses TextChunk from T011)
Task T013: "Implement chunking.py process_markdown_file" (uses chunk_section from T012)
Task T014: "Implement embeddings.py generate_embeddings" (uses TextChunk from T011)
```

---

## Parallel Example: User Story 2 (Backend)

```bash
# Launch all model definitions together:
Task T018: "Create Query model"
Task T019: "Create RetrievedChunk model"
Task T020: "Create RetrievalResult model"
Task T021: "Create SourceCitation model"
Task T022: "Create ChatbotAnswer model"

# After models, schemas and health endpoint can be done in parallel:
Task T028: "Create API schemas"
Task T032: "Implement /health endpoint"
```

---

## Parallel Example: User Story 3 (UI)

```bash
# Launch all component files together:
Task T036: "Create ChatButton.tsx"
Task T037: "Create ChatPanel.tsx"
Task T038: "Create MessageList.tsx"
Task T039: "Create InputBox.tsx"
Task T040: "Create styles.module.css"
Task T043: "Create TextSelectionMenu/index.tsx"
Task T044: "Create TextSelectionMenu/styles.module.css"

# These components can all be developed in parallel since they don't depend on each other initially
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T009) - CRITICAL GATE
3. Complete Phase 3: User Story 1 (T010-T017)
4. **STOP and VALIDATE**:
   - Run `python backend/scripts/verify_qdrant.py` to check collection setup
   - Run `python backend/scripts/ingest.py` to populate Qdrant with all 33 files
   - Query Qdrant directly to verify chunks have embeddings and metadata
   - Test acceptance scenario: "Query Qdrant for a specific chapter topic, all relevant chunks from that chapter are returned"
5. If validation passes → MVP complete (data layer ready for backend)

### Incremental Delivery

1. **Foundation** (Phases 1-2): Setup + Config → Can't test anything yet, but foundation is solid
2. **Increment 1** (Phase 3): Add US1 Ingestion → Test: Qdrant populated with 100 chunks, metadata complete → **First deployable milestone**
3. **Increment 2** (Phase 4): Add US2 Backend → Test: `/ask` and `/ask-local` return grounded answers → **Second deployable milestone** (backend API ready)
4. **Increment 3** (Phase 5): Add US3 UI → Test: Chat widget in Docusaurus, highlight-to-ask works → **Third deployable milestone** (full end-to-end)
5. **Polish** (Phase 6): Performance, security, documentation → **Production-ready**

### Sequential Implementation (Recommended)

Due to hard dependencies between user stories, sequential implementation is REQUIRED:

1. **Week 1**: Phases 1-2 (Setup + Foundational) + Phase 3 (US1 Ingestion)
   - Deliverable: Qdrant populated with all book content
   - Validation: Query Qdrant and get relevant chunks for any topic

2. **Week 2**: Phase 4 (US2 Backend)
   - Deliverable: FastAPI backend with `/ask`, `/ask-local`, `/track` endpoints
   - Validation: `curl` requests return grounded answers with source citations

3. **Week 3**: Phase 5 (US3 UI)
   - Deliverable: Chat widget integrated into Docusaurus
   - Validation: Open browser, click chat button, submit queries, highlight text

4. **Week 4**: Phase 6 (Polish)
   - Deliverable: Performance optimization, security audit, deployment
   - Validation: Load testing (500 req/day), latency < 2s for 90% queries

---

## Notes

- **[P] tasks**: Different files, no dependencies, can run in parallel
- **[Story] label**: Maps task to specific user story (US1, US2, US3) for traceability
- **Sequential constraint**: User stories CANNOT be parallelized due to hard dependencies (US2 needs US1 data, US3 needs US2 API)
- **Independent testability**: Each user story has clear validation criteria (see "Independent Test" in each phase)
- **Checkpoint strategy**: Stop after each phase to validate story works independently before proceeding
- **File path convention**: All paths absolute from repo root `/home/nizam/Physical-AI-Humanoid-Robotics/`
- **No tests**: Spec does not explicitly request unit/integration tests, focus on implementation with manual validation per acceptance scenarios
- **Avoid**: Vague tasks, same file conflicts, skipping error handling, breaking independence of user stories

---

## Task Count Summary

- **Phase 1 (Setup)**: 5 tasks (T001-T005)
- **Phase 2 (Foundational)**: 4 tasks (T006-T009)
- **Phase 3 (US1 Ingestion)**: 8 tasks (T010-T017)
- **Phase 4 (US2 Backend)**: 17 tasks (T018-T034)
- **Phase 5 (US3 UI)**: 15 tasks (T035-T049)
- **Phase 6 (Polish)**: 6 tasks (T050-T055)

**Total**: 55 tasks

**Parallel opportunities**: 23 tasks marked [P] (42% parallelizable within constraints)

**MVP scope**: Phases 1-3 (17 tasks) delivers data layer with Qdrant populated

**Full end-to-end**: Phases 1-5 (49 tasks) delivers complete RAG assistant
