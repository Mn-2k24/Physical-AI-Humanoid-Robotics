---
id: 0003
title: RAG Embeddings Pipeline Plan
stage: plan
date: 2025-12-09
surface: agent
model: claude-sonnet-4.5
feature: 001-rag-embeddings-pipeline
branch: 001-rag-embeddings-pipeline
user: nizam
command: /sp.plan
labels: ["plan", "architecture", "embedding", "qdrant", "huggingface"]
links:
  spec: specs/001-rag-embeddings-pipeline/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-embeddings-pipeline/plan.md
  - specs/001-rag-embeddings-pipeline/data-model.md
  - specs/001-rag-embeddings-pipeline/quickstart.md
tests:
  - Constitution Check performed (deviation documented)
  - Phase 0 research completed (6 technical decisions)
  - Phase 1 design completed (data model + quickstart)
---

## Prompt

Objective:
Read all .md files in the docs folder of the Docusaurus book, generate embeddings for each section, and store them in Qdrant for Retrieval-Augmented Generation (RAG) chatbot use. The embeddings will be used to answer questions about the book, including user-selected text.

Target audience:
Developers and readers of the Docusaurus book who want AI-powered querying of book content via chatbot.

Focus:
- Extract clean text from all .md files in docs
- Chunk text into sections suitable for embedding
- Generate embeddings from book content using Hugging Face sentence-transformers (`all-MiniLM-L6-v2`) model
- Store embeddings and metadata (title, section ID, URL slug) in Qdrant Cloud Free Tier
- Ensure retrieval supports both full-book queries and selected-text queries

Success criteria:
- All book content successfully embedded and stored in Qdrant
- Embeddings can be retrieved for RAG chatbot queries efficiently (<3s per query)
- Metadata per chunk correctly stored for reference in answers
- Embeddings updateable when book content changes
- Supports secure use of API key and endpoint provided by user

Constraints:
- Vector collection name: physical-ai-book
- Qdrant endpoint: https://0c35c843-5e43-4739-8213-6e6f7fd66b40.europe-west3-0.gcp.cloud.qdrant.io
- Qdrant API key: [REDACTED for security]
- Minimum 1,000 text chunks for full-book coverage
- Embedding dimension: 1536 (Note: Specification says 1536, but Hugging Face model outputs 384)
- All environment variables stored server-side only
- Only Qdrant Cloud Free Tier used
- Format: Markdown source files as input

Not building:
- Chatbot UI / front-end integration (handled separately)
- Conversation analytics (handled separately)
- Multiple vector DB support (only Qdrant)

## Response snapshot

Created comprehensive implementation plan for RAG Embeddings Pipeline with complete architecture design, technical decisions, and implementation guides.

**Critical Decision**: Identified and resolved embedding dimension conflict:
- Specification requires OpenAI embeddings (dimension 1536)
- User explicitly requested Hugging Face `all-MiniLM-L6-v2` (dimension 384)
- **Resolution**: Proceeded with Hugging Face per user request, documented deviation in Constitution Check with justification (no API costs, no rate limiting, faster processing)

**Planning Artifacts Created**:

1. **plan.md** - Complete implementation plan with:
   - Technical context (Python 3.11+, sentence-transformers, qdrant-client, markdown parsing)
   - Constitution check with deviation documentation
   - Project structure (CLI tool with library modules)
   - Phase 0 research (6 technical decisions)
   - Phase 1 design (data model, CLI interface)

2. **data-model.md** - Entity definitions:
   - **TextChunk**: Semantic text segment (500-800 words with metadata)
   - **ChunkMetadata**: Qdrant payload (title, section_id, url, chapter_number, word_count, embedding_date, source_file_hash)
   - **EmbeddingState**: Progress tracking (file hashes, last checkpoint, session info)
   - **AuditLogEntry**: Structured logging (timestamp, level, message, context)
   - Qdrant schema (dimension 384, cosine distance)

3. **quickstart.md** - Complete setup and usage guide:
   - Installation steps (Python 3.11+, dependencies, virtual environment)
   - Configuration (`.env` file with Qdrant credentials)
   - CLI commands (embed, update, validate, resume, force)
   - Troubleshooting guide
   - Example workflows

**Phase 0 Research - 6 Technical Decisions**:

1. **Embedding Model**: Hugging Face `all-MiniLM-L6-v2`
   - Rationale: User request, free (no API costs), no rate limiting, dimension 384
   - Alternatives rejected: OpenAI (costs + rate limits), other HF models (not requested)

2. **Markdown Parsing**: `python-markdown` with `pymdownx` extensions
   - Rationale: Standard library, GitHub Flavored Markdown support, metadata extraction
   - Alternatives rejected: mistune (less features), regex (error-prone)

3. **Chunking Strategy**: Recursive chunking with semantic boundaries + 100-word overlap
   - Implementation: Split on headings → paragraphs → sentences, preserve context at boundaries
   - Rationale: No mid-sentence splits, overlap prevents context loss (critical for RAG)
   - Alternatives rejected: Fixed chunks (breaks semantics), no overlap (loses context)

4. **Change Detection**: SHA-256 file hashing with `.embedding_state.json`
   - Implementation: Hash files, store in state file, compare on update
   - Rationale: Fast, reliable, simple state management, no external dependencies
   - Alternatives rejected: Timestamps (unreliable), content diff (slower), git tracking (assumes workflow)

5. **Error Handling**: Halt-and-resume with progress tracking
   - Implementation: Save checkpoint after each chunk, `--resume` flag continues
   - Rationale: Prevents data loss, allows manual intervention, aligns with clarifications
   - Alternatives rejected: Skip failed chunks (incomplete data), retry indefinitely (hangs), queue retry (complexity)

6. **Qdrant Client**: Official `qdrant-client` SDK with batch upserts (100 chunks/batch)
   - Rationale: Official SDK, batch efficiency, cosine similarity standard, metadata in payload
   - Alternatives rejected: Raw HTTP (reinventing wheel), single upserts (slow), Euclidean distance (suboptimal)

**Project Structure**:
- CLI tool (`embedding-pipeline/`) with modular architecture
- Modules: cli.py, config.py, markdown_extractor.py, chunker.py, embedder.py, qdrant_client_wrapper.py, change_detector.py, pipeline.py
- Data models: chunk.py, metadata.py
- Tests: unit (chunker, extractor, detector), integration (qdrant ops, full pipeline)

**CLI Interface**:
- `embed`: Full embedding (initial run, with --resume and --force options)
- `update`: Incremental update (changed files only)
- `validate`: Quality check (coverage, semantic search tests)

**State Management**:
- `.embedding_state.json`: Progress checkpoint, file hashes (gitignored)
- `embedding_audit.log`: Structured logs (warnings, errors, info) (gitignored)

## Outcome

- ✅ Impact: Complete architectural plan ready for task breakdown and implementation
- 🧪 Tests: Constitution check performed, deviation documented and justified
- 📁 Files: 3 files created (plan.md, data-model.md, quickstart.md)
- 🔁 Next prompts: Run `/sp.tasks` to generate implementation task breakdown
- 🧠 Reflection: Successfully navigated embedding dimension conflict by prioritizing user's explicit technology choice (Hugging Face) over specification requirement (OpenAI). The deviation is well-justified (cost, rate limits, performance) and documented for transparency. All 6 technical decisions are grounded in best practices with clear rationale. The recursive chunking strategy with 100-word overlap is critical for RAG accuracy - prevents context loss at boundaries. Halt-and-resume error handling aligns perfectly with clarifications from `/sp.clarify`. Ready for implementation.

## Evaluation notes (flywheel)

- Failure modes observed: Embedding dimension mismatch between spec (1536) and user request (384) - resolved by documenting deviation
- Graders run and results (PASS/FAIL): PASS - All constitutional requirements met except embedding dimension (justified deviation)
- Prompt variant (if applicable): N/A (standard planning workflow)
- Next experiment (smallest change to try): Consider benchmarking Hugging Face `all-MiniLM-L6-v2` (dim 384) vs `all-mpnet-base-v2` (dim 768) for retrieval accuracy on technical book content - may inform future optimization
