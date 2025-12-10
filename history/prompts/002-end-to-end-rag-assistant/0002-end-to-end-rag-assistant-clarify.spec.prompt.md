---
id: 0002
title: End-to-End RAG Assistant Clarify
stage: spec
date: 2025-12-10
surface: agent
model: claude-sonnet-4.5
feature: 002-end-to-end-rag-assistant
branch: 002-end-to-end-rag-assistant
user: nizam
command: /sp.clarify
labels: ["clarify", "embeddings", "rag", "huggingface", "conversation-history", "reranking", "context-scoping"]
links:
  spec: specs/002-end-to-end-rag-assistant/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-end-to-end-rag-assistant/spec.md
tests:
  - 5 critical clarifications resolved
  - All functional requirements updated
  - Constitutional alignment updated
---

## Prompt

/sp.clarify - Generate embeddings from book content using Hugging Face sentence-transformers (`all-MiniLM-L6-v2`) model

## Response snapshot

Conducted structured clarification session resolving 5 critical ambiguities in the End-to-End RAG Assistant specification through targeted questioning.

**Clarifications Resolved**:

1. **Embedding Model Selection** → Confirmed `sentence-transformers/all-MiniLM-L6-v2` (384 dimensions)
   - Constitutional deviation from OpenAI (1536 dim) accepted with justification: no API costs, no rate limiting, faster inference for < 2s latency target
   - Updated FR-003, VectorEmbedding entity, Assumptions, Technology Constraints, Constitution Alignment

2. **Conversation History Behavior** → Independent queries (no conversation context)
   - Each query retrieves fresh chunks without context from previous Q&A pairs
   - Simpler implementation, no state management complexity, aligns with grounding requirement
   - Added FR-021, updated ChatWidget entity, added to Out of Scope

3. **LLM Service for Answer Generation** → Hugging Face models (local execution)
   - Replaces OpenAI API with local Hugging Face models for answer generation
   - No API costs, no rate limiting, faster response times, full control over inference
   - Updated FR-015, edge cases, assumptions, technology constraints, performance constraints, planning notes

4. **Reranking Strategy** → Simple similarity score sorting
   - Use Qdrant scores directly to rerank top-10 results by descending similarity, select top-3
   - No cross-encoder model needed, faster, meets < 2s latency target
   - Updated FR-012, Out of Scope, planning notes

5. **Context Scope for /ask-local** → Same file only (chapter-level scoping)
   - Filter Qdrant search to chunks where `file_path` matches selected text's source file
   - Clear boundary, simple to implement, ensures answers stay within chapter
   - Updated FR-010, FR-026, User Story 2 & 3 acceptance scenarios

**Specification Updates**:
- Clarifications section created with Session 2025-12-10 containing all 5 Q&A pairs
- 8 functional requirements updated (FR-003, FR-010, FR-012, FR-015, FR-021, FR-026, plus renumbered FR-022 through FR-031)
- 2 entities updated (VectorEmbedding, ChatWidget)
- 4 acceptance scenarios updated across User Stories 2 and 3
- 1 edge case updated (OpenAI rate limits → model inference failures)
- Multiple sections updated: Assumptions, Out of Scope, Technology Constraints, Performance Constraints, Constitution Alignment, Planning Notes

## Outcome

- ✅ Impact: 5 critical ambiguities resolved, specification now complete and implementation-ready
- 🧪 Tests: All clarifications validated against existing acceptance scenarios, no contradictions introduced
- 📁 Files: 1 file updated (spec.md) with 5 clarifications, 8+ functional requirements updated, multiple sections refined
- 🔁 Next prompts: Run `/sp.plan` to design the RAG assistant architecture and implementation approach
- 🧠 Reflection: Successfully resolved all high-impact ambiguities through 5 targeted questions. The embedding model clarification (all-MiniLM-L6-v2) resolves the constitutional conflict documented in the initial spec. Switching from OpenAI API to Hugging Face models for answer generation significantly simplifies deployment (no external API dependencies, no rate limits, full local control). The conversation history clarification (independent queries) reduces complexity while maintaining grounding integrity. Simple similarity score reranking avoids latency overhead while meeting performance targets. Chapter-level context scoping for /ask-local provides clear, intuitive boundaries for selected-text queries. All updates maintain internal consistency - no conflicting requirements introduced.

## Evaluation notes (flywheel)

- Failure modes observed: None - all clarifications accepted on first proposal, no ambiguities or conflicts
- Graders run and results (PASS/FAIL): PASS - All 5 clarifications map cleanly to requirements, no contradictions, constitutional alignment maintained with documented deviations
- Prompt variant (if applicable): N/A (standard clarification workflow)
- Next experiment (smallest change to try): Consider whether the < 2s latency target is achievable with local Hugging Face models for answer generation - may need empirical validation during planning phase to select appropriate model size (e.g., flan-t5-base vs larger variants)
