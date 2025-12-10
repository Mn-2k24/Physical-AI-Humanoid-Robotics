# Data Model: End-to-End RAG Assistant

**Feature**: 002-end-to-end-rag-assistant | **Date**: 2025-12-10
**Related Docs**: [spec.md](./spec.md) | [plan.md](./plan.md) | [research.md](./research.md)

This document defines all entities, their attributes, relationships, and schemas across the four architectural layers of the RAG assistant system.

---

## Entity Overview

The system comprises 8 entities organized across 4 layers:

| Layer | Entity | Purpose | Storage |
|-------|--------|---------|---------|
| **Book Content** | MarkdownFile | Source document metadata | In-memory (ingestion) |
| | TextChunk | Semantically coherent segment | Qdrant payload |
| | VectorEmbedding | 384-dimensional vector | Qdrant vector |
| **RAG Backend** | Query | User input with context | Runtime object |
| | RetrievalResult | Qdrant search results | Runtime object |
| | ChatbotAnswer | Generated response | Runtime object |
| **Analytics** | Conversation | Query/answer log entry | Neon Postgres |
| **UI** | ChatWidget | React component state | Browser state |

---

## Layer 1: Book Content

### Entity 1.1: MarkdownFile

**Purpose**: Represents a single Markdown document from the `/docs` directory during ingestion.

**Attributes**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `file_path` | `str` | ✓ | Relative path from project root | e.g., `docs/01-introduction/overview.md` |
| `raw_content` | `str` | ✓ | Non-empty | Full Markdown text |
| `ast` | `mistune.BlockState` | ✓ | Valid AST | Parsed Markdown tree |
| `heading_path` | `List[str]` | ✓ | Non-empty | Top-level heading (e.g., `["Introduction", "Overview"]`) |
| `file_size_bytes` | `int` | ✓ | > 0 | File size for logging |
| `last_modified` | `datetime` | ✓ | ISO 8601 | File modification timestamp |

**Pydantic Model**:

```python
from pydantic import BaseModel, Field, field_validator
from datetime import datetime
from typing import Any

class MarkdownFile(BaseModel):
    file_path: str = Field(..., min_length=1, description="Relative path from project root")
    raw_content: str = Field(..., min_length=1, description="Full Markdown text")
    ast: Any = Field(..., description="mistune AST (BlockState)")
    heading_path: list[str] = Field(..., min_length=1, description="Top-level heading chain")
    file_size_bytes: int = Field(..., gt=0, description="File size in bytes")
    last_modified: datetime = Field(..., description="File modification timestamp")

    @field_validator('file_path')
    @classmethod
    def validate_file_path(cls, v: str) -> str:
        if not v.startswith('docs/'):
            raise ValueError('file_path must start with "docs/"')
        if not v.endswith('.md'):
            raise ValueError('file_path must end with ".md"')
        return v
```

**State Transitions**: None (immutable during ingestion)

**Relationships**:
- **1-to-many** → `TextChunk` (one file produces multiple chunks)

---

### Entity 1.2: TextChunk

**Purpose**: A semantically coherent segment of Markdown content (300-1,000 tokens) with metadata for retrieval.

**Attributes**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `chunk_id` | `str` | ✓ | UUID v4 | Unique identifier |
| `file_path` | `str` | ✓ | Same as MarkdownFile | Source document path |
| `section` | `str` | ✓ | Non-empty | Heading path (e.g., `"Introduction > Overview"`) |
| `text` | `str` | ✓ | 300-1,000 tokens | Chunk content (with 50-token overlap) |
| `token_count` | `int` | ✓ | 300 ≤ n ≤ 1000 | tiktoken cl100k_base count |
| `chunk_index` | `int` | ✓ | ≥ 0 | Sequential index within file (0-based) |
| `overlap_with_prev` | `bool` | ✓ | — | `True` if chunk includes overlap from previous |
| `overlap_with_next` | `bool` | ✓ | — | `True` if next chunk will include overlap from this |

**Pydantic Model**:

```python
from pydantic import BaseModel, Field, field_validator
from uuid import uuid4

class TextChunk(BaseModel):
    chunk_id: str = Field(default_factory=lambda: str(uuid4()), description="Unique identifier")
    file_path: str = Field(..., min_length=1, description="Source document path")
    section: str = Field(..., min_length=1, description="Heading path (e.g., 'Intro > Overview')")
    text: str = Field(..., min_length=1, description="Chunk content with overlap")
    token_count: int = Field(..., ge=300, le=1000, description="tiktoken cl100k_base count")
    chunk_index: int = Field(..., ge=0, description="Sequential index within file")
    overlap_with_prev: bool = Field(default=False, description="Has overlap from previous chunk")
    overlap_with_next: bool = Field(default=False, description="Next chunk will have overlap from this")

    @field_validator('text')
    @classmethod
    def validate_text_not_empty(cls, v: str) -> str:
        if not v.strip():
            raise ValueError('text must be non-empty after stripping')
        return v
```

**State Transitions**: None (immutable after creation)

**Relationships**:
- **Many-to-one** → `MarkdownFile` (multiple chunks belong to one file)
- **1-to-1** → `VectorEmbedding` (each chunk has exactly one embedding)

---

### Entity 1.3: VectorEmbedding

**Purpose**: A 384-dimensional embedding vector generated by `sentence-transformers/all-MiniLM-L6-v2` for similarity search.

**Attributes**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `vector` | `List[float]` | ✓ | len == 384 | Embedding vector |
| `chunk_id` | `str` | ✓ | UUID v4 | Reference to TextChunk |

**Qdrant Schema**:

```python
from qdrant_client.models import Distance, VectorParams, PointStruct

# Collection configuration
collection_config = VectorParams(
    size=384,
    distance=Distance.COSINE
)

# Point structure (one per TextChunk)
point = PointStruct(
    id=chunk_id,  # UUID as string
    vector=vector,  # 384-dimensional list[float]
    payload={
        "file_path": "docs/01-introduction/overview.md",
        "section": "Introduction > Overview",
        "text": "Full chunk text with overlap...",
        "token_count": 512,
        "chunk_index": 0,
        "overlap_with_prev": False,
        "overlap_with_next": True
    }
)
```

**State Transitions**: None (immutable after upload)

**Relationships**:
- **1-to-1** → `TextChunk` (each vector corresponds to one chunk)

---

## Layer 2: RAG Backend

### Entity 2.1: Query

**Purpose**: Represents a user's question with context for routing and retrieval.

**Attributes**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `query_id` | `str` | ✓ | UUID v4 | Unique identifier |
| `query_text` | `str` | ✓ | 1-500 chars | User's question |
| `query_type` | `str` | ✓ | `full_book` or `local` | Context scope |
| `selected_text` | `str` | ✗ | Only if query_type == `local` | Highlighted text (for `/ask-local`) |
| `source_file_path` | `str` | ✗ | Only if query_type == `local` | File containing selected text |
| `timestamp` | `datetime` | ✓ | ISO 8601 | Query creation time |

**Pydantic Model**:

```python
from pydantic import BaseModel, Field, field_validator, model_validator
from datetime import datetime
from uuid import uuid4

class Query(BaseModel):
    query_id: str = Field(default_factory=lambda: str(uuid4()), description="Unique identifier")
    query_text: str = Field(..., min_length=1, max_length=500, description="User's question")
    query_type: str = Field(..., pattern="^(full_book|local)$", description="Context scope")
    selected_text: str | None = Field(default=None, description="Highlighted text (for /ask-local)")
    source_file_path: str | None = Field(default=None, description="File containing selected text")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Query creation time")

    @model_validator(mode='after')
    def validate_local_query_requirements(self):
        if self.query_type == 'local':
            if not self.selected_text or not self.source_file_path:
                raise ValueError('selected_text and source_file_path required for local queries')
        return self
```

**State Transitions**: None (immutable after creation)

**Relationships**:
- **1-to-1** → `RetrievalResult` (each query produces one retrieval result)

---

### Entity 2.2: RetrievalResult

**Purpose**: Represents the top-3 chunks retrieved from Qdrant after similarity search and reranking.

**Attributes**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `query_id` | `str` | ✓ | UUID v4 | Reference to Query |
| `chunks` | `List[RetrievedChunk]` | ✓ | len == 3 | Top-3 chunks (sorted by similarity descending) |
| `retrieval_latency_ms` | `int` | ✓ | > 0 | Time taken for Qdrant search + reranking |

**Nested Entity: RetrievedChunk**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `chunk_id` | `str` | ✓ | UUID v4 | Reference to TextChunk |
| `file_path` | `str` | ✓ | Non-empty | Source document path |
| `section` | `str` | ✓ | Non-empty | Heading path |
| `text` | `str` | ✓ | Non-empty | Chunk content |
| `similarity_score` | `float` | ✓ | 0.0 ≤ n ≤ 1.0 | Cosine similarity from Qdrant |
| `rank` | `int` | ✓ | 1, 2, or 3 | Position in top-3 results |

**Pydantic Models**:

```python
from pydantic import BaseModel, Field, field_validator
from typing import List

class RetrievedChunk(BaseModel):
    chunk_id: str = Field(..., description="Reference to TextChunk")
    file_path: str = Field(..., min_length=1, description="Source document path")
    section: str = Field(..., min_length=1, description="Heading path")
    text: str = Field(..., min_length=1, description="Chunk content")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Cosine similarity")
    rank: int = Field(..., ge=1, le=3, description="Position in top-3 results")

class RetrievalResult(BaseModel):
    query_id: str = Field(..., description="Reference to Query")
    chunks: List[RetrievedChunk] = Field(..., min_length=3, max_length=3, description="Top-3 chunks")
    retrieval_latency_ms: int = Field(..., gt=0, description="Qdrant search + reranking time")

    @field_validator('chunks')
    @classmethod
    def validate_chunks_sorted(cls, v: List[RetrievedChunk]) -> List[RetrievedChunk]:
        # Ensure chunks are sorted by similarity descending
        if not all(v[i].similarity_score >= v[i+1].similarity_score for i in range(len(v)-1)):
            raise ValueError('chunks must be sorted by similarity_score descending')
        # Ensure ranks are 1, 2, 3
        if [c.rank for c in v] != [1, 2, 3]:
            raise ValueError('ranks must be 1, 2, 3')
        return v
```

**State Transitions**: None (immutable after creation)

**Relationships**:
- **1-to-1** → `Query` (one retrieval result per query)
- **1-to-1** → `ChatbotAnswer` (retrieval result feeds answer generation)

---

### Entity 2.3: ChatbotAnswer

**Purpose**: Represents the final answer generated by `google/flan-t5-base` with source citations.

**Attributes**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `query_id` | `str` | ✓ | UUID v4 | Reference to Query |
| `answer_text` | `str` | ✓ | 1-1000 chars | Generated answer |
| `sources` | `List[SourceCitation]` | ✓ | len == 3 | Citations for top-3 chunks |
| `generation_latency_ms` | `int` | ✓ | > 0 | Time taken for flan-t5-base inference |
| `total_latency_ms` | `int` | ✓ | > 0 | retrieval_latency_ms + generation_latency_ms |
| `timestamp` | `datetime` | ✓ | ISO 8601 | Answer creation time |

**Nested Entity: SourceCitation**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `file_path` | `str` | ✓ | Non-empty | Source document path |
| `section` | `str` | ✓ | Non-empty | Heading path |
| `chunk_index` | `int` | ✓ | ≥ 0 | Sequential index within file |
| `similarity_score` | `float` | ✓ | 0.0 ≤ n ≤ 1.0 | Cosine similarity |

**Pydantic Models**:

```python
from pydantic import BaseModel, Field, field_validator
from datetime import datetime
from typing import List

class SourceCitation(BaseModel):
    file_path: str = Field(..., min_length=1, description="Source document path")
    section: str = Field(..., min_length=1, description="Heading path")
    chunk_index: int = Field(..., ge=0, description="Sequential index within file")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Cosine similarity")

class ChatbotAnswer(BaseModel):
    query_id: str = Field(..., description="Reference to Query")
    answer_text: str = Field(..., min_length=1, max_length=1000, description="Generated answer")
    sources: List[SourceCitation] = Field(..., min_length=3, max_length=3, description="Top-3 citations")
    generation_latency_ms: int = Field(..., gt=0, description="flan-t5-base inference time")
    total_latency_ms: int = Field(..., gt=0, description="retrieval + generation time")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Answer creation time")

    @field_validator('total_latency_ms')
    @classmethod
    def validate_total_latency(cls, v: int, info) -> int:
        # Ensure total_latency_ms includes generation_latency_ms
        gen_latency = info.data.get('generation_latency_ms', 0)
        if v < gen_latency:
            raise ValueError('total_latency_ms must be >= generation_latency_ms')
        return v
```

**State Transitions**: None (immutable after creation)

**Relationships**:
- **1-to-1** → `Query` (one answer per query)
- **1-to-1** → `RetrievalResult` (answer based on retrieval results)
- **1-to-1** → `Conversation` (answer logged to analytics)

---

## Layer 3: Analytics

### Entity 3.1: Conversation

**Purpose**: Persistent log entry in Neon Postgres for analytics and debugging.

**Attributes**:

| Attribute | Type | Required | Validation | Database Column | Description |
|-----------|------|----------|------------|----------------|-------------|
| `conversation_id` | `str` | ✓ | UUID v4 | `UUID PRIMARY KEY` | Unique identifier |
| `query_id` | `str` | ✓ | UUID v4 | `UUID NOT NULL` | Reference to Query |
| `query_text` | `str` | ✓ | 1-500 chars | `TEXT NOT NULL` | User's question |
| `answer_text` | `str` | ✓ | 1-1000 chars | `TEXT NOT NULL` | Generated answer |
| `sources` | `List[SourceCitation]` | ✓ | len == 3 | `JSONB NOT NULL` | Top-3 citations |
| `query_type` | `str` | ✓ | `full_book` or `local` | `VARCHAR(20) NOT NULL` | Context scope |
| `latency_ms` | `int` | ✓ | > 0 | `INTEGER NOT NULL` | Total latency |
| `timestamp` | `datetime` | ✓ | ISO 8601 | `TIMESTAMP WITH TIME ZONE` | Query creation time |
| `user_session_id` | `str` | ✗ | UUID v4 | `UUID` | Browser session ID (optional) |

**PostgreSQL Schema**:

```sql
CREATE TABLE conversations (
    conversation_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    query_id UUID NOT NULL,
    query_text TEXT NOT NULL,
    answer_text TEXT NOT NULL,
    sources JSONB NOT NULL,  -- Array of {file_path, section, chunk_index, similarity_score}
    query_type VARCHAR(20) NOT NULL CHECK (query_type IN ('full_book', 'local')),
    latency_ms INTEGER NOT NULL,
    timestamp TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    user_session_id UUID
);

CREATE INDEX idx_conversations_timestamp ON conversations (timestamp DESC);
CREATE INDEX idx_conversations_query_type ON conversations (query_type);
CREATE INDEX idx_conversations_sources ON conversations USING GIN (sources);
```

**Example JSONB `sources` Value**:

```json
[
  {
    "file_path": "docs/01-introduction/overview.md",
    "section": "Introduction > Overview",
    "chunk_index": 0,
    "similarity_score": 0.89
  },
  {
    "file_path": "docs/02-fundamentals/sensors.md",
    "section": "Fundamentals > Sensors",
    "chunk_index": 2,
    "similarity_score": 0.84
  },
  {
    "file_path": "docs/01-introduction/history.md",
    "section": "Introduction > History",
    "chunk_index": 1,
    "similarity_score": 0.78
  }
]
```

**Pydantic Model** (for Python ORM):

```python
from pydantic import BaseModel, Field, field_validator
from datetime import datetime
from uuid import uuid4
from typing import List, Optional

class Conversation(BaseModel):
    conversation_id: str = Field(default_factory=lambda: str(uuid4()), description="Unique identifier")
    query_id: str = Field(..., description="Reference to Query")
    query_text: str = Field(..., min_length=1, max_length=500, description="User's question")
    answer_text: str = Field(..., min_length=1, max_length=1000, description="Generated answer")
    sources: List[SourceCitation] = Field(..., min_length=3, max_length=3, description="Top-3 citations")
    query_type: str = Field(..., pattern="^(full_book|local)$", description="Context scope")
    latency_ms: int = Field(..., gt=0, description="Total latency")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Query creation time")
    user_session_id: Optional[str] = Field(default=None, description="Browser session ID")

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }
```

**State Transitions**: None (append-only log)

**Relationships**:
- **1-to-1** → `Query` (one log entry per query)
- **1-to-1** → `ChatbotAnswer` (answer fields copied to log)

---

## Layer 4: UI

### Entity 4.1: ChatWidget

**Purpose**: React component state for the chat interface in Docusaurus.

**Attributes**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `isOpen` | `boolean` | ✓ | — | Panel visibility (slide-over) |
| `messages` | `List[ChatMessage]` | ✓ | — | Display-only conversation history |
| `isLoading` | `boolean` | ✓ | — | True during API call |
| `error` | `str` | ✗ | — | Error message (if API fails) |

**Nested Entity: ChatMessage**:

| Attribute | Type | Required | Validation | Description |
|-----------|------|----------|------------|-------------|
| `role` | `str` | ✓ | `user` or `assistant` | Message sender |
| `content` | `str` | ✓ | Non-empty | Message text |
| `sources` | `List[SourceCitation]` | ✗ | Only if role == `assistant` | Top-3 citations (for assistant messages) |
| `timestamp` | `datetime` | ✓ | ISO 8601 | Message creation time |

**TypeScript Interface**:

```typescript
interface SourceCitation {
  file_path: string;
  section: string;
  chunk_index: number;
  similarity_score: number;
}

interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  sources?: SourceCitation[];  // Only for assistant messages
  timestamp: string;  // ISO 8601
}

interface ChatWidgetState {
  isOpen: boolean;
  messages: ChatMessage[];
  isLoading: boolean;
  error?: string;
}
```

**React State Hook**:

```typescript
import { useState } from 'react';

const useChatWidget = (): [ChatWidgetState, ChatWidgetActions] => {
  const [state, setState] = useState<ChatWidgetState>({
    isOpen: false,
    messages: [],
    isLoading: false,
    error: undefined
  });

  const actions = {
    togglePanel: () => setState(s => ({ ...s, isOpen: !s.isOpen })),
    sendMessage: async (query: string, queryType: 'full_book' | 'local', selectedText?: string, sourceFilePath?: string) => {
      setState(s => ({ ...s, isLoading: true, error: undefined }));

      try {
        const endpoint = queryType === 'full_book' ? '/api/ask' : '/api/ask-local';
        const body = queryType === 'full_book'
          ? { query }
          : { query, selected_text: selectedText, source_file_path: sourceFilePath };

        const response = await fetch(endpoint, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body)
        });

        if (!response.ok) throw new Error(`API error: ${response.status}`);

        const data = await response.json();

        setState(s => ({
          ...s,
          messages: [
            ...s.messages,
            { role: 'user', content: query, timestamp: new Date().toISOString() },
            {
              role: 'assistant',
              content: data.answer,
              sources: data.sources,
              timestamp: new Date().toISOString()
            }
          ],
          isLoading: false
        }));
      } catch (err) {
        setState(s => ({
          ...s,
          isLoading: false,
          error: err instanceof Error ? err.message : 'Unknown error'
        }));
      }
    },
    clearError: () => setState(s => ({ ...s, error: undefined }))
  };

  return [state, actions];
};
```

**State Transitions**:

1. **Initial** → `{ isOpen: false, messages: [], isLoading: false }`
2. **User Opens Panel** → `{ isOpen: true, messages: [], isLoading: false }`
3. **User Sends Query** → `{ isOpen: true, messages: [...], isLoading: true }`
4. **API Success** → `{ isOpen: true, messages: [...user_msg, assistant_msg], isLoading: false }`
5. **API Failure** → `{ isOpen: true, messages: [...user_msg], isLoading: false, error: "..." }`

**Relationships**:
- **Many-to-many** → `ChatbotAnswer` (messages array displays answers from multiple queries, but no persistence across sessions)

**Important Notes**:
- **No Conversation Context**: Each query is independent; `messages` array is display-only and not sent to backend.
- **Session Scope**: State resets on page refresh (browser-only storage, no localStorage).
- **Error Handling**: Failed queries show error banner; user can retry.

---

## Entity Relationship Diagram

```
┌──────────────┐
│ MarkdownFile │
└──────┬───────┘
       │ 1:N
       ↓
┌──────────────┐      1:1      ┌──────────────────┐
│  TextChunk   │ ←────────────→ │ VectorEmbedding  │
└──────────────┘                └──────────────────┘
       ↑                                │
       │                                │ (stored in Qdrant)
       │                                ↓
       │                        ┌──────────────────┐
       │                        │  Qdrant Point    │
       │                        │  (id, vector,    │
       │                        │   payload)       │
       │                        └──────────────────┘
       │
       │ (referenced by chunk_id)
       │
┌──────┴───────┐      1:1      ┌──────────────────┐
│    Query     │ ←────────────→ │ RetrievalResult  │
└──────┬───────┘                └──────────────────┘
       │ 1:1                            │ 1:1
       ↓                                ↓
┌──────────────────┐            ┌──────────────────┐
│ ChatbotAnswer    │ ←──────────│  (answer based   │
│                  │            │   on retrieval)  │
└──────┬───────────┘            └──────────────────┘
       │ 1:1
       ↓
┌──────────────────┐
│  Conversation    │ (Neon Postgres - append-only log)
└──────────────────┘

┌──────────────────┐
│   ChatWidget     │ (Browser state - no persistence)
│   (UI Layer)     │
└──────────────────┘
       │ (displays)
       ↓
┌──────────────────┐
│   ChatMessage    │ (Display-only; no backend relationship)
└──────────────────┘
```

---

## Implementation Notes

### Pydantic Model Organization

**File Structure**:

```
backend/src/models/
├── __init__.py
├── chunk.py          # MarkdownFile, TextChunk
├── embedding.py      # VectorEmbedding
├── query.py          # Query
├── retrieval.py      # RetrievalResult, RetrievedChunk
├── answer.py         # ChatbotAnswer, SourceCitation
└── conversation.py   # Conversation
```

### Validation Rules Summary

| Entity | Critical Validations |
|--------|---------------------|
| **MarkdownFile** | `file_path` starts with `docs/` and ends with `.md` |
| **TextChunk** | `token_count` between 300-1,000; `text` non-empty after stripping |
| **VectorEmbedding** | `vector` length exactly 384 |
| **Query** | If `query_type == 'local'`, requires `selected_text` and `source_file_path` |
| **RetrievalResult** | Exactly 3 chunks, sorted by `similarity_score` descending, ranks 1-2-3 |
| **ChatbotAnswer** | `total_latency_ms >= generation_latency_ms`; exactly 3 sources |
| **Conversation** | `query_type` in `{'full_book', 'local'}`; `sources` JSONB with 3 items |

### Performance Considerations

- **Qdrant Payload Size**: Each point payload ~1KB (chunk text + metadata). 33 files × ~10 chunks/file = ~330KB total payload.
- **Neon Postgres Row Size**: Each conversation row ~2KB (query + answer + sources JSONB). 500 requests/day × 30 days = 30MB/month.
- **React State Size**: ChatWidget messages array grows unbounded during session. Consider limiting to last 20 messages if memory becomes an issue.

---

**Next Steps**: See [quickstart.md](./quickstart.md) for setup and usage guide.
