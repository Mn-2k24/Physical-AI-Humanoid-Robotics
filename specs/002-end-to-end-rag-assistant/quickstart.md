# Quickstart Guide: End-to-End RAG Assistant

**Feature**: 002-end-to-end-rag-assistant | **Date**: 2025-12-10
**Related Docs**: [spec.md](./spec.md) | [plan.md](./plan.md) | [research.md](./research.md) | [data-model.md](./data-model.md)

This guide provides step-by-step instructions to set up, run, and test the RAG assistant from scratch.

---

## Prerequisites

**System Requirements**:
- **Python**: 3.11+ (for backend)
- **Node.js**: 20+ (for Docusaurus frontend)
- **Docker**: 24+ (for deployment)
- **Git**: 2.40+ (for version control)
- **RAM**: 4GB minimum (8GB recommended for local model inference)
- **Disk**: 5GB free space (for models and dependencies)

**External Services**:
- **Qdrant Cloud**: Free tier account with API key and endpoint
- **Neon Postgres**: Free tier account with connection string

---

## Step 1: Clone Repository and Install Dependencies

### 1.1 Clone Repository

```bash
git clone https://github.com/YOUR_USERNAME/Physical-AI-Humanoid-Robotics.git
cd Physical-AI-Humanoid-Robotics
```

### 1.2 Create Python Virtual Environment (Backend)

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python3.11 -m venv venv

# Activate virtual environment
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Upgrade pip
pip install --upgrade pip

# Install dependencies
pip install -r requirements.txt
```

**Expected `requirements.txt` Contents**:

```txt
# FastAPI and server
fastapi==0.104.0
uvicorn[standard]==0.24.0
pydantic==2.5.0
python-dotenv==1.0.0

# Embeddings and LLM
sentence-transformers==2.2.2
transformers==4.35.0
torch==2.1.0

# Vector database
qdrant-client==1.7.0

# Markdown parsing and chunking
mistune==3.0.2
tiktoken==0.5.1

# Database
psycopg2-binary==2.9.9

# HTTP client
httpx==0.25.0

# Testing
pytest==7.4.3
pytest-asyncio==0.21.1
```

### 1.3 Install Node.js Dependencies (Frontend)

```bash
# Navigate back to project root
cd ..

# Install frontend dependencies
npm install
```

---

## Step 2: Configure Environment Variables

### 2.1 Create `.env` File (Backend)

Create `backend/.env` with the following content:

```env
# Qdrant Cloud Configuration
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiIxMjM0NTY3ODkwIiwibmFtZSI6IkpvaG4gRG9lIiwiaWF0IjoxNTE2MjM5MDIyfQ.SflKxwRJSMeKKF2QT4fwpMeJf36POk6yJV_adQssw5c
QDRANT_ENDPOINT=https://0c35c843-5e43-4739-8213-6e6f7fd66b40.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_COLLECTION_NAME=physical-ai-book

# Neon Postgres Configuration
NEON_CONNECTION_STRING=postgresql://user:password@ep-example-123456.us-east-2.aws.neon.tech/neondb?sslmode=require

# Model Configuration
EMBEDDING_MODEL_NAME=sentence-transformers/all-MiniLM-L6-v2
ANSWER_MODEL_NAME=google/flan-t5-base

# API Configuration
API_HOST=0.0.0.0
API_PORT=8000
API_CORS_ORIGINS=http://localhost:3000,http://localhost:3001

# Performance Configuration
MAX_CHUNK_SIZE_TOKENS=1000
MIN_CHUNK_SIZE_TOKENS=300
CHUNK_OVERLAP_TOKENS=50
TOP_K_RETRIEVAL=10
TOP_N_RESULTS=3

# Logging
LOG_LEVEL=INFO
```

**⚠️ Security Note**: Never commit `.env` files to version control. Add `backend/.env` to `.gitignore`.

### 2.2 Update `.gitignore`

Ensure the following lines exist in your `.gitignore`:

```gitignore
# Environment variables
backend/.env
.env

# Python
backend/venv/
backend/__pycache__/
backend/*.pyc

# Node.js
node_modules/
.docusaurus/
build/

# Models (downloaded at runtime)
backend/models/
```

---

## Step 3: Initialize Qdrant Collection

### 3.1 Run Qdrant Verification Script

```bash
cd backend
source venv/bin/activate  # If not already activated

python scripts/verify_qdrant.py
```

**Expected Output**:

```
[INFO] Connecting to Qdrant at https://0c35c843-5e43-4739-8213-6e6f7fd66b40...
[INFO] Connection successful!
[INFO] Checking for collection 'physical-ai-book'...
[WARN] Collection 'physical-ai-book' not found. Creating...
[INFO] Collection 'physical-ai-book' created successfully!
[INFO] Collection config:
  - Vectors: size=384, distance=COSINE
  - Points: 0
[SUCCESS] Qdrant setup complete!
```

**Script Contents** (`backend/scripts/verify_qdrant.py`):

```python
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

load_dotenv()

QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_ENDPOINT = os.getenv("QDRANT_ENDPOINT")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME")

def verify_and_create_collection():
    print(f"[INFO] Connecting to Qdrant at {QDRANT_ENDPOINT}...")
    client = QdrantClient(url=QDRANT_ENDPOINT, api_key=QDRANT_API_KEY)
    print("[INFO] Connection successful!")

    print(f"[INFO] Checking for collection '{QDRANT_COLLECTION_NAME}'...")
    collections = client.get_collections().collections
    collection_names = [c.name for c in collections]

    if QDRANT_COLLECTION_NAME not in collection_names:
        print(f"[WARN] Collection '{QDRANT_COLLECTION_NAME}' not found. Creating...")
        client.create_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=VectorParams(size=384, distance=Distance.COSINE)
        )
        print(f"[INFO] Collection '{QDRANT_COLLECTION_NAME}' created successfully!")
    else:
        print(f"[INFO] Collection '{QDRANT_COLLECTION_NAME}' already exists.")

    # Verify collection config
    collection_info = client.get_collection(QDRANT_COLLECTION_NAME)
    print("[INFO] Collection config:")
    print(f"  - Vectors: size={collection_info.config.params.vectors.size}, distance={collection_info.config.params.vectors.distance}")
    print(f"  - Points: {collection_info.points_count}")
    print("[SUCCESS] Qdrant setup complete!")

if __name__ == "__main__":
    verify_and_create_collection()
```

---

## Step 4: Initialize Neon Postgres Database

### 4.1 Run Neon Initialization Script

```bash
python scripts/init_neon.py
```

**Expected Output**:

```
[INFO] Connecting to Neon Postgres...
[INFO] Connection successful!
[INFO] Creating 'conversations' table...
[INFO] Table 'conversations' created successfully!
[INFO] Creating indexes...
[INFO] Indexes created: idx_conversations_timestamp, idx_conversations_query_type, idx_conversations_sources
[SUCCESS] Neon setup complete!
```

**Script Contents** (`backend/scripts/init_neon.py`):

```python
import os
import psycopg2
from dotenv import load_dotenv

load_dotenv()

NEON_CONNECTION_STRING = os.getenv("NEON_CONNECTION_STRING")

def init_database():
    print("[INFO] Connecting to Neon Postgres...")
    conn = psycopg2.connect(NEON_CONNECTION_STRING)
    cursor = conn.cursor()
    print("[INFO] Connection successful!")

    print("[INFO] Creating 'conversations' table...")
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS conversations (
            conversation_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            query_id UUID NOT NULL,
            query_text TEXT NOT NULL,
            answer_text TEXT NOT NULL,
            sources JSONB NOT NULL,
            query_type VARCHAR(20) NOT NULL CHECK (query_type IN ('full_book', 'local')),
            latency_ms INTEGER NOT NULL,
            timestamp TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
            user_session_id UUID
        );
    """)
    print("[INFO] Table 'conversations' created successfully!")

    print("[INFO] Creating indexes...")
    cursor.execute("CREATE INDEX IF NOT EXISTS idx_conversations_timestamp ON conversations (timestamp DESC);")
    cursor.execute("CREATE INDEX IF NOT EXISTS idx_conversations_query_type ON conversations (query_type);")
    cursor.execute("CREATE INDEX IF NOT EXISTS idx_conversations_sources ON conversations USING GIN (sources);")
    print("[INFO] Indexes created: idx_conversations_timestamp, idx_conversations_query_type, idx_conversations_sources")

    conn.commit()
    cursor.close()
    conn.close()
    print("[SUCCESS] Neon setup complete!")

if __name__ == "__main__":
    init_database()
```

---

## Step 5: Ingest Book Content

### 5.1 Run Ingestion Script

```bash
python scripts/ingest.py
```

**Expected Output**:

```
[INFO] Loading embedding model 'sentence-transformers/all-MiniLM-L6-v2'...
[INFO] Model loaded successfully!
[INFO] Scanning /docs directory...
[INFO] Found 33 Markdown files.
[INFO] Processing docs/01-introduction/overview.md...
  - Extracted 3 sections (headings: Introduction > Overview)
  - Generated 5 chunks (token counts: 512, 678, 423, 891, 345)
  - Generated 5 embeddings (384-dimensional)
  - Uploaded 5 points to Qdrant
[INFO] Processing docs/01-introduction/history.md...
  ...
[INFO] Ingestion complete!
  - Files processed: 33
  - Total chunks: 327
  - Total embeddings: 327
  - Qdrant points uploaded: 327
  - Time elapsed: 4m 32s
[SUCCESS] Book content ingested successfully!
```

**Script Contents** (`backend/scripts/ingest.py`):

```python
import os
import time
from pathlib import Path
from dotenv import load_dotenv
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
import tiktoken
import mistune
from uuid import uuid4

load_dotenv()

# Configuration
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_ENDPOINT = os.getenv("QDRANT_ENDPOINT")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME")
EMBEDDING_MODEL_NAME = os.getenv("EMBEDDING_MODEL_NAME")
DOCS_DIR = Path("../docs")  # Relative to backend/scripts/
MAX_CHUNK_SIZE = int(os.getenv("MAX_CHUNK_SIZE_TOKENS", 1000))
MIN_CHUNK_SIZE = int(os.getenv("MIN_CHUNK_SIZE_TOKENS", 300))
CHUNK_OVERLAP = int(os.getenv("CHUNK_OVERLAP_TOKENS", 50))

# Initialize components
print(f"[INFO] Loading embedding model '{EMBEDDING_MODEL_NAME}'...")
model = SentenceTransformer(EMBEDDING_MODEL_NAME)
print("[INFO] Model loaded successfully!")

qdrant_client = QdrantClient(url=QDRANT_ENDPOINT, api_key=QDRANT_API_KEY)
encoding = tiktoken.get_encoding("cl100k_base")
markdown_parser = mistune.create_markdown()

def chunk_section(section_text, section_heading, file_path, chunk_index_start):
    """Recursive chunking with 50-token overlap."""
    tokens = encoding.encode(section_text)
    chunks = []
    current_index = chunk_index_start

    if len(tokens) <= MAX_CHUNK_SIZE:
        chunk_id = str(uuid4())
        chunks.append({
            "chunk_id": chunk_id,
            "file_path": file_path,
            "section": section_heading,
            "text": section_text,
            "token_count": len(tokens),
            "chunk_index": current_index,
            "overlap_with_prev": False,
            "overlap_with_next": False
        })
    else:
        paragraphs = section_text.split("\n\n")
        current_chunk_tokens = []
        current_chunk_text = []

        for para in paragraphs:
            para_tokens = encoding.encode(para)

            if len(current_chunk_tokens) + len(para_tokens) > MAX_CHUNK_SIZE:
                # Finalize current chunk
                chunk_text = "\n\n".join(current_chunk_text)
                overlap_text = encoding.decode(current_chunk_tokens[-CHUNK_OVERLAP:])

                chunk_id = str(uuid4())
                chunks.append({
                    "chunk_id": chunk_id,
                    "file_path": file_path,
                    "section": section_heading,
                    "text": chunk_text,
                    "token_count": len(current_chunk_tokens),
                    "chunk_index": current_index,
                    "overlap_with_prev": current_index > chunk_index_start,
                    "overlap_with_next": True
                })
                current_index += 1

                # Start new chunk with overlap
                current_chunk_text = [overlap_text, para]
                current_chunk_tokens = encoding.encode(overlap_text) + para_tokens
            else:
                current_chunk_text.append(para)
                current_chunk_tokens.extend(para_tokens)

        # Finalize last chunk
        if current_chunk_text:
            chunk_text = "\n\n".join(current_chunk_text)
            chunk_id = str(uuid4())
            chunks.append({
                "chunk_id": chunk_id,
                "file_path": file_path,
                "section": section_heading,
                "text": chunk_text,
                "token_count": len(current_chunk_tokens),
                "chunk_index": current_index,
                "overlap_with_prev": True,
                "overlap_with_next": False
            })

    return chunks

def process_file(file_path):
    """Process a single Markdown file."""
    print(f"[INFO] Processing {file_path}...")
    with open(file_path, 'r', encoding='utf-8') as f:
        raw_content = f.read()

    # Parse Markdown AST
    ast = markdown_parser(raw_content)

    # Extract sections (simplified - assumes h1 and h2 headings)
    sections = []
    current_section = {"heading": "", "text": ""}
    for block in ast:
        if block['type'] == 'heading' and block['level'] in [1, 2]:
            if current_section["text"]:
                sections.append(current_section)
            current_section = {"heading": block['children'][0]['raw'], "text": ""}
        else:
            current_section["text"] += block.get('raw', '') + "\n\n"
    if current_section["text"]:
        sections.append(current_section)

    print(f"  - Extracted {len(sections)} sections (headings: {', '.join([s['heading'] for s in sections])})")

    # Chunk sections
    all_chunks = []
    chunk_index = 0
    for section in sections:
        chunks = chunk_section(
            section["text"],
            section["heading"],
            str(file_path.relative_to(file_path.parents[2])),
            chunk_index
        )
        all_chunks.extend(chunks)
        chunk_index += len(chunks)

    print(f"  - Generated {len(all_chunks)} chunks (token counts: {', '.join([str(c['token_count']) for c in all_chunks])})")

    # Generate embeddings
    embeddings = model.encode([c["text"] for c in all_chunks])
    print(f"  - Generated {len(embeddings)} embeddings (384-dimensional)")

    # Upload to Qdrant
    points = [
        PointStruct(
            id=chunk["chunk_id"],
            vector=embeddings[i].tolist(),
            payload={
                "file_path": chunk["file_path"],
                "section": chunk["section"],
                "text": chunk["text"],
                "token_count": chunk["token_count"],
                "chunk_index": chunk["chunk_index"],
                "overlap_with_prev": chunk["overlap_with_prev"],
                "overlap_with_next": chunk["overlap_with_next"]
            }
        )
        for i, chunk in enumerate(all_chunks)
    ]
    qdrant_client.upsert(collection_name=QDRANT_COLLECTION_NAME, points=points)
    print(f"  - Uploaded {len(points)} points to Qdrant")

    return len(all_chunks)

def main():
    start_time = time.time()

    print(f"[INFO] Scanning {DOCS_DIR} directory...")
    markdown_files = list(DOCS_DIR.rglob("*.md"))
    print(f"[INFO] Found {len(markdown_files)} Markdown files.")

    total_chunks = 0
    for file_path in markdown_files:
        chunk_count = process_file(file_path)
        total_chunks += chunk_count

    elapsed = time.time() - start_time
    print(f"[INFO] Ingestion complete!")
    print(f"  - Files processed: {len(markdown_files)}")
    print(f"  - Total chunks: {total_chunks}")
    print(f"  - Total embeddings: {total_chunks}")
    print(f"  - Qdrant points uploaded: {total_chunks}")
    print(f"  - Time elapsed: {int(elapsed // 60)}m {int(elapsed % 60)}s")
    print("[SUCCESS] Book content ingested successfully!")

if __name__ == "__main__":
    main()
```

---

## Step 6: Start Backend API Server

### 6.1 Run FastAPI Server

```bash
cd backend
source venv/bin/activate
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
```

**Expected Output**:

```
INFO:     Will watch for changes in these directories: ['/path/to/backend']
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

### 6.2 Test API Health Endpoint

Open a new terminal and run:

```bash
curl http://localhost:8000/health
```

**Expected Response**:

```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "neon_connected": true,
  "model_loaded": true
}
```

---

## Step 7: Start Docusaurus Frontend

### 7.1 Run Development Server

Open a new terminal:

```bash
cd /path/to/Physical-AI-Humanoid-Robotics
npm start
```

**Expected Output**:

```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at http://localhost:3000/
```

### 7.2 Verify Chat Widget

1. Open browser to `http://localhost:3000/`
2. Click floating chat button (bottom-right corner)
3. Chat panel should slide in from the right
4. Type a test query: "What is Physical AI?"
5. Verify answer appears with source citations

---

## Step 8: Test End-to-End Flow

### 8.1 Test `/ask` Endpoint (Full Book Query)

```bash
curl -X POST http://localhost:8000/api/ask \
  -H "Content-Type: application/json" \
  -d '{"query": "What are the main components of a humanoid robot?"}'
```

**Expected Response**:

```json
{
  "query_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
  "answer": "The main components of a humanoid robot include sensors (cameras, IMUs, force-torque sensors), actuators (servo motors, hydraulic systems), a control system (microcontrollers, embedded computers), and a power supply (batteries, power management units). These components work together to enable perception, movement, and decision-making.",
  "sources": [
    {
      "file_path": "docs/02-fundamentals/sensors.md",
      "section": "Fundamentals > Sensors",
      "chunk_index": 0,
      "similarity_score": 0.89
    },
    {
      "file_path": "docs/02-fundamentals/actuators.md",
      "section": "Fundamentals > Actuators",
      "chunk_index": 1,
      "similarity_score": 0.84
    },
    {
      "file_path": "docs/02-fundamentals/control-systems.md",
      "section": "Fundamentals > Control Systems",
      "chunk_index": 0,
      "similarity_score": 0.78
    }
  ],
  "latency_ms": 1234
}
```

### 8.2 Test `/ask-local` Endpoint (Chapter-Scoped Query)

```bash
curl -X POST http://localhost:8000/api/ask-local \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the advantages of LiDAR sensors?",
    "selected_text": "LiDAR sensors provide high-precision depth measurements...",
    "source_file_path": "docs/02-fundamentals/sensors.md"
  }'
```

**Expected Response**:

```json
{
  "query_id": "b2c3d4e5-f6a7-8901-2345-678901bcdef0",
  "answer": "LiDAR sensors offer several advantages: (1) high-precision depth measurements with millimeter accuracy, (2) long-range detection up to 100 meters, (3) robustness to lighting conditions, and (4) 360-degree field of view for comprehensive environment mapping.",
  "sources": [
    {
      "file_path": "docs/02-fundamentals/sensors.md",
      "section": "Fundamentals > Sensors > LiDAR",
      "chunk_index": 3,
      "similarity_score": 0.92
    },
    {
      "file_path": "docs/02-fundamentals/sensors.md",
      "section": "Fundamentals > Sensors > Comparison",
      "chunk_index": 5,
      "similarity_score": 0.87
    },
    {
      "file_path": "docs/02-fundamentals/sensors.md",
      "section": "Fundamentals > Sensors > Use Cases",
      "chunk_index": 7,
      "similarity_score": 0.81
    }
  ],
  "latency_ms": 987
}
```

### 8.3 Test `/track` Endpoint (Analytics)

```bash
curl -X POST http://localhost:8000/api/track \
  -H "Content-Type: application/json" \
  -d '{
    "query_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
    "query_text": "What are the main components of a humanoid robot?",
    "answer_text": "The main components include sensors, actuators...",
    "sources": [...],
    "query_type": "full_book",
    "latency_ms": 1234
  }'
```

**Expected Response**:

```json
{
  "conversation_id": "c3d4e5f6-a7b8-9012-3456-7890cdef0123",
  "status": "logged"
}
```

---

## Step 9: Deployment (Docker + Render.com)

### 9.1 Build Docker Image

Create `backend/Dockerfile`:

```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy source code
COPY src/ ./src/
COPY scripts/ ./scripts/

# Download models at build time (cache for faster startups)
RUN python -c "from sentence_transformers import SentenceTransformer; SentenceTransformer('sentence-transformers/all-MiniLM-L6-v2')"
RUN python -c "from transformers import AutoTokenizer, AutoModelForSeq2SeqLM; AutoTokenizer.from_pretrained('google/flan-t5-base'); AutoModelForSeq2SeqLM.from_pretrained('google/flan-t5-base')"

# Expose port
EXPOSE 8000

# Run server
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Build image:

```bash
cd backend
docker build -t rag-assistant-backend:latest .
```

### 9.2 Deploy to Render.com

1. Create new **Web Service** on Render.com
2. Connect GitHub repository
3. Configure service:
   - **Build Command**: `docker build -t rag-backend backend/`
   - **Start Command**: `docker run -p 8000:8000 rag-backend`
   - **Environment Variables**: Add all `.env` variables
4. Deploy (Render will build and start the container)

### 9.3 Deploy Frontend to Vercel

```bash
npm run build
npx vercel --prod
```

Update `docusaurus.config.ts` with production API URL:

```typescript
const config: Config = {
  customFields: {
    apiUrl: process.env.NODE_ENV === 'production'
      ? 'https://your-render-app.onrender.com'
      : 'http://localhost:8000'
  }
};
```

---

## Troubleshooting

### Issue 1: Qdrant Connection Timeout

**Symptom**: `QdrantException: Connection timeout`

**Solution**:
- Verify `QDRANT_ENDPOINT` includes `https://` prefix
- Check Qdrant API key is valid (no extra whitespace)
- Ensure firewall allows outbound HTTPS (port 443)

### Issue 2: Neon Postgres SSL Error

**Symptom**: `psycopg2.OperationalError: SSL connection required`

**Solution**:
- Ensure connection string includes `?sslmode=require`
- Update `psycopg2-binary` to latest version: `pip install --upgrade psycopg2-binary`

### Issue 3: Model Download Fails

**Symptom**: `OSError: Unable to load model sentence-transformers/all-MiniLM-L6-v2`

**Solution**:
- Check internet connection (models download from Hugging Face Hub)
- Increase timeout: `export TRANSFORMERS_TIMEOUT=300`
- Manually download: `python -c "from sentence_transformers import SentenceTransformer; SentenceTransformer('sentence-transformers/all-MiniLM-L6-v2')"`

### Issue 4: Chat Widget Not Appearing

**Symptom**: No floating button visible on Docusaurus site

**Solution**:
- Verify `src/theme/Root.tsx` wraps app with `<ChatProvider>`
- Check browser console for React errors
- Clear browser cache and reload: `Ctrl+Shift+R` (Windows/Linux) or `Cmd+Shift+R` (Mac)

### Issue 5: Slow Answer Generation (> 2s)

**Symptom**: API responses exceed 2-second latency target

**Solution**:
- Enable GPU acceleration: `pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118`
- Reduce model size: Switch from `flan-t5-base` to `flan-t5-small` in `.env`
- Optimize Qdrant: Increase `top_k` to 5 (fewer network round-trips)

---

## Next Steps

1. **Run `/sp.tasks`** - Generate implementation task breakdown
2. **Implement Backend API** - Follow tasks.md for step-by-step implementation
3. **Implement Frontend UI** - Build ChatWidget components
4. **Write Tests** - Unit tests (pytest) and integration tests (end-to-end)
5. **Deploy to Production** - Follow Step 9 for Docker + Render.com + Vercel deployment

---

**Questions?** See [spec.md](./spec.md) for complete requirements or [plan.md](./plan.md) for architecture details.
