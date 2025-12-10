# RAG Backend API

FastAPI backend for the Physical AI & Humanoid Robotics book RAG assistant.

## Setup

1. Create a Python virtual environment:
```bash
python3.11 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Configure environment variables:
```bash
cp .env.example .env
# Edit .env with your actual credentials
```

4. Initialize the Neon Postgres database:
```bash
python scripts/init_neon.py
```

5. Ingest book content into Qdrant:
```bash
python scripts/ingest.py
```

6. Start the API server:
```bash
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
```

## API Endpoints

- `POST /api/ask` - Full-book query
- `POST /api/ask-local` - Chapter-scoped query
- `POST /api/track` - Log conversation analytics
- `GET /health` - Health check

## Documentation

For detailed setup and usage instructions, see: [quickstart.md](../specs/002-end-to-end-rag-assistant/quickstart.md)
