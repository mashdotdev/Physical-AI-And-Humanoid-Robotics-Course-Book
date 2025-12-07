# RAG Chatbot Backend

Python FastAPI backend for the RAG-powered documentation chatbot.

## Architecture

- **FastAPI** - Async web framework
- **Cohere** - Text embeddings (`embed-english-v3.0`, 1024 dimensions)
- **Qdrant** - Vector database for semantic search
- **OpenAI** - Chat completion (`gpt-4o-mini`)

## Quick Start

### 1. Install Dependencies

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Configure Environment

Copy the example environment file and add your API keys:

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
OPENAI_API_KEY=your_openai_api_key
CORS_ORIGINS=http://localhost:3000,http://localhost:3001
```

### 3. Create Qdrant Collection

```bash
python scripts/create_collection.py
```

### 4. Ingest Documents

Index your book content into the vector database:

```bash
python scripts/ingest.py
```

This will:
- Scan markdown files from `../book/docs/`
- Chunk content (~500 tokens with 50 token overlap)
- Generate embeddings via Cohere
- Store vectors in Qdrant

### 5. Run the Server

```bash
uvicorn app.main:app --reload --port 8000
```

The API will be available at `http://localhost:8000`.

## API Endpoints

### Health Check

```
GET /health
```

Returns server status:

```json
{
  "status": "healthy",
  "version": "1.0.0"
}
```

### Chat

```
POST /api/chat
Content-Type: application/json

{
  "message": "What is ROS 2?",
  "conversation_id": "optional-uuid"
}
```

Response:

```json
{
  "response": "ROS 2 (Robot Operating System 2) is...",
  "sources": [
    {
      "chapter": "Chapter 1",
      "section": "The Robotic Nervous System",
      "relevance": 0.89
    }
  ],
  "conversation_id": "uuid-v4"
}
```

## Docker Deployment

### Build Image

```bash
docker build -t rag-chatbot-backend .
```

### Run Container

```bash
docker run -d \
  --name rag-chatbot \
  -p 8000:8000 \
  --env-file .env \
  rag-chatbot-backend
```

### Docker Compose (Optional)

Create `docker-compose.yml`:

```yaml
version: '3.8'
services:
  backend:
    build: .
    ports:
      - "8000:8000"
    env_file:
      - .env
    restart: unless-stopped
```

Run with:

```bash
docker-compose up -d
```

## Project Structure

```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py          # FastAPI app entry point
│   ├── config.py        # Environment configuration
│   ├── api/
│   │   ├── __init__.py
│   │   └── chat.py      # Chat endpoint
│   ├── models/
│   │   ├── __init__.py
│   │   └── schemas.py   # Pydantic models
│   └── rag/
│       ├── __init__.py
│       ├── chain.py     # RAG pipeline orchestration
│       ├── embeddings.py # Cohere embeddings
│       └── vectorstore.py # Qdrant operations
├── scripts/
│   ├── create_collection.py
│   └── ingest.py
├── tests/
├── .env.example
├── Dockerfile
├── README.md
└── requirements.txt
```

## Configuration

| Variable | Description | Required |
|----------|-------------|----------|
| `COHERE_API_KEY` | Cohere API key for embeddings | Yes |
| `QDRANT_URL` | Qdrant cluster URL | Yes |
| `QDRANT_API_KEY` | Qdrant API key | Yes |
| `OPENAI_API_KEY` | OpenAI API key for chat | Yes |
| `CORS_ORIGINS` | Allowed CORS origins (comma-separated) | No |
| `COLLECTION_NAME` | Qdrant collection name | No (default: `book_docs`) |
| `EMBEDDING_MODEL` | Cohere model name | No (default: `embed-english-v3.0`) |
| `CHAT_MODEL` | OpenAI model name | No (default: `gpt-4o-mini`) |

## Development

### Run Tests

```bash
pytest tests/ -v
```

### Code Formatting

```bash
pip install black isort
black app/ scripts/
isort app/ scripts/
```

## Troubleshooting

### "No vectors found" error

Make sure you've run the ingestion script:

```bash
python scripts/ingest.py
```

### CORS errors

Add your frontend URL to `CORS_ORIGINS` in `.env`:

```
CORS_ORIGINS=http://localhost:3000,https://your-domain.com
```

### Rate limiting

The free tiers have rate limits:
- Cohere: 100 calls/minute
- OpenAI: Varies by plan

Consider adding caching for repeated queries.
