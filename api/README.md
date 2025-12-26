# RAG Chatbot API

A FastAPI-based Retrieval-Augmented Generation (RAG) chatbot for the **Physical AI & Humanoid Robotics** book.

## Features

- 🔍 **Semantic Search** - Uses all-MiniLM-L6-v2 embeddings for accurate retrieval
- 🤖 **Soft-coded Agent** - Dynamically understands queries without hardcoded patterns
- 💬 **Conversational** - Maintains context across multiple turns
- 📝 **Text Selection** - Answer questions about user-selected text
- 🔄 **Streaming** - Real-time response streaming via SSE
- 💾 **Persistent History** - Conversation history stored in Postgres

## Tech Stack

- **FastAPI** - Python web framework
- **Qdrant Cloud** - Vector database for embeddings
- **Neon Postgres** - Serverless PostgreSQL for conversation history
- **Google Gemini 1.5 Flash** - LLM for response generation
- **sentence-transformers** - all-MiniLM-L6-v2 embeddings

## Setup

### 1. Install Dependencies

```bash
cd api
pip install -r requirements.txt
```

### 2. Configure Environment

Copy the template and fill in your credentials:

```bash
cp env.template .env
```

Edit `.env` with your API keys:

```env
# Google Gemini (https://ai.google.dev)
GEMINI_API_KEY=your-gemini-key
GEMINI_MODEL=gemini-1.5-flash

# Qdrant Cloud (https://cloud.qdrant.io)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key

# Neon Postgres (https://neon.tech)
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require

# CORS (add your frontend URL)
CORS_ORIGINS=http://localhost:3000
```

### 3. Ingest Book Content

Before using the chatbot, ingest the book's markdown files:

```bash
cd api
python -m scripts.ingest_docs
```

This will:
- Parse all markdown files in `../docs`
- Chunk text into ~500 character segments
- Generate embeddings using all-MiniLM-L6-v2
- Store vectors in Qdrant

### 4. Start the Server

```bash
cd api
uvicorn main:app --reload --port 8000
```

The API will be available at `http://localhost:8000`

## API Endpoints

### Chat

**POST** `/api/chat`

Send a message and get a response.

```json
{
  "message": "What is ROS 2?",
  "session_id": "optional-session-id",
  "selected_text": "optional selected text from the page"
}
```

Response:
```json
{
  "message": "ROS 2 (Robot Operating System 2) is...",
  "session_id": "abc123",
  "sources": ["module-1/Chapter 1 Core Concepts"]
}
```

### Streaming Chat

**POST** `/api/chat/stream`

Same request body as `/api/chat`, returns Server-Sent Events (SSE).

### Search

**POST** `/api/search`

Semantic search in the book content.

```json
{
  "query": "how to create a ROS 2 node",
  "top_k": 5
}
```

### Ingest

**POST** `/api/ingest/`

Trigger document ingestion (for re-indexing).

**GET** `/api/ingest/status`

Get vector collection statistics.

## Development

### Project Structure

```
api/
├── main.py                 # FastAPI app entry
├── config.py               # Environment configuration
├── requirements.txt        # Python dependencies
├── models/
│   ├── chat.py             # Chat request/response models
│   └── document.py         # Document models
├── services/
│   ├── embeddings.py       # all-MiniLM-L6-v2 embeddings
│   ├── qdrant_service.py   # Vector database operations
│   ├── postgres_service.py # Conversation history
│   └── agent_service.py    # Gemini Agent (soft-coded)
├── routers/
│   ├── chat.py             # Chat endpoints
│   └── ingest.py           # Ingestion endpoints
└── scripts/
    └── ingest_docs.py      # CLI for ingestion
```

### Agent Design

The chatbot uses a **soft-coded agent** approach:

- **No hardcoded patterns** - The agent dynamically interprets user intent
- **Context-grounded** - Only answers using retrieved book content
- **Human-like tone** - Responds conversationally, not robotically
- **Multi-turn aware** - Maintains conversation context

## Deployment

### Docker

```dockerfile
FROM python:3.11-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY . .
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Environment Variables

Remember to set all environment variables in your deployment platform.

## License

MIT
