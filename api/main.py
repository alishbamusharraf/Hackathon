"""
FastAPI main application entry point.
RAG Chatbot API for Physical AI & Humanoid Robotics book.
"""

from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

from config import settings
from routers import chat, ingest
from services.postgres_service import get_postgres_service
from services.qdrant_service import get_qdrant_service


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for startup and shutdown events.
    """
    # Startup
    print("Starting RAG Chatbot API...")
    
    # Initialize database tables
    try:
        postgres_service = get_postgres_service()
        await postgres_service.init_tables()
        print("Database initialized")
    except Exception as e:
        print(f"Database initialization failed: {e}")
        print("   (Will retry on first request)")
    
    # Ensure Qdrant collection exists
    try:
        qdrant_service = get_qdrant_service()
        await qdrant_service.ensure_collection_exists()
        print("Qdrant collection ready")
    except Exception as e:
        print(f"Qdrant initialization failed: {e}")
        print("   (Check QDRANT_URL and QDRANT_API_KEY)")
    
    print("API ready!")
    
    yield
    
    # Shutdown
    print("Shutting down...")
    try:
        postgres_service = get_postgres_service()
        await postgres_service.disconnect()
    except:
        pass


# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for Physical AI & Humanoid Robotics book",
    version="1.0.0",
    lifespan=lifespan
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat.router)
app.include_router(ingest.router)


@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "name": "RAG Chatbot API",
        "version": "1.0.0",
        "description": "Chat with the Physical AI & Humanoid Robotics book",
        "endpoints": {
            "chat": "/api/chat",
            "chat_stream": "/api/chat/stream",
            "search": "/api/search",
            "ingest": "/api/ingest/",
            "ingest_status": "/api/ingest/status"
        }
    }


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=False
    )
