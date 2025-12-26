"""
Pydantic models for chat-related requests and responses.
"""

from typing import Optional, List
from pydantic import BaseModel, Field
from datetime import datetime
import uuid


class ChatMessage(BaseModel):
    """A single chat message."""
    
    role: str = Field(..., description="Message role: 'user' or 'assistant'")
    content: str = Field(..., description="Message content")
    timestamp: datetime = Field(default_factory=datetime.utcnow)


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    
    message: str = Field(..., description="User's message/question")
    session_id: Optional[str] = Field(
        default=None,
        description="Session ID for conversation continuity"
    )
    selected_text: Optional[str] = Field(
        default=None,
        description="Text that user has selected on the page for context"
    )
    
    class Config:
        json_schema_extra = {
            "example": {
                "message": "What is ROS 2?",
                "session_id": "abc123",
                "selected_text": "ROS 2 nodes communicate using topics"
            }
        }


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    
    message: str = Field(..., description="Assistant's response")
    session_id: str = Field(..., description="Session ID for the conversation")
    sources: List[str] = Field(
        default=[],
        description="Source documents used for the answer"
    )


class SearchRequest(BaseModel):
    """Request model for semantic search endpoint."""
    
    query: str = Field(..., description="Search query")
    top_k: int = Field(default=5, ge=1, le=20, description="Number of results")


class SearchResult(BaseModel):
    """A single search result."""
    
    text: str = Field(..., description="Retrieved text chunk")
    source: str = Field(..., description="Source file path")
    chapter: str = Field(..., description="Chapter name")
    module: str = Field(..., description="Module name")
    score: float = Field(..., description="Similarity score")


class SearchResponse(BaseModel):
    """Response model for search endpoint."""
    
    results: List[SearchResult] = Field(default=[], description="Search results")
    query: str = Field(..., description="Original query")


class ConversationHistory(BaseModel):
    """Conversation history for a session."""
    
    session_id: str
    messages: List[ChatMessage] = Field(default=[])
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
