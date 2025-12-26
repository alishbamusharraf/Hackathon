"""
Pydantic models for document-related operations.
"""

from typing import Optional, Dict, Any
from pydantic import BaseModel, Field
from datetime import datetime


class DocumentChunk(BaseModel):
    """A chunk of document content for embedding."""
    
    id: str = Field(..., description="Unique chunk identifier")
    text: str = Field(..., description="Text content of the chunk")
    source: str = Field(..., description="Source file path")
    chapter: str = Field(..., description="Chapter name")
    module: str = Field(..., description="Module name")
    chunk_index: int = Field(..., description="Position in the original document")
    metadata: Dict[str, Any] = Field(default={}, description="Additional metadata")


class DocumentMetadata(BaseModel):
    """Metadata for a document."""
    
    source: str = Field(..., description="Source file path")
    title: str = Field(..., description="Document title")
    chapter: str = Field(default="", description="Chapter name")
    module: str = Field(default="", description="Module name")
    word_count: int = Field(default=0, description="Word count")
    chunk_count: int = Field(default=0, description="Number of chunks")
    created_at: datetime = Field(default_factory=datetime.utcnow)


class IngestRequest(BaseModel):
    """Request model for document ingestion."""
    
    docs_path: str = Field(
        default="../docs",
        description="Path to the docs directory"
    )
    chunk_size: int = Field(
        default=500,
        ge=100,
        le=2000,
        description="Maximum characters per chunk"
    )
    chunk_overlap: int = Field(
        default=50,
        ge=0,
        le=200,
        description="Overlap between chunks"
    )


class IngestResponse(BaseModel):
    """Response model for document ingestion."""
    
    success: bool = Field(..., description="Whether ingestion succeeded")
    documents_processed: int = Field(default=0, description="Number of docs processed")
    chunks_created: int = Field(default=0, description="Number of chunks created")
    errors: list = Field(default=[], description="Any errors encountered")
