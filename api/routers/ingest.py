"""
Ingestion router for document processing endpoints.
"""

import os
from pathlib import Path
from fastapi import APIRouter, HTTPException, BackgroundTasks

from models.document import IngestRequest, IngestResponse
from services.embeddings import get_embeddings_service
from services.qdrant_service import get_qdrant_service


router = APIRouter(prefix="/api/ingest", tags=["ingestion"])


@router.post("/", response_model=IngestResponse)
async def ingest_documents(
    request: IngestRequest,
    background_tasks: BackgroundTasks
):
    """
    Ingest markdown documents from the docs directory.
    
    Parses all markdown files, chunks them, generates embeddings,
    and stores them in Qdrant.
    """
    from scripts.ingest_docs import ingest_all_docs
    
    try:
        result = await ingest_all_docs(
            docs_path=request.docs_path,
            chunk_size=request.chunk_size,
            chunk_overlap=request.chunk_overlap
        )
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/status")
async def get_ingestion_status():
    """Get the current status of the vector collection."""
    try:
        qdrant_service = get_qdrant_service()
        info = await qdrant_service.get_collection_info()
        return info
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/collection")
async def delete_collection():
    """Delete and recreate the vector collection."""
    try:
        qdrant_service = get_qdrant_service()
        await qdrant_service.delete_collection()
        await qdrant_service.ensure_collection_exists()
        return {"success": True, "message": "Collection reset successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
