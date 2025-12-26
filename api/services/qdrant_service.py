"""
Qdrant Cloud service for vector storage and semantic search.
Handles document embedding storage and retrieval.
"""

from typing import List, Dict, Any, Optional
from qdrant_client import AsyncQdrantClient, QdrantClient

from qdrant_client.http import models
from qdrant_client.http.models import (
    Distance,
    VectorParams,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue,
)
import uuid

from config import settings
from models.document import DocumentChunk
from models.chat import SearchResult


class QdrantService:
    """Service for Qdrant vector database operations."""
    
    def __init__(self):
        """Initialize Qdrant client with cloud credentials."""
        self.client = AsyncQdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )

        self.collection_name = settings.qdrant_collection_name
        self.vector_size = settings.embedding_dimension
    
    async def ensure_collection_exists(self) -> bool:
        """
        Ensure the collection exists, create if not.
        
        Returns:
            True if collection exists or was created successfully
        """
        try:
            collections = await self.client.get_collections()
            collection_names = [c.name for c in collections.collections]
            
            if self.collection_name not in collection_names:
                await self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.vector_size,
                        distance=Distance.COSINE
                    )
                )

                print(f"Created collection: {self.collection_name}")
            else:
                print(f"Collection exists: {self.collection_name}")
            
            return True
        except Exception as e:
            print(f"Error ensuring collection: {e}")
            return False
    
    async def upsert_chunks(
        self,
        chunks: List[DocumentChunk],
        embeddings: List[List[float]]
    ) -> int:
        """
        Upsert document chunks with their embeddings.
        
        Args:
            chunks: List of document chunks
            embeddings: Corresponding embeddings for each chunk
            
        Returns:
            Number of points upserted
        """
        if len(chunks) != len(embeddings):
            raise ValueError("Chunks and embeddings must have same length")
        
        points = []
        for chunk, embedding in zip(chunks, embeddings):
            point = PointStruct(
                id=str(uuid.uuid5(uuid.NAMESPACE_DNS, chunk.id)),
                vector=embedding,
                payload={
                    "text": chunk.text,
                    "source": chunk.source,
                    "chapter": chunk.chapter,
                    "module": chunk.module,
                    "chunk_index": chunk.chunk_index,
                    "metadata": chunk.metadata
                }
            )
            points.append(point)
        
        # Upsert in batches of 100
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            await self.client.upsert(
                collection_name=self.collection_name,
                points=batch
            )

        
        return len(points)
    
    async def search(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        module_filter: Optional[str] = None,
        chapter_filter: Optional[str] = None
    ) -> List[SearchResult]:
        """
        Search for similar documents using query embedding.
        
        Args:
            query_embedding: Query vector
            top_k: Number of results to return
            module_filter: Optional filter by module name
            chapter_filter: Optional filter by chapter name
            
        Returns:
            List of search results with scores
        """
        # Build filter if specified
        filter_conditions = []
        if module_filter:
            filter_conditions.append(
                FieldCondition(
                    key="module",
                    match=MatchValue(value=module_filter)
                )
            )
        if chapter_filter:
            filter_conditions.append(
                FieldCondition(
                    key="chapter",
                    match=MatchValue(value=chapter_filter)
                )
            )
        
        query_filter = None
        if filter_conditions:
            query_filter = Filter(must=filter_conditions)
        
        # Perform search using query_points (modern universal method)
        results = await self.client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            query_filter=query_filter,
            limit=top_k,
            with_payload=True
        )
        
        # Results from query_points are also ScoredPoint objects or similar
        # depending on the client version/mode.
        points = results.points if hasattr(results, 'points') else results


        
        # Convert to SearchResult models
        search_results = []
        for result in points:
            payload = result.payload
            search_results.append(SearchResult(
                text=payload.get("text", ""),
                source=payload.get("source", ""),
                chapter=payload.get("chapter", ""),
                module=payload.get("module", ""),
                score=result.score
            ))

        
        return search_results
    
    async def delete_collection(self) -> bool:
        """Delete the collection (use with caution)."""
        try:
            await self.client.delete_collection(self.collection_name)
            print(f"Deleted collection: {self.collection_name}")
            return True

        except Exception as e:
            print(f"Error deleting collection: {e}")
            return False
    
    async def get_collection_info(self) -> Dict[str, Any]:
        """Get information about the collection."""
        try:
            info = await self.client.get_collection(self.collection_name)
            return {

                "name": self.collection_name,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status.value
            }
        except Exception as e:
            return {"error": str(e)}


# Global singleton instance
_qdrant_service: QdrantService = None


def get_qdrant_service() -> QdrantService:
    """Get or create the global Qdrant service instance."""
    global _qdrant_service
    if _qdrant_service is None:
        _qdrant_service = QdrantService()
    return _qdrant_service
