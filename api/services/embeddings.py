"""
Embeddings service using Google Generative AI (Gemini).
Provides text embedding generation for RAG pipeline.
"""

from typing import List, Union
import google.generativeai as genai
import numpy as np
from config import settings


class EmbeddingsService:
    """Service for generating text embeddings using Google Gemini."""
    
    def __init__(self, model_name: str = None):
        """
        Initialize the embeddings service.
        
        Args:
            model_name: Name of the Gemini embedding model to use
        """
        self.model_name = model_name or settings.embedding_model
        genai.configure(api_key=settings.gemini_api_key)
        self._dimension = settings.embedding_dimension
    
    @property
    def dimension(self) -> int:
        """Get the embedding dimension (768 for text-embedding-004)."""
        return self._dimension
    
    def embed_text(self, text: str, task_type: str = "retrieval_document") -> List[float]:
        """
        Generate embedding for a single text.
        
        Args:
            text: Text to embed
            task_type: Type of task (retrieval_document or retrieval_query)
            
        Returns:
            List of floats representing the embedding vector
        """
        if not text.strip():
            return [0.0] * self.dimension
            
        result = genai.embed_content(
            model=self.model_name,
            content=text,
            task_type=task_type
        )
        return result['embedding']
    
    def embed_texts(self, texts: List[str], task_type: str = "retrieval_document") -> List[List[float]]:
        """
        Generate embeddings for multiple texts.
        
        Args:
            texts: List of texts to embed
            task_type: Type of task (retrieval_document or retrieval_query)
            
        Returns:
            List of embedding vectors
        """
        if not texts:
            return []
            
        # Clean text
        cleaned_texts = [t if t.strip() else " " for t in texts]
        
        result = genai.embed_content(
            model=self.model_name,
            content=cleaned_texts,
            task_type=task_type
        )
        return result['embeddings']

    
    def similarity(self, embedding1: List[float], embedding2: List[float]) -> float:
        """
        Calculate cosine similarity between two embeddings.
        
        Args:
            embedding1: First embedding vector
            embedding2: Second embedding vector
            
        Returns:
            Cosine similarity score (0 to 1)
        """
        vec1 = np.array(embedding1)
        vec2 = np.array(embedding2)
        
        dot_product = np.dot(vec1, vec2)
        norm1 = np.linalg.norm(vec1)
        norm2 = np.linalg.norm(vec2)
        
        if norm1 == 0 or norm2 == 0:
            return 0.0
        
        return float(dot_product / (norm1 * norm2))


# Global singleton instance
_embeddings_service: EmbeddingsService = None


def get_embeddings_service() -> EmbeddingsService:
    """Get or create the global embeddings service instance."""
    global _embeddings_service
    if _embeddings_service is None:
        _embeddings_service = EmbeddingsService()
    return _embeddings_service
