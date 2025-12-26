"""
Embeddings service using sentence-transformers with all-MiniLM-L6-v2.
Provides text embedding generation for RAG pipeline.
"""

from typing import List, Union
from sentence_transformers import SentenceTransformer
import numpy as np
from functools import lru_cache


class EmbeddingsService:
    """Service for generating text embeddings using all-MiniLM-L6-v2."""
    
    def __init__(self, model_name: str = "all-MiniLM-L6-v2"):
        """
        Initialize the embeddings service.
        
        Args:
            model_name: Name of the sentence-transformers model to use
        """
        self.model_name = model_name
        self._model = None
    
    @property
    def model(self) -> SentenceTransformer:
        """Lazy load the model to save memory."""
        if self._model is None:
            print(f"Loading embedding model: {self.model_name}")
            # FORCE OFFLINE MODE to prevent connection timeouts
            import os
            os.environ["HF_HUB_OFFLINE"] = "1"
            try:
                self._model = SentenceTransformer(self.model_name)
            except Exception as e:
                print(f"Failed to load model offline, trying online: {e}")
                del os.environ["HF_HUB_OFFLINE"]
                self._model = SentenceTransformer(self.model_name)
                
            print(f"Model loaded. Embedding dimension: {self.dimension}")
        return self._model
    
    @property
    def dimension(self) -> int:
        """Get the embedding dimension (384 for all-MiniLM-L6-v2)."""
        return self.model.get_sentence_embedding_dimension()
    
    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.
        
        Args:
            text: Text to embed
            
        Returns:
            List of floats representing the embedding vector
        """
        embedding = self.model.encode(text, convert_to_numpy=True)
        return embedding.tolist()
    
    def embed_texts(self, texts: List[str], batch_size: int = 32) -> List[List[float]]:
        """
        Generate embeddings for multiple texts.
        
        Args:
            texts: List of texts to embed
            batch_size: Batch size for encoding
            
        Returns:
            List of embedding vectors
        """
        embeddings = self.model.encode(
            texts,
            batch_size=batch_size,
            convert_to_numpy=True,
            show_progress_bar=True
        )
        return embeddings.tolist()
    
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
