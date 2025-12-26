"""
Configuration settings for the RAG Chatbot API.
Loads environment variables and provides typed settings.
"""

import os
from typing import List
from pydantic_settings import BaseSettings
from pydantic import Field


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""
    
    # Google Gemini Configuration
    gemini_api_key: str = Field(..., env="GEMINI_API_KEY")
    gemini_model: str = Field(default="gemini-flash-latest", env="GEMINI_MODEL")
    
    # Qdrant Cloud Configuration
    qdrant_url: str = Field(..., env="QDRANT_URL")
    qdrant_api_key: str = Field(..., env="QDRANT_API_KEY")
    qdrant_collection_name: str = Field(default="book_content_v2", env="QDRANT_COLLECTION")

    
    # Neon Postgres Configuration
    database_url: str = Field(..., env="DATABASE_URL")
    
    # Application Settings
    cors_origins: str = Field(
        default="http://localhost:3000,http://localhost:3001",
        env="CORS_ORIGINS"
    )
    debug: bool = Field(default=False, env="DEBUG")
    
    # Embedding Settings
    embedding_model: str = Field(
        default="models/text-embedding-004",
        env="EMBEDDING_MODEL"
    )
    embedding_dimension: int = Field(default=768)
    
    # Search Settings
    top_k_results: int = Field(default=5, env="TOP_K_RESULTS")
    similarity_threshold: float = Field(default=0.5, env="SIMILARITY_THRESHOLD")
    
    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]
    
    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False


# Global settings instance
settings = Settings()
