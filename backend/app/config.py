"""Configuration module for the RAG chatbot backend."""

from pydantic_settings import BaseSettings
from functools import lru_cache


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Cohere API
    cohere_api_key: str = ""

    # Qdrant
    qdrant_url: str = ""
    qdrant_api_key: str = ""
    qdrant_collection: str = "book_docs"

    # Google Gemini (free tier: https://aistudio.google.com/apikey)
    gemini_api_key: str = ""
    gemini_model: str = "gemini-2.5-flash"

    # CORS
    cors_origins: str = (
        "http://localhost:3000,http://localhost:3001,https://vercel.com/mashhood-husssains-projects/physical-ai-and-humanoid-robotics-course-book"
    )

    # RAG Settings
    embedding_model: str = "embed-english-v3.0"
    embedding_dimensions: int = 1024
    chunk_size: int = 2000  # characters (~500 tokens)
    chunk_overlap: int = 200  # characters (~50 tokens)
    top_k: int = 5
    relevance_threshold: float = 0.3

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"

    @property
    def cors_origins_list(self) -> list[str]:
        """Parse CORS origins from comma-separated string."""
        return [
            origin.strip() for origin in self.cors_origins.split(",") if origin.strip()
        ]


@lru_cache
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
