"""Cohere embeddings service."""

import cohere
from typing import Optional

from app.config import get_settings


class CohereEmbeddings:
    """Service for generating embeddings using Cohere API."""

    def __init__(self, api_key: Optional[str] = None):
        """Initialize the Cohere client."""
        settings = get_settings()
        self.client = cohere.Client(api_key or settings.cohere_api_key)
        self.model = settings.embedding_model

    def embed_query(self, text: str) -> list[float]:
        """
        Embed a single query text.

        Uses input_type="search_query" for optimal retrieval performance.
        """
        response = self.client.embed(
            texts=[text],
            model=self.model,
            input_type="search_query",
        )
        return response.embeddings[0]

    def embed_documents(self, texts: list[str]) -> list[list[float]]:
        """
        Embed multiple document texts.

        Uses input_type="search_document" for optimal retrieval performance.
        Handles batching automatically (Cohere supports up to 96 texts per call).
        """
        if not texts:
            return []

        batch_size = 96
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i : i + batch_size]
            response = self.client.embed(
                texts=batch,
                model=self.model,
                input_type="search_document",
            )
            all_embeddings.extend(response.embeddings)

        return all_embeddings
