"""Qdrant vector store service."""

from typing import Optional
from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue,
)

from app.config import get_settings


class QdrantVectorStore:
    """Service for storing and searching vectors in Qdrant."""

    def __init__(
        self,
        url: Optional[str] = None,
        api_key: Optional[str] = None,
        collection_name: Optional[str] = None,
    ):
        """Initialize the Qdrant client."""
        settings = get_settings()
        self.client = QdrantClient(
            url=url or settings.qdrant_url,
            api_key=api_key or settings.qdrant_api_key,
        )
        self.collection_name = collection_name or settings.qdrant_collection
        self.vector_size = settings.embedding_dimensions

    def create_collection(self) -> None:
        """Create the collection if it doesn't exist."""
        collections = self.client.get_collections().collections
        collection_names = [c.name for c in collections]

        if self.collection_name not in collection_names:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,
                    distance=Distance.COSINE,
                ),
            )

    def upsert(
        self,
        ids: list[str],
        embeddings: list[list[float]],
        payloads: list[dict],
    ) -> None:
        """
        Upsert vectors with payloads into the collection.

        Args:
            ids: Unique identifiers for each point
            embeddings: Vector embeddings
            payloads: Metadata for each point (text, file_path, chapter, section, position)
        """
        points = [
            PointStruct(
                id=idx,
                vector=embedding,
                payload=payload,
            )
            for idx, (embedding, payload) in enumerate(zip(embeddings, payloads))
        ]

        # Upsert in batches of 100
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i : i + batch_size]
            self.client.upsert(
                collection_name=self.collection_name,
                points=batch,
            )

    def search(
        self,
        query_embedding: list[float],
        top_k: int = 5,
        score_threshold: Optional[float] = None,
        filter_conditions: Optional[dict] = None,
    ) -> list[dict]:
        """
        Search for similar vectors.

        Args:
            query_embedding: The query vector
            top_k: Number of results to return
            score_threshold: Minimum similarity score (0-1)
            filter_conditions: Optional filter on metadata fields

        Returns:
            List of results with text, metadata, and relevance score
        """
        settings = get_settings()
        threshold = score_threshold or settings.relevance_threshold

        # Build filter if conditions provided
        query_filter = None
        if filter_conditions:
            must_conditions = [
                FieldCondition(key=k, match=MatchValue(value=v))
                for k, v in filter_conditions.items()
            ]
            query_filter = Filter(must=must_conditions)

        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=top_k,
            score_threshold=threshold,
            query_filter=query_filter,
        )

        return [
            {
                "text": hit.payload.get("text", ""),
                "file_path": hit.payload.get("file_path", ""),
                "chapter": hit.payload.get("chapter", ""),
                "section": hit.payload.get("section", ""),
                "position": hit.payload.get("position", 0),
                "relevance": hit.score,
            }
            for hit in results.points
        ]

    def delete_collection(self) -> None:
        """Delete the collection."""
        self.client.delete_collection(collection_name=self.collection_name)

    def count(self) -> int:
        """Get the number of points in the collection."""
        info = self.client.get_collection(collection_name=self.collection_name)
        return info.points_count
