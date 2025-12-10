"""
Qdrant Vector Database Setup and Operations
Handles vector storage and similarity search for RAG
"""

import os
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import logging

logger = logging.getLogger(__name__)


class QdrantManager:
    """
    Qdrant vector database manager for embeddings storage and retrieval
    """

    def __init__(
        self,
        url: Optional[str] = None,
        api_key: Optional[str] = None,
        collection_name: Optional[str] = None
    ):
        """
        Initialize Qdrant manager

        Args:
            url: Qdrant Cloud URL (defaults to QDRANT_URL env var)
            api_key: Qdrant API key (defaults to QDRANT_API_KEY env var)
            collection_name: Name of the collection to use (defaults to QDRANT_COLLECTION env var)
        """
        self.url = url or os.getenv("QDRANT_URL")
        self.api_key = api_key or os.getenv("QDRANT_API_KEY")
        self.collection_name = collection_name or os.getenv("QDRANT_COLLECTION", "documentation_chunks")

        if not self.url or not self.api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set")

        # Initialize client
        self.client = QdrantClient(
            url=self.url,
            api_key=self.api_key,
            timeout=30
        )

        logger.info(f"Initialized Qdrant manager with collection: {collection_name}")

    def create_collection(self, vector_size: int = 1024):
        """
        Create collection if it doesn't exist

        Args:
            vector_size: Dimension of embedding vectors (1024 for Cohere)
        """
        try:
            collections = self.client.get_collections().collections
            collection_names = [c.name for c in collections]

            if self.collection_name not in collection_names:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=vector_size,
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"Created collection: {self.collection_name}")
            else:
                logger.info(f"Collection {self.collection_name} already exists")

        except Exception as e:
            logger.error(f"Failed to create collection: {e}")
            raise

    def upload_embeddings(
        self,
        embeddings: List[List[float]],
        texts: List[str],
        metadata_list: List[Dict[str, Any]]
    ):
        """
        Upload embeddings to Qdrant

        Args:
            embeddings: List of embedding vectors
            texts: List of text chunks
            metadata_list: List of metadata dicts for each chunk
        """
        if not (len(embeddings) == len(texts) == len(metadata_list)):
            raise ValueError("Embeddings, texts, and metadata must have same length")

        try:
            points = []
            for idx, (embedding, text, metadata) in enumerate(zip(embeddings, texts, metadata_list)):
                point = PointStruct(
                    id=idx,
                    vector=embedding,
                    payload={
                        "text": text,
                        "metadata": metadata
                    }
                )
                points.append(point)

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Uploaded {len(points)} embeddings to Qdrant")

        except Exception as e:
            logger.error(f"Failed to upload embeddings: {e}")
            raise

    def search_similar(
        self,
        query_embedding: List[float],
        top_k: int = 3,
        score_threshold: float = 0.0
    ) -> List[Dict[str, Any]]:
        """
        Search for similar chunks

        Args:
            query_embedding: Query vector
            top_k: Number of results to return
            score_threshold: Minimum similarity score

        Returns:
            List of matching chunks with metadata and scores
        """
        try:
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                score_threshold=score_threshold
            ).points

            # Format results
            formatted_results = []
            for result in results:
                # Extract metadata fields from payload
                metadata = {
                    "doc_name": result.payload.get("doc_name", "Unknown"),
                    "chunk_index": result.payload.get("chunk_index", 0),
                    "total_chunks": result.payload.get("total_chunks", 0),
                    "file_path": result.payload.get("file_path", "")
                }
                formatted_results.append({
                    "text": result.payload.get("text", ""),
                    "metadata": metadata,
                    "score": result.score
                })

            logger.info(f"Found {len(formatted_results)} similar chunks")
            return formatted_results

        except Exception as e:
            logger.error(f"Search failed: {e}")
            raise

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get collection information

        Returns:
            Dict with collection stats
        """
        try:
            info = self.client.get_collection(self.collection_name)
            return {
                "name": info.name,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status
            }
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            raise

    def delete_collection(self):
        """Delete the collection"""
        try:
            self.client.delete_collection(self.collection_name)
            logger.info(f"Deleted collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Failed to delete collection: {e}")
            raise


# Global instance (lazy initialization)
_qdrant_manager: Optional[QdrantManager] = None


def get_qdrant_manager() -> QdrantManager:
    """
    Get or create global Qdrant manager instance

    Returns:
        QdrantManager instance
    """
    global _qdrant_manager
    if _qdrant_manager is None:
        _qdrant_manager = QdrantManager()
    return _qdrant_manager
