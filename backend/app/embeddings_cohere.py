"""
Cohere Embeddings Module
Uses Cohere's embed-english-v3.0 model for generating embeddings
"""

import os
import cohere
from typing import List
import logging

logger = logging.getLogger(__name__)


class CohereEmbeddings:
    """
    Cohere embeddings wrapper for generating text embeddings
    """

    def __init__(self, api_key: str = None, model: str = "embed-english-v3.0"):
        """
        Initialize Cohere embeddings

        Args:
            api_key: Cohere API key (defaults to COHERE_API_KEY env var)
            model: Cohere embedding model to use
        """
        self.api_key = api_key or os.getenv("COHERE_API_KEY")
        if not self.api_key:
            raise ValueError("COHERE_API_KEY not found in environment variables")

        self.model = model
        self.client = cohere.Client(self.api_key)
        logger.info(f"Initialized Cohere embeddings with model: {model}")

    def embed_documents(self, texts: List[str]) -> List[List[float]]:
        """
        Embed a list of documents

        Args:
            texts: List of text strings to embed

        Returns:
            List of embedding vectors
        """
        try:
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type="search_document"
            )
            return response.embeddings
        except Exception as e:
            logger.error(f"Error embedding documents: {e}")
            raise

    def embed_query(self, text: str) -> List[float]:
        """
        Embed a single query string

        Args:
            text: Query text to embed

        Returns:
            Embedding vector
        """
        try:
            response = self.client.embed(
                texts=[text],
                model=self.model,
                input_type="search_query"
            )
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Error embedding query: {e}")
            raise


# Global instance (lazy initialization)
_cohere_embeddings = None


def get_cohere_embeddings() -> CohereEmbeddings:
    """
    Get or create global Cohere embeddings instance

    Returns:
        CohereEmbeddings instance
    """
    global _cohere_embeddings
    if _cohere_embeddings is None:
        _cohere_embeddings = CohereEmbeddings()
    return _cohere_embeddings
