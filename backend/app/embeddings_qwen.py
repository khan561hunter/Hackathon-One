"""
Qwen Embeddings Module (Alternative)
Note: Currently using Cohere for embeddings, but this module is kept for reference
"""

import os
import requests
from typing import List
import logging

logger = logging.getLogger(__name__)


class QwenEmbeddings:
    """
    Qwen embeddings wrapper (not currently used)
    """

    def __init__(self, api_key: str = None, model: str = "text-embedding-v3"):
        """
        Initialize Qwen embeddings

        Args:
            api_key: Qwen API key
            model: Qwen embedding model
        """
        self.api_key = api_key or os.getenv("DASHSCOPE_API_KEY")
        if not self.api_key:
            raise ValueError("DASHSCOPE_API_KEY not found")

        self.model = model
        self.base_url = "https://dashscope.aliyuncs.com/api/v1/services/embeddings/text-embedding/text-embedding"
        logger.info(f"Initialized Qwen embeddings with model: {model}")

    def embed_documents(self, texts: List[str]) -> List[List[float]]:
        """Embed multiple documents"""
        embeddings = []
        for text in texts:
            embedding = self.embed_query(text)
            embeddings.append(embedding)
        return embeddings

    def embed_query(self, text: str) -> List[float]:
        """Embed a single query"""
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }

        payload = {
            "model": self.model,
            "input": {
                "texts": [text]
            }
        }

        try:
            response = requests.post(self.base_url, json=payload, headers=headers)
            response.raise_for_status()
            data = response.json()
            return data["output"]["embeddings"][0]["embedding"]
        except Exception as e:
            logger.error(f"Error embedding with Qwen: {e}")
            raise


def get_qwen_embeddings() -> QwenEmbeddings:
    """Get Qwen embeddings instance"""
    return QwenEmbeddings()
