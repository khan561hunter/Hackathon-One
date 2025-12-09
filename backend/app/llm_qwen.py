"""
Qwen LLM Module (Alternative)
Uses Alibaba's Qwen model for text generation
Note: Currently not used due to API limitations
"""

import os
import requests
from typing import List, Dict, Any
import logging

logger = logging.getLogger(__name__)


class QwenGenerator:
    """
    Qwen-based text generator
    """

    def __init__(self, api_key: str = None, model: str = "qwen-plus"):
        """
        Initialize Qwen generator

        Args:
            api_key: Qwen API key
            model: Qwen model to use
        """
        self.api_key = api_key or os.getenv("DASHSCOPE_API_KEY")
        if not self.api_key:
            raise ValueError("DASHSCOPE_API_KEY not found")

        self.model = model
        self.base_url = "https://dashscope.aliyuncs.com/api/v1/services/aigc/text-generation/generation"
        logger.info(f"Initialized Qwen generator with model: {model}")

    def generate_rag_answer(
        self,
        question: str,
        context_chunks: List[Dict[str, Any]],
        max_context_length: int = 4000
    ) -> str:
        """
        Generate answer using RAG context

        Args:
            question: User's question
            context_chunks: Retrieved chunks
            max_context_length: Max context length

        Returns:
            Generated answer
        """
        # Build context
        context_parts = []
        for i, chunk in enumerate(context_chunks[:3]):
            text = chunk.get("text", "")
            doc_name = chunk.get("metadata", {}).get("doc_name", "Unknown")
            context_parts.append(f"[Source {i+1}: {doc_name}]\n{text}")

        context = "\n\n".join(context_parts)

        prompt = f"""Based on the following documentation context, answer the question concisely and accurately.

Context:
{context}

Question: {question}

Answer:"""

        return self._call_api(prompt)

    def generate_direct_answer(self, question: str, selected_text: str) -> str:
        """Generate answer for selected text"""
        prompt = f"""Based on the following text, answer the question.

Text:
{selected_text}

Question: {question}

Answer:"""

        return self._call_api(prompt)

    def _call_api(self, prompt: str) -> str:
        """Call Qwen API"""
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }

        payload = {
            "model": self.model,
            "input": {
                "messages": [
                    {
                        "role": "system",
                        "content": "You are a helpful documentation assistant."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ]
            },
            "parameters": {
                "result_format": "message"
            }
        }

        try:
            response = requests.post(self.base_url, json=payload, headers=headers)
            response.raise_for_status()
            data = response.json()

            return data["output"]["choices"][0]["message"]["content"]

        except Exception as e:
            logger.error(f"Qwen generation error: {e}")
            raise Exception(f"Failed to generate with Qwen: {e}")

    def generate_streaming_response(self, question: str, context: str, is_rag: bool = True):
        """Streaming not implemented for Qwen"""
        answer = self._call_api(f"Context: {context}\n\nQuestion: {question}")
        yield answer


def get_qwen_generator() -> QwenGenerator:
    """Get Qwen generator instance"""
    return QwenGenerator()
