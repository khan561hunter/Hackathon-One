"""
Claude LLM Module
Uses Anthropic's Claude 4.5 Sonnet for text generation
"""

import os
from anthropic import Anthropic
from typing import List, Dict, Any
import logging

logger = logging.getLogger(__name__)


class ClaudeGenerator:
    """
    Claude-based text generator for RAG answers
    """

    def __init__(self, api_key: str = None, model: str = "claude-sonnet-4-20250514"):
        """
        Initialize Claude generator

        Args:
            api_key: Anthropic API key
            model: Claude model to use
        """
        self.api_key = api_key or os.getenv("ANTHROPIC_API_KEY")
        if not self.api_key:
            raise ValueError("ANTHROPIC_API_KEY not found in environment variables")

        self.model = model
        self.client = Anthropic(api_key=self.api_key)
        logger.info(f"Initialized Claude generator with model: {model}")

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
            context_chunks: Retrieved chunks from vector DB
            max_context_length: Maximum characters of context

        Returns:
            Generated answer
        """
        # Build context from chunks
        context_parts = []
        total_length = 0

        for i, chunk in enumerate(context_chunks):
            chunk_text = chunk.get("text", "")
            metadata = chunk.get("metadata", {})
            doc_name = metadata.get("doc_name", "Unknown")

            # Add source header and text
            source_text = f"[Source {i+1}: {doc_name}]\n{chunk_text}\n"

            if total_length + len(source_text) > max_context_length:
                break

            context_parts.append(source_text)
            total_length += len(source_text)

        context = "\n".join(context_parts)

        # Create prompt
        system_prompt = """You are a helpful documentation assistant. Answer questions based on the provided documentation context.

Guidelines:
- Be concise and accurate
- If the context doesn't contain enough information, say so
- Reference specific sources when possible
- Use clear, technical language"""

        user_prompt = f"""Context from documentation:

{context}

Question: {question}

Please provide a clear, accurate answer based on the context above."""

        try:
            response = self.client.messages.create(
                model=self.model,
                max_tokens=1024,
                system=system_prompt,
                messages=[
                    {"role": "user", "content": user_prompt}
                ]
            )

            return response.content[0].text

        except Exception as e:
            logger.error(f"Claude generation error: {e}")
            raise Exception(f"Failed to generate answer with Claude: {e}")

    def generate_direct_answer(
        self,
        question: str,
        selected_text: str
    ) -> str:
        """
        Generate answer for selected text (no retrieval)

        Args:
            question: User's question
            selected_text: Text selected by user

        Returns:
            Generated answer
        """
        system_prompt = """You are a helpful assistant. Answer questions about the provided text selection clearly and accurately."""

        user_prompt = f"""Selected text:

{selected_text}

Question: {question}

Please answer the question based on the selected text above."""

        try:
            response = self.client.messages.create(
                model=self.model,
                max_tokens=1024,
                system=system_prompt,
                messages=[
                    {"role": "user", "content": user_prompt}
                ]
            )

            return response.content[0].text

        except Exception as e:
            logger.error(f"Claude generation error: {e}")
            raise Exception(f"Failed to generate answer with Claude: {e}")

    def generate_streaming_response(
        self,
        question: str,
        context: str,
        is_rag: bool = True
    ):
        """
        Stream response generation

        Args:
            question: User's question
            context: Context text or selected text
            is_rag: Whether this is RAG mode or selection mode

        Yields:
            Text chunks as they're generated
        """
        if is_rag:
            system_prompt = "You are a helpful documentation assistant."
            user_prompt = f"Context:\n\n{context}\n\nQuestion: {question}\n\nAnswer:"
        else:
            system_prompt = "You are a helpful assistant."
            user_prompt = f"Text:\n\n{context}\n\nQuestion: {question}\n\nAnswer:"

        try:
            with self.client.messages.stream(
                model=self.model,
                max_tokens=1024,
                system=system_prompt,
                messages=[{"role": "user", "content": user_prompt}]
            ) as stream:
                for text in stream.text_stream:
                    yield text

        except Exception as e:
            logger.error(f"Claude streaming error: {e}")
            yield f"Error: {e}"


def get_claude_generator() -> ClaudeGenerator:
    """Get Claude generator instance"""
    return ClaudeGenerator()
