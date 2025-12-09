"""
RAG (Retrieval-Augmented Generation) Core Logic
Orchestrates the complete RAG workflow:
1. Embed question with Cohere
2. Retrieve relevant chunks from Qdrant
3. Generate answer with Qwen LLM
"""

from typing import List, Dict, Any, Optional
import logging
from .embeddings_cohere import get_cohere_embeddings
from .llm_simple import get_simple_generator  # Using simple fallback for generation
from .qdrant_setup import get_qdrant_manager
from .postgres import get_postgres_manager

logger = logging.getLogger(__name__)


class RAGPipeline:
    """
    Complete RAG pipeline for question answering
    """

    def __init__(
        self,
        top_k: int = 3,
        score_threshold: float = 0.0,  # No threshold - return all results
        max_context_length: int = 4000
    ):
        """
        Initialize RAG pipeline

        Args:
            top_k: Number of chunks to retrieve
            score_threshold: Minimum similarity score for retrieval
            max_context_length: Maximum characters of context for Claude
        """
        self.top_k = top_k
        self.score_threshold = score_threshold
        self.max_context_length = max_context_length

        # Initialize components
        self.embeddings = get_cohere_embeddings()
        self.generator = get_simple_generator()  # Using simple fallback
        self.vector_db = get_qdrant_manager()
        self.postgres = get_postgres_manager()

        logger.info("Initialized RAG pipeline with Cohere embeddings and simple text generation")

    async def answer_question(
        self,
        question: str,
        log_to_db: bool = True
    ) -> Dict[str, Any]:
        """
        Answer a question using RAG workflow

        Args:
            question: User's question
            log_to_db: Whether to log interaction to Postgres

        Returns:
            Dict with answer, sources, and metadata
        """
        try:
            # Step 1: Embed question using Cohere (NOT Claude!)
            logger.info(f"Embedding question: {question[:50]}...")
            query_embedding = self.embeddings.embed_query(question)

            # Step 2: Retrieve relevant chunks from Qdrant
            logger.info(f"Retrieving top {self.top_k} chunks from Qdrant")
            retrieved_chunks = self.vector_db.search_similar(
                query_embedding=query_embedding,
                top_k=self.top_k,
                score_threshold=self.score_threshold
            )

            if not retrieved_chunks:
                answer = "I couldn't find relevant information in the documentation to answer your question. Please try rephrasing or ask about a different topic."

                if log_to_db:
                    await self.postgres.log_chat_interaction(
                        question=question,
                        answer=answer,
                        retrieved_docs=[],
                        model_used="qwen-plus",
                        response_time=None
                    )

                return {
                    "answer": answer,
                    "sources": [],
                    "chunks_found": 0
                }

            # Step 3: Generate answer using Claude 4.5 Sonnet (NOT for embeddings!)
            logger.info(f"Generating answer with Claude using {len(retrieved_chunks)} chunks")
            answer = self.generator.generate_rag_answer(
                question=question,
                context_chunks=retrieved_chunks,
                max_context_length=self.max_context_length
            )

            # Prepare sources for response
            sources = []
            for chunk in retrieved_chunks:
                sources.append({
                    "doc_name": chunk["metadata"].get("doc_name", "Unknown"),
                    "chunk_index": chunk["metadata"].get("chunk_index", 0),
                    "score": chunk["score"]
                })

            # Log to Postgres
            if log_to_db:
                import time
                retrieved_docs_meta = [
                    {
                        "doc_name": c["metadata"].get("doc_name", "Unknown"),
                        "chunk_index": c["metadata"].get("chunk_index", 0),
                        "score": c["score"]
                    }
                    for c in retrieved_chunks
                ]
                await self.postgres.log_chat_interaction(
                    question=question,
                    answer=answer,
                    retrieved_docs=retrieved_docs_meta,
                    model_used="qwen-plus",
                    response_time=None  # Can add timing later
                )

            return {
                "answer": answer,
                "sources": sources,
                "chunks_found": len(retrieved_chunks)
            }

        except Exception as e:
            logger.error(f"RAG pipeline error: {e}")
            raise Exception(f"Failed to answer question: {e}")

    async def answer_with_selected_text(
        self,
        selected_text: str,
        question: str,
        log_to_db: bool = True
    ) -> Dict[str, Any]:
        """
        Answer question based on selected text (NO retrieval)

        Args:
            selected_text: Text selected by user
            question: User's question
            log_to_db: Whether to log interaction to Postgres

        Returns:
            Dict with answer and metadata
        """
        try:
            logger.info(f"Answering question with selected text (length: {len(selected_text)})")

            # Generate answer directly with Claude (no RAG retrieval)
            answer = self.generator.generate_direct_answer(
                question=question,
                selected_text=selected_text
            )

            # Log to Postgres
            if log_to_db:
                await self.postgres.log_selection_interaction(
                    selected_text=selected_text,
                    question=question,
                    answer=answer
                )

            return {
                "answer": answer,
                "mode": "selected_text",
                "text_length": len(selected_text)
            }

        except Exception as e:
            logger.error(f"Selected text answer error: {e}")
            raise Exception(f"Failed to answer with selected text: {e}")

    def stream_answer(
        self,
        question: str,
        context_chunks: List[Dict[str, Any]]
    ):
        """
        Stream answer generation for real-time UI updates

        Args:
            question: User's question
            context_chunks: Retrieved chunks from Qdrant

        Yields:
            Text chunks as they're generated
        """
        # Build context string
        context_parts = []
        for i, chunk in enumerate(context_chunks):
            chunk_text = chunk.get("text", "")
            metadata = chunk.get("metadata", {})
            doc_name = metadata.get("doc_name", "Unknown")
            context_parts.append(f"[Source {i+1}: {doc_name}]\n{chunk_text}\n")

        context = "\n".join(context_parts)

        # Stream response from Claude
        for text_chunk in self.generator.generate_streaming_response(
            question=question,
            context=context,
            is_rag=True
        ):
            yield text_chunk

    def stream_answer_with_selection(
        self,
        question: str,
        selected_text: str
    ):
        """
        Stream answer for selected text

        Args:
            question: User's question
            selected_text: Text selected by user

        Yields:
            Text chunks as they're generated
        """
        for text_chunk in self.generator.generate_streaming_response(
            question=question,
            context=selected_text,
            is_rag=False
        ):
            yield text_chunk


# Global instance (lazy initialization)
_rag_pipeline: Optional[RAGPipeline] = None


def get_rag_pipeline() -> RAGPipeline:
    """
    Get or create global RAG pipeline instance

    Returns:
        RAGPipeline instance
    """
    global _rag_pipeline
    if _rag_pipeline is None:
        _rag_pipeline = RAGPipeline()
    return _rag_pipeline
