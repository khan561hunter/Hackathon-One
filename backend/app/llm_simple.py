"""
Simple fallback text generator
Returns formatted context when LLM APIs are unavailable
"""

from typing import List, Dict, Any
import logging

logger = logging.getLogger(__name__)


class SimpleGenerator:
    """
    Simple text generator that formats retrieved context
    Fallback for when LLM APIs are unavailable
    """

    def __init__(self):
        logger.info("Initialized Simple Generator (fallback mode)")

    def generate_rag_answer(
        self,
        question: str,
        context_chunks: List[Dict[str, Any]],
        max_context_length: int = 4000
    ) -> str:
        """
        Format retrieved context as answer

        Args:
            question: User's question
            context_chunks: Retrieved chunks from Qdrant
            max_context_length: Maximum characters

        Returns:
            Formatted answer with context
        """
        if not context_chunks:
            return "I couldn't find relevant information in the documentation."

        # Build answer from context
        answer_parts = [f"Based on the documentation, here's what I found about: **{question}**\n"]

        for i, chunk in enumerate(context_chunks[:3]):  # Limit to top 3
            text = chunk.get("text", "")
            metadata = chunk.get("metadata", {})
            doc_name = metadata.get("doc_name", "Unknown")
            score = chunk.get("score", 0)

            # Clean up the text
            text = text.strip()
            if len(text) > 500:
                text = text[:500] + "..."

            answer_parts.append(f"\n**From {doc_name}** (relevance: {score:.0%}):\n{text}\n")

        answer_parts.append("\n*Note: This response shows retrieved documentation. For AI-generated answers, please configure a valid Claude API key.*")

        return "\n".join(answer_parts)

    def generate_direct_answer(
        self,
        question: str,
        selected_text: str
    ) -> str:
        """
        Format selected text with question

        Args:
            question: User's question
            selected_text: Text selected by user

        Returns:
            Formatted response
        """
        return f"""**Question:** {question}

**Selected Text:**
{selected_text[:1000]}

**Response:**
The selected text discusses the topic you asked about. For detailed AI analysis, please configure a valid Claude API key.
"""

    def generate_streaming_response(self, question: str, context: str, is_rag: bool = True):
        """Streaming not supported in simple mode"""
        if is_rag:
            yield self.generate_rag_answer(question, [{"text": context, "metadata": {}}])
        else:
            yield self.generate_direct_answer(question, context)


def get_simple_generator() -> SimpleGenerator:
    """Get simple generator instance"""
    return SimpleGenerator()
