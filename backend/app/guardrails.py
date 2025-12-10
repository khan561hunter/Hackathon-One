"""
Guardrails for chatbot to ensure it only answers book-related questions
Validates questions against Physical AI and Humanoid Robotics topics
"""

from typing import Dict, List, Optional
import logging
import re

logger = logging.getLogger(__name__)


class ChatbotGuardrails:
    """
    Validates user questions to ensure they're related to the book content
    """

    # Book topics: Physical AI, Humanoid Robotics, ROS2, etc.
    ALLOWED_TOPICS = [
        # Core concepts
        "physical ai", "physical artificial intelligence",
        "humanoid robot", "humanoid", "robotics",
        "embodied ai", "embodied intelligence",

        # Technical topics
        "ros2", "ros", "robot operating system",
        "perception", "computer vision", "lidar", "sensor",
        "manipulation", "grasping", "motion planning", "trajectory",
        "navigation", "slam", "localization", "mapping",
        "control", "actuator", "servo", "motor",
        "kinematics", "inverse kinematics", "forward kinematics",
        "dynamics", "locomotion", "bipedal", "walking", "gait",

        # Hardware
        "hardware", "setup", "configuration", "sensor", "camera",
        "microcontroller", "raspberry pi", "jetson", "nvidia",

        # Software/Frameworks
        "pytorch", "tensorflow", "machine learning", "deep learning",
        "reinforcement learning", "imitation learning", "vla",
        "vision-language", "llm", "language model",
        "simulation", "isaac sim", "gazebo", "unity", "unreal",

        # Chapters/sections from the book
        "introduction", "getting started", "foundation",
        "capstone", "project", "tutorial", "exercise",
        "digital twin", "workstation",

        # General robotics concepts
        "degree of freedom", "dof", "joint", "link",
        "coordinate frame", "transformation", "rotation",
        "urdf", "mesh", "collision", "visual",
        "workspace", "singularity", "jacobian",

        #general ai simple questions
        "artificial intelligence", "ai", "machine learning", "deep learning",
        "neural network", "model training", "dataset", "computer vision", "natural language processing",
        "reinforcement learning", "supervised learning", "unsupervised learning", "robot perception", "robot control",
        "robot navigation", "robot localization","book"
    ]

    # Disallowed patterns (obvious off-topic questions)
    DISALLOWED_PATTERNS = [
        r"\b(weather|recipe|movie|music|game|sport|politics|news)\b",
        r"\b(cook|bake|eat|restaurant|food)\b",
        r"\b(celebrity|actor|singer|artist)\b",
        r"\b(stock|invest|trading|crypto|bitcoin)\b",
        r"\b(health|medical|doctor|disease)\b",
        r"\b(travel|vacation|hotel|flight)\b",
        r"\b(hobby|fun|entertainment|leisure)\b",
        r"\b(personal|relationship|friend|family)\b",
        r"\b(joke|funny|humor|meme)\b",
        r"\b(art|design|painting|drawing|photography)\b",
        r"\b(sport|exercise|fitness|gym|workout)\b",    
        r"\b(science|technology|invention|you)\b",

    ]

    def __init__(self, min_relevance_score: float = 0.25):
        """
        Initialize guardrails

        Args:
            min_relevance_score: Minimum Qdrant relevance score to consider relevant
        """
        self.min_relevance_score = min_relevance_score
        logger.info(f"Initialized chatbot guardrails (min_relevance_score={min_relevance_score})")

    def is_book_related(self, question: str) -> Dict[str, any]:
        """
        Check if question is related to book content

        Args:
            question: User's question

        Returns:
            Dict with:
                - is_valid: bool
                - reason: str (if not valid)
                - matched_topics: List[str] (topics found in question)
        """
        question_lower = question.lower()

        # Check for disallowed patterns (obvious off-topic)
        for pattern in self.DISALLOWED_PATTERNS:
            if re.search(pattern, question_lower, re.IGNORECASE):
                logger.warning(f"Question blocked by disallowed pattern: {pattern}")
                return {
                    "is_valid": False,
                    "reason": "off_topic",
                    "matched_topics": [],
                    "message": "I can only answer questions about Physical AI and Humanoid Robotics. Please ask about topics covered in the book."
                }

        # Check for allowed topics
        matched_topics = []
        for topic in self.ALLOWED_TOPICS:
            if topic in question_lower:
                matched_topics.append(topic)

        # Allow question if it matches ANY topic
        if matched_topics:
            logger.info(f"Question validated: matched topics {matched_topics}")
            return {
                "is_valid": True,
                "reason": "topic_match",
                "matched_topics": matched_topics,
                "message": None
            }

        # If no keywords matched, allow it but mark for relevance check
        # (Will be validated by Qdrant retrieval scores)
        logger.info("Question has no keyword matches, will validate via retrieval scores")
        return {
            "is_valid": True,  # Tentatively allow
            "reason": "needs_retrieval_validation",
            "matched_topics": [],
            "message": None
        }

    def validate_retrieval_results(
        self,
        question: str,
        retrieved_chunks: List[Dict],
        validation_result: Dict
    ) -> Dict[str, any]:
        """
        Validate based on retrieval relevance scores

        Args:
            question: User's question
            retrieved_chunks: Chunks from Qdrant
            validation_result: Result from is_book_related()

        Returns:
            Dict with is_valid and message
        """
        # If already validated by keywords, skip relevance check
        if validation_result["reason"] == "topic_match":
            return {"is_valid": True, "message": None}

        # If no chunks retrieved or all scores too low
        if not retrieved_chunks:
            logger.warning(f"No relevant chunks found for question: {question[:50]}...")
            return {
                "is_valid": False,
                "message": "I couldn't find relevant information about that in the Physical AI and Humanoid Robotics documentation. Please ask about topics covered in the book."
            }

        # Check if best match score is above threshold
        best_score = max(chunk.get("score", 0) for chunk in retrieved_chunks)
        if best_score < self.min_relevance_score:
            logger.warning(f"Best relevance score ({best_score:.2%}) below threshold ({self.min_relevance_score:.2%})")
            return {
                "is_valid": False,
                "message": f"I couldn't find highly relevant information about that in the book. The best match had only {best_score:.0%} relevance. Please ask about Physical AI, Humanoid Robotics, ROS2, or related topics."
            }

        # Passed relevance threshold
        logger.info(f"Question validated by retrieval score: {best_score:.2%}")
        return {"is_valid": True, "message": None}


def get_guardrails(min_relevance_score: float = 0.25) -> ChatbotGuardrails:
    """Get guardrails instance"""
    return ChatbotGuardrails(min_relevance_score=min_relevance_score)
