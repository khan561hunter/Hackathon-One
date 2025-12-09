"""
Utility functions for the RAG chatbot backend
"""

import os
import logging
from pathlib import Path
from typing import List, Dict, Any


def setup_logging(log_level: str = "INFO"):
    """
    Setup logging configuration

    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR)
    """
    logging.basicConfig(
        level=getattr(logging, log_level.upper()),
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )


def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """
    Split text into overlapping chunks

    Args:
        text: Text to chunk
        chunk_size: Size of each chunk in characters
        overlap: Overlap between chunks

    Returns:
        List of text chunks
    """
    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]

        # Try to break at sentence boundary
        if end < len(text):
            last_period = chunk.rfind('.')
            last_newline = chunk.rfind('\n')
            break_point = max(last_period, last_newline)

            if break_point > chunk_size // 2:
                chunk = chunk[:break_point + 1]
                end = start + break_point + 1

        chunks.append(chunk.strip())
        start = end - overlap

    return chunks


def load_markdown_files(docs_dir: str = "docs") -> List[Dict[str, Any]]:
    """
    Load all markdown files from docs directory

    Args:
        docs_dir: Directory containing markdown files

    Returns:
        List of dicts with file name and content
    """
    docs_path = Path(docs_dir)
    if not docs_path.exists():
        raise FileNotFoundError(f"Docs directory not found: {docs_dir}")

    markdown_files = []

    for md_file in docs_path.rglob("*.md"):
        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()
                markdown_files.append({
                    "name": md_file.name,
                    "path": str(md_file),
                    "content": content
                })
        except Exception as e:
            logging.error(f"Failed to read {md_file}: {e}")

    return markdown_files


def validate_env_vars(required_vars: List[str]):
    """
    Validate that required environment variables are set

    Args:
        required_vars: List of required environment variable names

    Raises:
        ValueError: If any required variables are missing
    """
    missing_vars = []

    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        raise ValueError(f"Missing required environment variables: {', '.join(missing_vars)}")
