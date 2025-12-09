"""
FastAPI Backend for RAG Chatbot
Provides REST API endpoints for chat functionality
"""

import os
import logging
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
from contextlib import asynccontextmanager

# Import app modules
from app.rag import get_rag_pipeline
from app.postgres import get_postgres_manager
from app.utils import setup_logging

# Setup logging
setup_logging(log_level=os.getenv("LOG_LEVEL", "INFO"))
logger = logging.getLogger(__name__)


# Lifespan context manager for startup/shutdown
@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for FastAPI app
    Handles startup and shutdown events
    """
    # Startup
    logger.info("Starting up RAG chatbot backend...")

    # Initialize database connection
    postgres = get_postgres_manager()
    await postgres.connect()
    await postgres.initialize_schema()

    logger.info("Backend startup complete!")

    yield

    # Shutdown
    logger.info("Shutting down...")
    await postgres.disconnect()
    logger.info("Shutdown complete")


# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Backend API for Retrieval-Augmented Generation chatbot",
    version="1.0.0",
    lifespan=lifespan
)

# CORS configuration
cors_origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Request models
class AskRequest(BaseModel):
    question: str
    stream: bool = False


class AskSelectedRequest(BaseModel):
    selected_text: str
    question: str
    stream: bool = False


# API Endpoints
@app.get("/health")
async def health_check():
    """Health check endpoint"""
    postgres = get_postgres_manager()
    db_connected = postgres.pool is not None

    return {
        "status": "ok",
        "message": "RAG chatbot is running",
        "database_connected": db_connected
    }


@app.post("/ask")
async def ask_question(request: AskRequest):
    """
    Ask a question using RAG (retrieval from Qdrant + generation)

    Args:
        request: Question and stream flag

    Returns:
        Answer with sources and metadata
    """
    try:
        rag_pipeline = get_rag_pipeline()

        if request.stream:
            # TODO: Implement streaming response
            raise HTTPException(status_code=501, detail="Streaming not yet implemented")

        # Non-streaming response
        result = await rag_pipeline.answer_question(
            question=request.question,
            log_to_db=True
        )

        return result

    except Exception as e:
        logger.error(f"Error in /ask endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/ask-selected")
async def ask_about_selection(request: AskSelectedRequest):
    """
    Ask a question about selected text (no retrieval, direct generation)

    Args:
        request: Selected text, question, and stream flag

    Returns:
        Answer with metadata
    """
    try:
        rag_pipeline = get_rag_pipeline()

        if request.stream:
            # TODO: Implement streaming response
            raise HTTPException(status_code=501, detail="Streaming not yet implemented")

        # Non-streaming response
        result = await rag_pipeline.answer_with_selected_text(
            selected_text=request.selected_text,
            question=request.question,
            log_to_db=True
        )

        return result

    except Exception as e:
        logger.error(f"Error in /ask-selected endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/chat-logs")
async def get_chat_logs(limit: int = 10):
    """
    Get recent chat logs

    Args:
        limit: Number of logs to retrieve

    Returns:
        List of chat logs
    """
    try:
        postgres = get_postgres_manager()
        logs = await postgres.get_recent_chats(limit=limit)
        return {"logs": logs, "count": len(logs)}

    except Exception as e:
        logger.error(f"Error in /chat-logs endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/stats")
async def get_stats():
    """
    Get chatbot statistics

    Returns:
        Statistics about usage
    """
    try:
        postgres = get_postgres_manager()
        stats = await postgres.get_stats()
        return stats

    except Exception as e:
        logger.error(f"Error in /stats endpoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# Run with: python main.py
if __name__ == "__main__":
    import uvicorn

    port = int(os.getenv("PORT", 8000))
    host = os.getenv("HOST", "0.0.0.0")

    logger.info(f"Starting server on {host}:{port}")

    uvicorn.run(
        "main:app",
        host=host,
        port=port,
        reload=True,
        log_level="info"
    )
