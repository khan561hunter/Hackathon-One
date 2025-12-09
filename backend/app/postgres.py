"""
Neon Serverless Postgres Database Connection and Operations
Stores chat logs and selected text interactions
"""

import os
from typing import Optional, List, Dict, Any
import asyncpg
from datetime import datetime
import logging

logger = logging.getLogger(__name__)


class PostgresManager:
    """
    Neon Postgres manager for chat logs and metadata
    """

    def __init__(self, connection_url: Optional[str] = None):
        """
        Initialize Postgres manager

        Args:
            connection_url: Postgres connection URL (from NEON_DB_URL if None)
        """
        self.connection_url = connection_url or os.getenv("NEON_DB_URL")
        if not self.connection_url:
            raise ValueError("NEON_DB_URL not found in environment variables")

        self.pool: Optional[asyncpg.Pool] = None
        logger.info("Initialized Postgres manager")

    async def connect(self):
        """
        Create connection pool
        """
        try:
            self.pool = await asyncpg.create_pool(
                self.connection_url,
                min_size=1,
                max_size=10,
                command_timeout=60
            )
            logger.info("Connected to Postgres database")
        except Exception as e:
            logger.error(f"Failed to connect to Postgres: {e}")
            raise

    async def disconnect(self):
        """
        Close connection pool
        """
        if self.pool:
            await self.pool.close()
            logger.info("Disconnected from Postgres database")

    async def initialize_schema(self):
        """
        Create tables if they don't exist
        """
        create_chat_logs_table = """
        CREATE TABLE IF NOT EXISTS chat_logs (
            id SERIAL PRIMARY KEY,
            session_id VARCHAR(255),
            user_query TEXT NOT NULL,
            bot_response TEXT NOT NULL,
            retrieved_docs JSON,
            model_used VARCHAR(100),
            response_time DOUBLE PRECISION,
            created_at TIMESTAMP DEFAULT NOW()
        );
        """

        create_selections_table = """
        CREATE TABLE IF NOT EXISTS selections (
            id SERIAL PRIMARY KEY,
            selected_text TEXT NOT NULL,
            question TEXT NOT NULL,
            answer TEXT NOT NULL,
            created_at TIMESTAMP DEFAULT NOW()
        );
        """

        create_indexes = """
        CREATE INDEX IF NOT EXISTS idx_chat_logs_created_at ON chat_logs(created_at DESC);
        CREATE INDEX IF NOT EXISTS idx_selections_created_at ON selections(created_at DESC);
        """

        try:
            async with self.pool.acquire() as conn:
                await conn.execute(create_chat_logs_table)
                await conn.execute(create_selections_table)
                await conn.execute(create_indexes)
                logger.info("Database schema initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize schema: {e}")
            raise

    async def log_chat_interaction(
        self,
        question: str,
        answer: str,
        context_used: Optional[str] = None,
        retrieved_docs: Optional[List[Dict]] = None,
        model_used: str = "qwen-plus",
        response_time: Optional[float] = None
    ) -> int:
        """
        Log a chat interaction to database

        Args:
            question: User's question
            answer: Generated answer
            context_used: Context chunks used (optional, for backwards compatibility)
            retrieved_docs: Retrieved document metadata as JSON
            model_used: Model used for generation
            response_time: Response time in seconds

        Returns:
            ID of inserted record
        """
        import json

        # Convert retrieved_docs to JSON if provided
        docs_json = json.dumps(retrieved_docs) if retrieved_docs else None

        query = """
        INSERT INTO chat_logs (user_query, bot_response, retrieved_docs, model_used, response_time)
        VALUES ($1, $2, $3, $4, $5)
        RETURNING id;
        """

        try:
            async with self.pool.acquire() as conn:
                record_id = await conn.fetchval(
                    query,
                    question,
                    answer,
                    docs_json,
                    model_used,
                    response_time
                )
                logger.info(f"Logged chat interaction (ID: {record_id})")
                return record_id
        except Exception as e:
            logger.error(f"Failed to log chat interaction: {e}")
            raise

    async def log_selection_interaction(
        self,
        selected_text: str,
        question: str,
        answer: str
    ) -> int:
        """
        Log a selected text interaction to database

        Args:
            selected_text: Text selected by user
            question: User's question
            answer: Generated answer

        Returns:
            ID of inserted record
        """
        query = """
        INSERT INTO selections (selected_text, question, answer)
        VALUES ($1, $2, $3)
        RETURNING id;
        """

        try:
            async with self.pool.acquire() as conn:
                record_id = await conn.fetchval(query, selected_text, question, answer)
                logger.info(f"Logged selection interaction (ID: {record_id})")
                return record_id
        except Exception as e:
            logger.error(f"Failed to log selection interaction: {e}")
            raise

    async def get_recent_chats(self, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Get recent chat interactions

        Args:
            limit: Number of records to retrieve

        Returns:
            List of chat records
        """
        query = """
        SELECT id, user_query, bot_response, retrieved_docs, model_used, response_time, created_at
        FROM chat_logs
        ORDER BY created_at DESC
        LIMIT $1;
        """

        try:
            async with self.pool.acquire() as conn:
                records = await conn.fetch(query, limit)
                return [dict(record) for record in records]
        except Exception as e:
            logger.error(f"Failed to get recent chats: {e}")
            raise

    async def get_recent_selections(self, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Get recent selection interactions

        Args:
            limit: Number of records to retrieve

        Returns:
            List of selection records
        """
        query = """
        SELECT id, selected_text, question, answer, created_at
        FROM selections
        ORDER BY created_at DESC
        LIMIT $1;
        """

        try:
            async with self.pool.acquire() as conn:
                records = await conn.fetch(query, limit)
                return [dict(record) for record in records]
        except Exception as e:
            logger.error(f"Failed to get recent selections: {e}")
            raise

    async def search_chats(
        self,
        search_term: str,
        limit: int = 20
    ) -> List[Dict[str, Any]]:
        """
        Search chat logs by question or answer

        Args:
            search_term: Term to search for
            limit: Maximum results

        Returns:
            List of matching chat records
        """
        query = """
        SELECT id, user_query, bot_response, retrieved_docs, model_used, response_time, created_at
        FROM chat_logs
        WHERE user_query ILIKE $1 OR bot_response ILIKE $1
        ORDER BY created_at DESC
        LIMIT $2;
        """

        try:
            async with self.pool.acquire() as conn:
                records = await conn.fetch(query, f"%{search_term}%", limit)
                return [dict(record) for record in records]
        except Exception as e:
            logger.error(f"Failed to search chats: {e}")
            raise

    async def get_stats(self) -> Dict[str, int]:
        """
        Get database statistics

        Returns:
            Dict with chat and selection counts
        """
        query_chats = "SELECT COUNT(*) FROM chat_logs;"
        query_selections = "SELECT COUNT(*) FROM selections;"

        try:
            async with self.pool.acquire() as conn:
                chat_count = await conn.fetchval(query_chats)
                selection_count = await conn.fetchval(query_selections)

                return {
                    "total_chats": chat_count,
                    "total_selections": selection_count
                }
        except Exception as e:
            logger.error(f"Failed to get stats: {e}")
            raise


# Global instance (lazy initialization)
_postgres_manager: Optional[PostgresManager] = None


def get_postgres_manager() -> PostgresManager:
    """
    Get or create global Postgres manager instance

    Returns:
        PostgresManager instance
    """
    global _postgres_manager
    if _postgres_manager is None:
        _postgres_manager = PostgresManager()
    return _postgres_manager
