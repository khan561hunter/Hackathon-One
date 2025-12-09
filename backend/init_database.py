"""
Database initialization script
Creates necessary tables in Neon Postgres
"""

import asyncio
import logging
from app.postgres import get_postgres_manager
from app.utils import setup_logging

setup_logging()
logger = logging.getLogger(__name__)


async def main():
    """Initialize database schema"""
    logger.info("Initializing database...")

    try:
        # Get Postgres manager
        postgres = get_postgres_manager()

        # Connect
        await postgres.connect()
        logger.info("Connected to Postgres")

        # Initialize schema
        await postgres.initialize_schema()
        logger.info("Schema initialized successfully")

        # Get stats
        stats = await postgres.get_stats()
        logger.info(f"Database stats: {stats}")

        # Disconnect
        await postgres.disconnect()
        logger.info("Database initialization complete!")

    except Exception as e:
        logger.error(f"Database initialization failed: {e}")
        raise


if __name__ == "__main__":
    asyncio.run(main())
