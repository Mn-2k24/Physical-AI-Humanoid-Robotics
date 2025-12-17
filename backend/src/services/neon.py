"""
Neon Postgres Connection Pool
Purpose: Async connection pool with psycopg for Neon Serverless Postgres
Date: 2025-12-13
"""

import logging
from contextlib import asynccontextmanager
from typing import AsyncGenerator

import psycopg
from psycopg_pool import AsyncConnectionPool

from ..core.config import settings

logger = logging.getLogger(__name__)


class NeonDatabase:
    """Neon Postgres database connection manager."""

    def __init__(self):
        """Initialize connection pool."""
        self.pool: Optional[AsyncConnectionPool] = None

    async def connect(self):
        """Create async connection pool."""
        try:
            self.pool = AsyncConnectionPool(
                conninfo=settings.neon_connection_string,
                min_size=2,
                max_size=10,
                timeout=30,
                max_lifetime=3600,  # Recycle connections every hour
                max_idle=600,  # Close idle connections after 10min
            )
            await self.pool.wait()
            logger.info("✓ Neon Postgres connection pool initialized")
        except Exception as e:
            logger.error(f"✗ Failed to connect to Neon Postgres: {e}")
            raise

    async def disconnect(self):
        """Close connection pool."""
        if self.pool:
            await self.pool.close()
            logger.info("✓ Neon Postgres connection pool closed")

    @asynccontextmanager
    async def get_connection(self) -> AsyncGenerator[psycopg.AsyncConnection, None]:
        """
        Get a database connection from the pool.

        Usage:
            async with db.get_connection() as conn:
                async with conn.cursor() as cursor:
                    await cursor.execute("SELECT * FROM users")
                    results = await cursor.fetchall()

        Yields:
            Async database connection
        """
        if not self.pool:
            raise RuntimeError("Database pool not initialized. Call connect() first.")

        async with self.pool.connection() as conn:
            yield conn

    async def execute_query(self, query: str, params: tuple = None):
        """
        Execute a query and return results.

        Args:
            query: SQL query string
            params: Query parameters tuple

        Returns:
            Query results as list of tuples
        """
        async with self.get_connection() as conn:
            async with conn.cursor() as cursor:
                await cursor.execute(query, params)
                return await cursor.fetchall()

    async def execute_one(self, query: str, params: tuple = None):
        """
        Execute a query and return single result.

        Args:
            query: SQL query string
            params: Query parameters tuple

        Returns:
            Single query result or None
        """
        async with self.get_connection() as conn:
            async with conn.cursor() as cursor:
                await cursor.execute(query, params)
                return await cursor.fetchone()

    async def execute_write(self, query: str, params: tuple = None):
        """
        Execute a write query (INSERT, UPDATE, DELETE).

        Args:
            query: SQL query string
            params: Query parameters tuple

        Returns:
            Number of affected rows
        """
        async with self.get_connection() as conn:
            async with conn.cursor() as cursor:
                await cursor.execute(query, params)
                await conn.commit()
                return cursor.rowcount

    async def health_check(self) -> bool:
        """
        Check database connection health.

        Returns:
            True if database is reachable, False otherwise
        """
        try:
            result = await self.execute_one("SELECT 1")
            return result is not None
        except Exception as e:
            logger.error(f"Database health check failed: {e}")
            return False


# Global database instance
db = NeonDatabase()


# Dependency for FastAPI
async def get_db():
    """FastAPI dependency for database access."""
    return db
