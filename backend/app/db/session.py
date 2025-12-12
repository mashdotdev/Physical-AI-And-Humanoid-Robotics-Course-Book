"""Neon PostgreSQL async database connection pool."""

import asyncpg
from typing import Optional
from app.config import get_settings


class DatabasePool:
    """Singleton database pool manager."""

    _pool: Optional[asyncpg.Pool] = None

    @classmethod
    async def get_pool(cls) -> asyncpg.Pool:
        """Get or create the database connection pool."""
        if cls._pool is None:
            settings = get_settings()
            if not settings.database_url:
                raise ValueError("DATABASE_URL is not configured")

            cls._pool = await asyncpg.create_pool(
                dsn=settings.database_url,
                min_size=1,
                max_size=10,
                # SSL required for Neon
                ssl="require" if "neon.tech" in settings.database_url else None,
            )
        return cls._pool

    @classmethod
    async def close_pool(cls) -> None:
        """Close the database connection pool."""
        if cls._pool is not None:
            await cls._pool.close()
            cls._pool = None


async def get_db_pool() -> asyncpg.Pool:
    """Get the database connection pool."""
    return await DatabasePool.get_pool()


async def close_db_pool() -> None:
    """Close the database connection pool."""
    await DatabasePool.close_pool()
