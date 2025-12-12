"""FastAPI application entry point."""

from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app import __version__
from app.config import get_settings
from app.models import HealthResponse
from app.db.session import get_db_pool, close_db_pool


settings = get_settings()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan events for startup and shutdown."""
    # Startup: Initialize database pool if DATABASE_URL is configured
    if settings.database_url:
        try:
            await get_db_pool()
            print("Database pool initialized")
        except Exception as e:
            print(f"Warning: Failed to initialize database pool: {e}")

    yield

    # Shutdown: Close database pool
    await close_db_pool()
    print("Database pool closed")


app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Physical AI textbook RAG chatbot",
    version=__version__,
    lifespan=lifespan,
)

# Configure CORS - include auth subdomain for cross-origin cookie sharing
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,  # Required for cookies
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health", response_model=HealthResponse)
async def health_check() -> HealthResponse:
    """Health check endpoint."""
    return HealthResponse(status="healthy", version=__version__)


# Import and include routers after app is created to avoid circular imports
from app.api import chat_router  # noqa: E402

app.include_router(chat_router, prefix="/api")
