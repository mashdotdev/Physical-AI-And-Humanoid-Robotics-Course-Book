"""FastAPI application entry point."""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app import __version__
from app.config import get_settings
from app.models import HealthResponse


settings = get_settings()

app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Physical AI textbook RAG chatbot",
    version=__version__,
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
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
