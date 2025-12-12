"""Chat API endpoint."""

from fastapi import APIRouter, HTTPException, Depends
from uuid import uuid4

from app.models import ChatRequest, ChatResponse, ErrorResponse
from app.models.auth import CurrentUser
from app.auth.dependencies import get_current_user
from app.rag import RAGChain

router = APIRouter()


@router.post(
    "/chat",
    response_model=ChatResponse,
    responses={
        400: {"model": ErrorResponse},
        401: {"model": ErrorResponse},
        500: {"model": ErrorResponse},
        503: {"model": ErrorResponse},
    },
)
async def chat(
    request: ChatRequest,
    current_user: CurrentUser = Depends(get_current_user),
) -> ChatResponse:
    """
    Process a chat message and return a RAG-generated response.

    Requires authentication - the user must be signed in via Google OAuth.

    The endpoint:
    1. Validates the user's session (via get_current_user dependency)
    2. Embeds the user's question
    3. Retrieves relevant document chunks from the vector store
    4. Generates a response using the LLM with the retrieved context
    5. Returns the response with source references
    """
    try:
        # Initialize RAG chain
        rag_chain = RAGChain()

        # Process the query
        response_text, sources = await rag_chain.process_query(request.message)

        # Generate conversation ID if not provided
        conversation_id = request.conversation_id or str(uuid4())

        return ChatResponse(
            response=response_text,
            sources=sources,
            conversation_id=conversation_id,
        )

    except ValueError as e:
        raise HTTPException(
            status_code=400,
            detail={"error": "validation_error", "message": str(e)},
        )
    except Exception as e:
        # Log the full error for debugging
        import traceback
        print(f"Chat error: {e}")
        traceback.print_exc()
        raise HTTPException(
            status_code=503,
            detail={
                "error": "service_unavailable",
                "message": f"Error: {str(e)}",
            },
        )
