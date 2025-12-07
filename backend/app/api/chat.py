"""Chat API endpoint."""

from fastapi import APIRouter, HTTPException
from uuid import uuid4

from app.models import ChatRequest, ChatResponse, ErrorResponse
from app.rag import RAGChain

router = APIRouter()


@router.post(
    "/chat",
    response_model=ChatResponse,
    responses={
        400: {"model": ErrorResponse},
        500: {"model": ErrorResponse},
        503: {"model": ErrorResponse},
    },
)
async def chat(request: ChatRequest) -> ChatResponse:
    """
    Process a chat message and return a RAG-generated response.

    The endpoint:
    1. Embeds the user's question
    2. Retrieves relevant document chunks from the vector store
    3. Generates a response using the LLM with the retrieved context
    4. Returns the response with source references
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
