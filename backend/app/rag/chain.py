"""RAG chain orchestration combining embeddings, retrieval, and generation."""

from typing import Optional
import google.generativeai as genai

from app.config import get_settings
from app.models import Source
from .embeddings import CohereEmbeddings
from .vectorstore import QdrantVectorStore


SYSTEM_PROMPT = """You are a helpful assistant for the Physical AI and Humanoid Robotics textbook.
Your role is to help readers understand the content of this book about robotics, ROS 2,
digital twins, AI integration, and Vision-Language-Action models.

IMPORTANT INSTRUCTIONS:
1. Answer questions based ONLY on the provided context from the book.
2. If the context doesn't contain relevant information to answer the question,
   say "I don't have information about that in the book" and suggest related topics.
3. Always cite which chapter/section the information comes from.
4. Be concise but thorough in your explanations.
5. If code examples are relevant, include them from the context.

The book covers these main topics:
- Module 1: The Robotic Nervous System (ROS 2)
- Module 2: The Digital Twin (Physics Simulation)
- Module 3: The AI-Robot Brain (Isaac Sim, Navigation)
- Module 4: Vision-Language-Action Models"""

OUT_OF_SCOPE_RESPONSE = """I'm designed to help you with the Physical AI and Humanoid Robotics textbook content.
I can answer questions about:

- **ROS 2**: Robot Operating System, nodes, topics, services, actions
- **Digital Twins**: Physics simulation, Gazebo, sensor modeling
- **AI Integration**: Isaac Sim, VSLAM, Nav2 navigation
- **VLA Models**: Voice commands, cognitive planning, computer vision

Please ask a question about one of these topics!"""


class RAGChain:
    """Orchestrates the RAG pipeline: embed query → retrieve context → generate response."""

    def __init__(
        self,
        embeddings: Optional[CohereEmbeddings] = None,
        vectorstore: Optional[QdrantVectorStore] = None,
        gemini_api_key: Optional[str] = None,
    ):
        """Initialize the RAG chain components."""
        settings = get_settings()
        self.embeddings = embeddings or CohereEmbeddings()
        self.vectorstore = vectorstore or QdrantVectorStore()

        # Configure Gemini
        genai.configure(api_key=gemini_api_key or settings.gemini_api_key)
        self.model = genai.GenerativeModel(settings.gemini_model)
        self.top_k = settings.top_k
        self.relevance_threshold = settings.relevance_threshold

    def _format_context(self, results: list[dict]) -> str:
        """Format retrieved results into context string for the LLM."""
        if not results:
            return ""

        context_parts = []
        for r in results:
            source_info = f"[{r['chapter']} - {r['section']}]"
            context_parts.append(f"{source_info}\n{r['text']}")

        return "\n\n---\n\n".join(context_parts)

    def _extract_sources(self, results: list[dict]) -> list[Source]:
        """Extract source references from retrieval results."""
        return [
            Source(
                chapter=r["chapter"],
                section=r["section"],
                file_path=r["file_path"],
                relevance=r["relevance"],
            )
            for r in results
        ]

    def _is_out_of_scope(self, results: list[dict]) -> bool:
        """Check if the query is out of scope based on retrieval results."""
        if not results:
            return True
        # If best result is below threshold, consider out of scope
        return results[0]["relevance"] < self.relevance_threshold

    async def process_query(self, query: str) -> tuple[str, list[Source]]:
        """
        Process a user query through the RAG pipeline.

        Args:
            query: The user's question

        Returns:
            Tuple of (response_text, list_of_sources)
        """
        # Step 1: Embed the query
        query_embedding = self.embeddings.embed_query(query)

        # Step 2: Retrieve relevant documents
        results = self.vectorstore.search(
            query_embedding=query_embedding,
            top_k=self.top_k,
            score_threshold=0.0,  # Get results even if low score, we'll check threshold
        )

        # Step 3: Check if out of scope
        if self._is_out_of_scope(results):
            return OUT_OF_SCOPE_RESPONSE, []

        # Step 4: Format context and generate response
        context = self._format_context(results)
        sources = self._extract_sources(results)

        # Build the prompt for Gemini
        prompt = f"""{SYSTEM_PROMPT}

Context from the book:
{context}

Question: {query}

Answer based on the context above:"""

        response = self.model.generate_content(
            prompt,
            generation_config=genai.types.GenerationConfig(
                temperature=0.3,
                max_output_tokens=500,
            ),
        )

        answer = response.text
        return answer, sources
