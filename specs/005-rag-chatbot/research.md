# Research Document: RAG Chatbot for Documentation

**Feature**: 005-rag-chatbot
**Date**: 2025-12-07
**Purpose**: Resolve technical decisions and document best practices

## Research Areas

### 1. Cohere Embeddings Integration

**Decision**: Use Cohere `embed-english-v3.0` model via Python SDK

**Rationale**:
- Free tier provides 1000 API calls/month (sufficient for initial deployment)
- 1024-dimensional embeddings with strong semantic understanding
- `input_type` parameter optimizes for search queries vs documents
- Simple Python SDK: `cohere.Client(api_key).embed()`

**Alternatives Considered**:
| Option | Pros | Cons | Why Rejected |
|--------|------|------|--------------|
| OpenAI ada-002 | High quality, same vendor as LLM | Paid only ($0.0001/1K tokens) | Cost constraint |
| Sentence Transformers (local) | Free, offline | Requires GPU, slower | Infrastructure complexity |
| Voyage AI | High quality | Limited free tier | Less documentation |

**Implementation Pattern**:
```python
import cohere
co = cohere.Client(api_key=COHERE_API_KEY)

# For documents (indexing)
embeddings = co.embed(
    texts=chunks,
    model="embed-english-v3.0",
    input_type="search_document"
).embeddings

# For queries (search)
query_embedding = co.embed(
    texts=[query],
    model="embed-english-v3.0",
    input_type="search_query"
).embeddings[0]
```

**Key Findings**:
- Must use `input_type="search_document"` for indexing, `"search_query"` for queries
- Batch embedding supported (up to 96 texts per call)
- Rate limit: 100 calls/minute on free tier

---

### 2. Qdrant Vector Database Setup

**Decision**: Use Qdrant Cloud with Python client

**Rationale**:
- 1GB free tier forever (no credit card required)
- Native Python client with async support
- Built-in filtering on metadata fields
- REST API fallback for debugging

**Alternatives Considered**:
| Option | Pros | Cons | Why Rejected |
|--------|------|------|--------------|
| Pinecone | Popular, managed | Free tier: 1 index, 100K vectors, deletes after inactivity | Limitations |
| Chroma | Simple, local | No managed hosting, persistence issues | Deployment complexity |
| Weaviate | Feature-rich | Steeper learning curve | Overkill for this scale |

**Implementation Pattern**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Create collection
client.create_collection(
    collection_name="book_docs",
    vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
)

# Upsert points
client.upsert(
    collection_name="book_docs",
    points=[
        PointStruct(
            id=idx,
            vector=embedding,
            payload={"text": chunk, "source": filepath, "chapter": chapter}
        )
        for idx, (embedding, chunk, filepath, chapter) in enumerate(data)
    ]
)

# Search
results = client.search(
    collection_name="book_docs",
    query_vector=query_embedding,
    limit=5
)
```

**Key Findings**:
- Use COSINE distance for text embeddings (normalized)
- Payload filtering: `Filter(must=[FieldCondition(...)])`
- Free tier: 1GB storage, ~100K vectors with 1024 dimensions

---

### 3. OpenAI Chat API for RAG Responses

**Decision**: Use `gpt-4o-mini` with system prompt for RAG

**Rationale**:
- Best cost/quality ratio ($0.15/1M input, $0.60/1M output)
- Strong instruction following for grounded responses
- 128K context window (plenty for RAG context)
- Streaming support for better UX

**Implementation Pattern**:
```python
from openai import OpenAI
client = OpenAI(api_key=OPENAI_API_KEY)

system_prompt = """You are a helpful assistant for the Physical AI and Humanoid Robotics textbook.
Answer questions based ONLY on the provided context. If the context doesn't contain
the answer, say "I don't have information about that in the book."

Always cite which chapter/section the information comes from."""

def generate_response(query: str, context_chunks: list[dict]) -> str:
    context = "\n\n".join([
        f"[{c['chapter']}]\n{c['text']}"
        for c in context_chunks
    ])

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {query}"}
        ],
        temperature=0.3,  # Lower for factual responses
        max_tokens=500
    )
    return response.choices[0].message.content
```

**Key Findings**:
- Temperature 0.3 balances accuracy with natural language
- Include source references in system prompt instruction
- Stream responses for better perceived latency

---

### 4. Markdown Chunking Strategy

**Decision**: Markdown-aware recursive chunking with ~500 tokens, 50 token overlap

**Rationale**:
- Preserves semantic boundaries (headers, code blocks, lists)
- 500 tokens balances context richness vs retrieval precision
- Overlap ensures no information loss at boundaries

**Implementation Pattern**:
```python
from langchain.text_splitter import RecursiveCharacterTextSplitter, Language

splitter = RecursiveCharacterTextSplitter.from_language(
    language=Language.MARKDOWN,
    chunk_size=2000,      # ~500 tokens
    chunk_overlap=200,    # ~50 tokens
    separators=[
        "\n## ",          # H2 headers first
        "\n### ",         # H3 headers
        "\n#### ",        # H4 headers
        "\n```",          # Code blocks
        "\n\n",           # Paragraphs
        "\n",             # Lines
        " ",              # Words
    ]
)
```

**Key Findings**:
- Use 4:1 character-to-token ratio as estimate
- Extract chapter/section from headers before chunking
- Preserve code blocks intact when possible

---

### 5. FastAPI Backend Structure

**Decision**: Async FastAPI with dependency injection

**Rationale**:
- Async for concurrent API calls (Cohere, Qdrant, OpenAI)
- Dependency injection for testability
- Built-in OpenAPI docs
- CORS middleware for Docusaurus frontend

**Implementation Pattern**:
```python
from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI(title="RAG Chatbot API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://your-docusaurus-site.com", "http://localhost:3000"],
    allow_methods=["POST", "GET"],
    allow_headers=["*"],
)

# Dependency injection
def get_rag_service():
    return RAGService(
        embeddings=CohereEmbeddings(),
        vectorstore=QdrantVectorStore(),
        llm=OpenAIChat()
    )

@app.post("/api/chat")
async def chat(request: ChatRequest, rag: RAGService = Depends(get_rag_service)):
    return await rag.process_query(request.message)
```

**Key Findings**:
- Use `async` for all external API calls
- Environment-based CORS configuration
- Health check endpoint for monitoring

---

### 6. React Chat Widget Integration

**Decision**: Docusaurus swizzling to inject global component

**Rationale**:
- Appears on all pages without modifying each page
- Uses Docusaurus theme system
- Can access Docusaurus context (current page, theme)

**Implementation Pattern**:
```tsx
// book/src/theme/Root.tsx (swizzled)
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
```

**Key Findings**:
- Swizzle `Root` component for global injection
- Use CSS modules for scoped styling
- z-index: 9999 for overlay behavior
- Position: fixed, bottom: 20px, right: 20px

---

## Resolved Clarifications

| Original Unknown | Resolution | Source |
|------------------|------------|--------|
| Embedding model | Cohere embed-english-v3.0 | User preference + free tier |
| Vector DB | Qdrant Cloud | User preference + free tier |
| LLM model | gpt-4o-mini | Cost optimization |
| Chunk size | 500 tokens | Industry best practice |
| Frontend integration | Docusaurus swizzling | Framework documentation |

## External References

- [Cohere Embed API](https://docs.cohere.com/reference/embed)
- [Qdrant Python Client](https://qdrant.tech/documentation/quick-start/)
- [OpenAI Chat Completions](https://platform.openai.com/docs/api-reference/chat)
- [Docusaurus Swizzling](https://docusaurus.io/docs/swizzling)
- [LangChain Text Splitters](https://python.langchain.com/docs/modules/data_connection/document_transformers/)
