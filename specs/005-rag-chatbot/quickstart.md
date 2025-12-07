# Quickstart: RAG Chatbot for Documentation

**Feature**: 005-rag-chatbot
**Time to complete**: ~30 minutes

## Prerequisites

- Python 3.11+
- Node.js 20+
- API Keys:
  - [Cohere API Key](https://dashboard.cohere.com/api-keys) (free)
  - [Qdrant Cloud](https://cloud.qdrant.io/) account (free)
  - [OpenAI API Key](https://platform.openai.com/api-keys) (paid)

## Step 1: Backend Setup

### 1.1 Create Backend Directory

```bash
mkdir -p backend/app/{rag,api,models}
mkdir -p backend/{scripts,tests}
cd backend
```

### 1.2 Create Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 1.3 Install Dependencies

```bash
# Create requirements.txt
cat > requirements.txt << 'EOF'
fastapi==0.109.0
uvicorn[standard]==0.27.0
cohere==4.45
qdrant-client==1.7.0
openai==1.12.0
python-dotenv==1.0.0
pydantic==2.6.0
httpx==0.26.0
pytest==8.0.0
pytest-asyncio==0.23.0
EOF

pip install -r requirements.txt
```

### 1.4 Configure Environment

```bash
# Create .env file
cat > .env << 'EOF'
COHERE_API_KEY=your_cohere_key_here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_key_here
OPENAI_API_KEY=your_openai_key_here
CORS_ORIGINS=http://localhost:3000,http://localhost:3001
EOF
```

### 1.5 Create Main Application

```python
# backend/app/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os

load_dotenv()

app = FastAPI(title="RAG Chatbot API", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=os.getenv("CORS_ORIGINS", "").split(","),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/api/health")
async def health():
    return {"status": "healthy", "version": "1.0.0"}

# Import and include chat router
from app.api.chat import router as chat_router
app.include_router(chat_router, prefix="/api")
```

### 1.6 Run Backend

```bash
uvicorn app.main:app --reload --port 8000
```

Verify: Open http://localhost:8000/api/health

## Step 2: Qdrant Setup

### 2.1 Create Qdrant Cloud Cluster

1. Go to [Qdrant Cloud](https://cloud.qdrant.io/)
2. Create a free cluster
3. Copy the URL and API key to `.env`

### 2.2 Create Collection

```python
# backend/scripts/create_collection.py
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
from dotenv import load_dotenv
import os

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

client.create_collection(
    collection_name="book_docs",
    vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
)

print("Collection 'book_docs' created successfully!")
```

Run: `python scripts/create_collection.py`

## Step 3: Ingest Documents

### 3.1 Create Ingestion Script

```python
# backend/scripts/ingest.py
import os
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
from dotenv import load_dotenv
import glob
import re
from uuid import uuid4

load_dotenv()

# Initialize clients
co = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

def chunk_markdown(text: str, max_chars: int = 2000) -> list[str]:
    """Simple markdown-aware chunking."""
    chunks = []
    current = ""
    for line in text.split("\n"):
        if len(current) + len(line) > max_chars and current:
            chunks.append(current.strip())
            current = ""
        current += line + "\n"
    if current.strip():
        chunks.append(current.strip())
    return chunks

def extract_metadata(filepath: str, content: str) -> dict:
    """Extract chapter and section from file."""
    chapter = filepath.split("/")[0] if "/" in filepath else "General"
    section_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
    section = section_match.group(1) if section_match else "Introduction"
    return {"chapter": chapter, "section": section, "file_path": filepath}

def ingest_docs(docs_path: str = "../../book/docs"):
    """Ingest all markdown files."""
    md_files = glob.glob(f"{docs_path}/**/*.md", recursive=True)

    all_chunks = []
    for filepath in md_files:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        rel_path = os.path.relpath(filepath, docs_path)
        metadata = extract_metadata(rel_path, content)

        chunks = chunk_markdown(content)
        for i, chunk in enumerate(chunks):
            all_chunks.append({
                "text": chunk,
                "metadata": {**metadata, "position": i}
            })

    print(f"Found {len(all_chunks)} chunks from {len(md_files)} files")

    # Embed in batches
    batch_size = 96
    points = []
    for i in range(0, len(all_chunks), batch_size):
        batch = all_chunks[i:i+batch_size]
        texts = [c["text"] for c in batch]

        embeddings = co.embed(
            texts=texts,
            model="embed-english-v3.0",
            input_type="search_document"
        ).embeddings

        for j, (chunk, embedding) in enumerate(zip(batch, embeddings)):
            points.append(PointStruct(
                id=str(uuid4()),
                vector=embedding,
                payload={"text": chunk["text"], **chunk["metadata"]}
            ))

        print(f"Embedded {min(i+batch_size, len(all_chunks))}/{len(all_chunks)}")

    # Upload to Qdrant
    qdrant.upsert(collection_name="book_docs", points=points)
    print(f"Uploaded {len(points)} vectors to Qdrant!")

if __name__ == "__main__":
    ingest_docs()
```

Run: `python scripts/ingest.py`

## Step 4: Frontend Widget

### 4.1 Create Chat Widget

```bash
cd ../book
mkdir -p src/components/ChatWidget
mkdir -p src/theme
```

### 4.2 Create Root Component (Swizzle)

```tsx
// book/src/theme/Root.tsx
import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({children}: {children: React.ReactNode}) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
```

### 4.3 Create Chat Widget Component

```tsx
// book/src/components/ChatWidget/index.tsx
import React, { useState } from 'react';
import styles from './styles.module.css';

const API_URL = 'http://localhost:8000';

interface Message {
  role: 'user' | 'assistant';
  content: string;
}

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);

  const sendMessage = async () => {
    if (!input.trim() || loading) return;

    const userMessage = input.trim();
    setInput('');
    setMessages(prev => [...prev, { role: 'user', content: userMessage }]);
    setLoading(true);

    try {
      const res = await fetch(`${API_URL}/api/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ message: userMessage }),
      });
      const data = await res.json();
      setMessages(prev => [...prev, { role: 'assistant', content: data.response }]);
    } catch (error) {
      setMessages(prev => [...prev, { role: 'assistant', content: 'Sorry, something went wrong.' }]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.container}>
      {isOpen ? (
        <div className={styles.panel}>
          <div className={styles.header}>
            <span>Ask about the book</span>
            <button onClick={() => setIsOpen(false)}>×</button>
          </div>
          <div className={styles.messages}>
            {messages.map((m, i) => (
              <div key={i} className={styles[m.role]}>{m.content}</div>
            ))}
            {loading && <div className={styles.loading}>Thinking...</div>}
          </div>
          <div className={styles.inputArea}>
            <input
              value={input}
              onChange={e => setInput(e.target.value)}
              onKeyDown={e => e.key === 'Enter' && sendMessage()}
              placeholder="Ask a question..."
            />
            <button onClick={sendMessage} disabled={loading}>Send</button>
          </div>
        </div>
      ) : (
        <button className={styles.bubble} onClick={() => setIsOpen(true)}>
          💬
        </button>
      )}
    </div>
  );
}
```

### 4.4 Add Styles

```css
/* book/src/components/ChatWidget/styles.module.css */
.container {
  position: fixed;
  bottom: 20px;
  right: 20px;
  z-index: 9999;
}

.bubble {
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: #3b82f6;
  border: none;
  font-size: 24px;
  cursor: pointer;
  box-shadow: 0 4px 12px rgba(0,0,0,0.15);
}

.panel {
  width: 380px;
  height: 500px;
  background: white;
  border-radius: 12px;
  box-shadow: 0 4px 24px rgba(0,0,0,0.2);
  display: flex;
  flex-direction: column;
}

.header {
  padding: 16px;
  background: #3b82f6;
  color: white;
  border-radius: 12px 12px 0 0;
  display: flex;
  justify-content: space-between;
}

.header button {
  background: none;
  border: none;
  color: white;
  font-size: 20px;
  cursor: pointer;
}

.messages {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
}

.user, .assistant {
  margin: 8px 0;
  padding: 12px;
  border-radius: 8px;
  max-width: 85%;
}

.user {
  background: #e0e7ff;
  margin-left: auto;
}

.assistant {
  background: #f3f4f6;
}

.loading {
  color: #6b7280;
  font-style: italic;
}

.inputArea {
  padding: 16px;
  border-top: 1px solid #e5e7eb;
  display: flex;
  gap: 8px;
}

.inputArea input {
  flex: 1;
  padding: 10px;
  border: 1px solid #d1d5db;
  border-radius: 6px;
}

.inputArea button {
  padding: 10px 20px;
  background: #3b82f6;
  color: white;
  border: none;
  border-radius: 6px;
  cursor: pointer;
}

.inputArea button:disabled {
  background: #9ca3af;
}
```

### 4.5 Start Docusaurus

```bash
npm start
```

## Step 5: Test the Integration

1. Open http://localhost:3000
2. Click the chat bubble (💬) in bottom-right
3. Ask: "What is ROS 2?"
4. Verify response includes source references

## Troubleshooting

| Issue | Solution |
|-------|----------|
| CORS error | Check `CORS_ORIGINS` in `.env` |
| Empty responses | Run `python scripts/ingest.py` to index docs |
| Cohere rate limit | Wait 1 minute, reduce batch size |
| Connection refused | Ensure backend is running on port 8000 |

## Next Steps

1. Deploy backend to Railway/Render
2. Update `API_URL` in widget to production URL
3. Configure production CORS origins
4. Set up monitoring and logging
