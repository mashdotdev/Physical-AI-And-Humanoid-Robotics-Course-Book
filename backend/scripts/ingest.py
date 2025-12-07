"""Document ingestion script for the RAG chatbot."""

import os
import re
import glob
import sys
from pathlib import Path
from uuid import uuid4

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv

load_dotenv()

from app.config import get_settings
from app.rag.embeddings import CohereEmbeddings
from app.rag.vectorstore import QdrantVectorStore


def chunk_markdown(text: str, max_chars: int = 2000, overlap: int = 200) -> list[str]:
    """
    Split markdown text into chunks with overlap.

    Respects markdown structure by preferring splits at:
    1. H2 headers (##)
    2. H3 headers (###)
    3. Code blocks (```)
    4. Paragraphs (double newline)
    5. Single newlines
    6. Spaces

    Args:
        text: The markdown text to chunk
        max_chars: Maximum characters per chunk (~500 tokens at 4 chars/token)
        overlap: Character overlap between chunks

    Returns:
        List of text chunks
    """
    if len(text) <= max_chars:
        return [text]

    # Define split patterns in order of preference
    separators = [
        "\n## ",  # H2 headers
        "\n### ",  # H3 headers
        "\n#### ",  # H4 headers
        "\n```",  # Code blocks
        "\n\n",  # Paragraphs
        "\n",  # Lines
        " ",  # Words
    ]

    chunks = []
    current_chunk = ""

    # Split by lines first for easier processing
    lines = text.split("\n")

    for line in lines:
        test_chunk = current_chunk + ("\n" if current_chunk else "") + line

        if len(test_chunk) <= max_chars:
            current_chunk = test_chunk
        else:
            # Current chunk is full, save it
            if current_chunk:
                chunks.append(current_chunk.strip())
                # Start new chunk with overlap from previous
                if overlap > 0 and len(current_chunk) > overlap:
                    # Find a good break point for overlap
                    overlap_text = current_chunk[-overlap:]
                    # Try to start at a sentence or paragraph boundary
                    for sep in ["\n\n", "\n", ". ", " "]:
                        idx = overlap_text.find(sep)
                        if idx > 0:
                            overlap_text = overlap_text[idx + len(sep) :]
                            break
                    current_chunk = overlap_text + "\n" + line
                else:
                    current_chunk = line
            else:
                # Single line exceeds max, keep it anyway
                chunks.append(line.strip())
                current_chunk = ""

    # Don't forget the last chunk
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


def extract_metadata(file_path: str, content: str) -> dict:
    """
    Extract chapter and section metadata from a markdown file.

    Args:
        file_path: Relative path to the file from docs/
        content: The file content

    Returns:
        Dict with chapter, section, and file_path
    """
    # Extract chapter from directory structure
    path_parts = file_path.replace("\\", "/").split("/")

    # Determine chapter from folder name or file location
    chapter = "General"
    if len(path_parts) > 1:
        folder = path_parts[0]
        # Map folder names to friendly chapter names
        chapter_map = {
            "chapter1": "Module 1: The Robotic Nervous System",
            "module-2-digital-twin": "Module 2: The Digital Twin",
            "module-3-ai-robot-brain": "Module 3: The AI-Robot Brain",
            "module-4-vla": "Module 4: Vision-Language-Action",
            "tutorial-basics": "Tutorial Basics",
        }
        chapter = chapter_map.get(folder, folder.replace("-", " ").title())

    # Extract section from first H1 or H2 heading
    section = "Introduction"
    h1_match = re.search(r"^#\s+(.+)$", content, re.MULTILINE)
    if h1_match:
        section = h1_match.group(1).strip()
    else:
        h2_match = re.search(r"^##\s+(.+)$", content, re.MULTILINE)
        if h2_match:
            section = h2_match.group(1).strip()

    return {
        "chapter": chapter,
        "section": section,
        "file_path": file_path,
    }


def ingest_docs(docs_path: str = None) -> int:
    """
    Ingest all markdown documentation into the vector store.

    Pipeline:
    1. Scan for .md files
    2. Parse and extract content
    3. Chunk with markdown awareness
    4. Add metadata
    5. Generate embeddings
    6. Store in Qdrant

    Args:
        docs_path: Path to the docs directory (defaults to book/docs/)

    Returns:
        Number of chunks indexed
    """
    settings = get_settings()

    # Default path relative to this script
    if docs_path is None:
        script_dir = Path(__file__).parent
        docs_path = script_dir.parent.parent / "book" / "docs"

    docs_path = Path(docs_path)

    if not docs_path.exists():
        print(f"ERROR: Docs path not found: {docs_path}")
        return 0

    print(f"Scanning for markdown files in: {docs_path}")

    # Find all markdown files
    md_files = list(docs_path.glob("**/*.md"))
    print(f"Found {len(md_files)} markdown files")

    if not md_files:
        print("No markdown files found!")
        return 0

    # Process each file
    all_chunks = []
    for file_path in md_files:
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()

            # Get relative path from docs directory
            rel_path = str(file_path.relative_to(docs_path))

            # Extract metadata
            metadata = extract_metadata(rel_path, content)

            # Chunk the content
            chunks = chunk_markdown(
                content,
                max_chars=settings.chunk_size,
                overlap=settings.chunk_overlap,
            )

            # Add chunks with metadata
            for i, chunk_text in enumerate(chunks):
                if chunk_text.strip():  # Skip empty chunks
                    all_chunks.append(
                        {
                            "text": chunk_text,
                            "chapter": metadata["chapter"],
                            "section": metadata["section"],
                            "file_path": metadata["file_path"],
                            "position": i,
                        }
                    )

            print(f"  Processed: {rel_path} ({len(chunks)} chunks)")

        except Exception as e:
            print(f"  ERROR processing {file_path}: {e}")

    print(f"\nTotal chunks to index: {len(all_chunks)}")

    if not all_chunks:
        print("No chunks to index!")
        return 0

    # Initialize services
    print("\nInitializing embedding service...")
    embeddings = CohereEmbeddings()

    print("Initializing vector store...")
    vectorstore = QdrantVectorStore()
    vectorstore.create_collection()

    # Generate embeddings in batches
    print("\nGenerating embeddings...")
    texts = [c["text"] for c in all_chunks]
    all_embeddings = embeddings.embed_documents(texts)
    print(f"Generated {len(all_embeddings)} embeddings")

    # Prepare payloads (without the embedding)
    payloads = [
        {
            "text": c["text"],
            "chapter": c["chapter"],
            "section": c["section"],
            "file_path": c["file_path"],
            "position": c["position"],
        }
        for c in all_chunks
    ]

    # Generate unique IDs
    ids = [str(uuid4()) for _ in all_chunks]

    # Upload to Qdrant
    print("\nUploading to Qdrant...")
    vectorstore.upsert(ids=ids, embeddings=all_embeddings, payloads=payloads)

    # Verify
    count = vectorstore.count()
    print(f"\nIndexing complete! {count} vectors in collection.")

    return count


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Ingest documentation into vector store")
    parser.add_argument(
        "--docs-path",
        type=str,
        default=None,
        help="Path to the docs directory",
    )
    args = parser.parse_args()

    ingest_docs(args.docs_path)
