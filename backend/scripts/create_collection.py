"""Script to create or recreate the Qdrant collection."""

import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv

load_dotenv()

from app.rag.vectorstore import QdrantVectorStore


def create_collection(force: bool = False) -> None:
    """
    Create the Qdrant collection.

    Args:
        force: If True, delete existing collection first
    """
    print("Initializing Qdrant client...")
    vectorstore = QdrantVectorStore()

    if force:
        print("Deleting existing collection...")
        try:
            vectorstore.delete_collection()
            print("Collection deleted.")
        except Exception as e:
            print(f"Could not delete collection (may not exist): {e}")

    print("Creating collection...")
    vectorstore.create_collection()
    print(f"Collection '{vectorstore.collection_name}' created successfully!")

    # Verify
    count = vectorstore.count()
    print(f"Current vector count: {count}")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Create Qdrant collection")
    parser.add_argument(
        "--force",
        action="store_true",
        help="Delete existing collection before creating",
    )
    args = parser.parse_args()

    create_collection(force=args.force)
