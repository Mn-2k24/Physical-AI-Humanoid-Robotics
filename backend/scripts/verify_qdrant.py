#!/usr/bin/env python3
"""Verify Qdrant connection and collection setup."""

import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

from src.core import Config, EMBEDDING_DIMENSION


def main():
    """Connect to Qdrant and verify/create collection."""
    print("🔍 Verifying Qdrant connection...")

    try:
        # Initialize Qdrant client
        client = QdrantClient(
            url=Config.QDRANT_ENDPOINT,
            api_key=Config.QDRANT_API_KEY,
        )

        print(f"✅ Connected to Qdrant at {Config.QDRANT_ENDPOINT}")

        # Check if collection exists
        collections = client.get_collections()
        collection_names = [col.name for col in collections.collections]

        if Config.QDRANT_COLLECTION_NAME in collection_names:
            print(f"✅ Collection '{Config.QDRANT_COLLECTION_NAME}' exists")

            # Get collection info
            collection_info = client.get_collection(Config.QDRANT_COLLECTION_NAME)
            print(f"\n📊 Collection Information:")
            print(f"   - Vectors count: {collection_info.points_count}")
            print(f"   - Vector size: {collection_info.config.params.vectors.size}")
            print(f"   - Distance: {collection_info.config.params.vectors.distance}")

        else:
            print(f"⚠️  Collection '{Config.QDRANT_COLLECTION_NAME}' does not exist")
            print(f"📝 Creating collection with {EMBEDDING_DIMENSION}-dimensional vectors...")

            # Create collection
            client.create_collection(
                collection_name=Config.QDRANT_COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=EMBEDDING_DIMENSION,
                    distance=Distance.COSINE,
                ),
            )

            print(f"✅ Collection '{Config.QDRANT_COLLECTION_NAME}' created successfully")

    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
