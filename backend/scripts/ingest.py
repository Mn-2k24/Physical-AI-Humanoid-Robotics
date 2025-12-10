#!/usr/bin/env python3
"""Ingest Markdown files from /docs into Qdrant."""

import sys
import time
import logging
from pathlib import Path
from typing import List
from uuid import uuid4

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct

from src.core import Config
from src.services.chunking import process_markdown_file
from src.services.embeddings import generate_embeddings
from src.models.chunk import TextChunk

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def find_markdown_files(docs_dir: Path) -> List[Path]:
    """Recursively find all .md files in the docs directory.

    Args:
        docs_dir: Path to the docs directory

    Returns:
        List of Path objects for .md files
    """
    return list(docs_dir.rglob("*.md"))


def count_words(text: str) -> int:
    """Count words in text."""
    return len(text.split())


def main():
    """Main ingestion pipeline."""
    start_time = time.time()

    # Determine docs directory (relative to repo root)
    repo_root = Path(__file__).parent.parent.parent
    docs_dir = repo_root / "docs"

    if not docs_dir.exists():
        logger.error(f"Docs directory not found: {docs_dir}")
        sys.exit(1)

    logger.info(f"📁 Scanning for Markdown files in: {docs_dir}")

    # Find all markdown files
    md_files = find_markdown_files(docs_dir)
    logger.info(f"📄 Found {len(md_files)} Markdown files")

    if not md_files:
        logger.warning("No Markdown files found. Exiting.")
        sys.exit(0)

    # Initialize Qdrant client
    logger.info(f"🔗 Connecting to Qdrant at {Config.QDRANT_ENDPOINT}")
    try:
        client = QdrantClient(
            url=Config.QDRANT_ENDPOINT,
            api_key=Config.QDRANT_API_KEY,
        )
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {e}")
        sys.exit(1)

    # Process files
    total_chunks = 0
    skipped_files = 0
    failed_files = 0

    all_chunks: List[TextChunk] = []

    for md_file in md_files:
        # Convert to relative path from repo root for storage
        relative_path = md_file.relative_to(repo_root)
        file_path_str = str(relative_path)

        logger.info(f"📖 Processing: {file_path_str}")

        try:
            # Read file and check word count
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            word_count = count_words(content)

            # Skip files with < 50 words
            if word_count < 50:
                logger.warning(f"⚠️  Skipping {file_path_str} (only {word_count} words)")
                skipped_files += 1
                continue

            # Process the file into chunks
            try:
                chunks = process_markdown_file(str(md_file))
                logger.info(f"   ✂️  Created {len(chunks)} chunks")
                all_chunks.extend(chunks)
                total_chunks += len(chunks)

            except Exception as e:
                logger.error(f"   ❌ Failed to chunk {file_path_str}: {e}")
                failed_files += 1
                continue

        except Exception as e:
            logger.error(f"   ❌ Failed to read {file_path_str}: {e}")
            failed_files += 1
            continue

    logger.info(f"\n📊 Chunking complete:")
    logger.info(f"   - Total chunks: {total_chunks}")
    logger.info(f"   - Skipped files: {skipped_files}")
    logger.info(f"   - Failed files: {failed_files}")

    if not all_chunks:
        logger.warning("No chunks to upload. Exiting.")
        sys.exit(0)

    # Generate embeddings
    logger.info(f"\n🧠 Generating embeddings for {len(all_chunks)} chunks...")
    try:
        embeddings = generate_embeddings(all_chunks)
        logger.info(f"✅ Generated {len(embeddings)} embeddings")
    except Exception as e:
        logger.error(f"❌ Failed to generate embeddings: {e}")
        sys.exit(1)

    # Upload to Qdrant
    logger.info(f"\n☁️  Uploading to Qdrant collection '{Config.QDRANT_COLLECTION_NAME}'...")

    points = []
    for chunk, embedding in zip(all_chunks, embeddings):
        point = PointStruct(
            id=str(uuid4()),
            vector=embedding,
            payload={
                "file_path": chunk.file_path,
                "section": chunk.section,
                "text": chunk.text,
                "token_count": chunk.token_count,
                "chunk_index": chunk.chunk_index,
                "overlap_with_prev": chunk.overlap_with_prev,
                "overlap_with_next": chunk.overlap_with_next,
            }
        )
        points.append(point)

    # Upload in batches to avoid memory issues
    batch_size = 100
    uploaded = 0

    try:
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            client.upsert(
                collection_name=Config.QDRANT_COLLECTION_NAME,
                points=batch,
            )
            uploaded += len(batch)
            logger.info(f"   📤 Uploaded {uploaded}/{len(points)} points")

        logger.info(f"✅ All chunks uploaded successfully!")

    except Exception as e:
        logger.error(f"❌ Failed to upload to Qdrant: {e}")
        logger.error(f"   Check if you've exceeded the free tier quota (1M vectors, 1GB)")
        sys.exit(1)

    # Summary
    elapsed_time = time.time() - start_time
    logger.info(f"\n✨ Ingestion complete!")
    logger.info(f"   - Files processed: {len(md_files) - skipped_files - failed_files}")
    logger.info(f"   - Total chunks: {total_chunks}")
    logger.info(f"   - Uploaded to Qdrant: {uploaded}")
    logger.info(f"   - Time elapsed: {elapsed_time:.2f}s")


if __name__ == "__main__":
    main()
