#!/usr/bin/env python3
"""
Book Ingestion Script
Purpose: Read /docs markdown files, chunk content, generate embeddings, upload to Qdrant
Date: 2025-12-13
"""

import os
import sys
import uuid
from pathlib import Path

from dotenv import load_dotenv
from qdrant_client.models import PointStruct

# Add parent directory to path to import from src
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.embedding import embedding_service
from src.services.qdrant import qdrant_service
from src.utils.chunking import TextChunker

# Load environment variables
load_dotenv()


class BookIngestion:
    """Book content ingestion pipeline."""

    def __init__(self, docs_dir: str = "docs"):
        """
        Initialize book ingestion.

        Args:
            docs_dir: Directory containing markdown files
        """
        self.docs_dir = Path(docs_dir)
        self.chunker = TextChunker(max_chunk_size=500)
        self.total_chunks = 0
        self.total_files = 0

    def ingest_all(self):
        """Ingest all markdown files from docs directory."""
        print("=" * 60)
        print("Physical AI Book - Content Ingestion")
        print("=" * 60)
        print()

        # Initialize services
        print("Initializing services...")
        qdrant_service.connect()
        print("✓ Qdrant client initialized")
        print()

        # Find all markdown files
        markdown_files = list(self.docs_dir.rglob("*.md"))
        print(f"Found {len(markdown_files)} markdown files in {self.docs_dir}")
        print()

        # Process each file
        all_points = []
        for md_file in markdown_files:
            points = self.process_file(md_file)
            all_points.extend(points)

        # Upload to Qdrant in batches
        if all_points:
            self.upload_to_qdrant(all_points)

        # Summary
        print()
        print("=" * 60)
        print("Ingestion Complete")
        print("=" * 60)
        print(f"Files processed: {self.total_files}")
        print(f"Chunks created: {self.total_chunks}")
        print(f"Vectors uploaded: {len(all_points)}")
        print()

        # Show collection info
        info = qdrant_service.get_collection_info()
        print("Collection Info:")
        print(f"  Points count: {info['points_count']}")
        print(f"  Status: {info['status']}")
        print("=" * 60)

    def process_file(self, file_path: Path) -> list[PointStruct]:
        """
        Process a single markdown file.

        Args:
            file_path: Path to markdown file

        Returns:
            List of PointStruct objects for Qdrant
        """
        print(f"Processing: {file_path.relative_to(self.docs_dir)}")

        # Read file content
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
        except Exception as e:
            print(f"  ✗ Failed to read file: {e}")
            return []

        # Skip empty files
        if not content.strip():
            print(f"  ⊘ Skipping empty file")
            return []

        # Chunk content
        chunks = self.chunker.chunk_text(content, preserve_paragraphs=True)
        print(f"  → Created {len(chunks)} chunks")

        # Generate embeddings
        chunk_texts = [chunk_text for chunk_text, _ in chunks]
        try:
            embeddings = embedding_service.generate_embeddings_batch(
                chunk_texts, task_type="retrieval_document", batch_size=100
            )
            print(f"  → Generated {len(embeddings)} embeddings")
        except Exception as e:
            print(f"  ✗ Failed to generate embeddings: {e}")
            return []

        # Create Qdrant points
        points = []
        for (chunk_text, chunk_index), embedding in zip(chunks, embeddings):
            # Extract section heading
            section_heading = self.chunker.extract_section_heading(chunk_text)

            # Create point
            point = PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload={
                    "file_path": str(file_path.relative_to(self.docs_dir.parent)),
                    "section_heading": section_heading,
                    "chunk_index": chunk_index,
                    "raw_text": chunk_text,
                },
            )
            points.append(point)

        self.total_chunks += len(chunks)
        self.total_files += 1

        print(f"  ✓ Processed {len(points)} points")
        return points

    def upload_to_qdrant(self, points: list[PointStruct], batch_size: int = 100):
        """
        Upload points to Qdrant in batches.

        Args:
            points: List of PointStruct objects
            batch_size: Number of points per batch
        """
        print()
        print(f"Uploading {len(points)} points to Qdrant...")

        for i in range(0, len(points), batch_size):
            batch = points[i : i + batch_size]
            try:
                qdrant_service.upsert(batch)
                print(f"  ✓ Uploaded batch {i // batch_size + 1} ({len(batch)} points)")
            except Exception as e:
                print(f"  ✗ Failed to upload batch {i // batch_size + 1}: {e}")

        print(f"✓ Upload complete")


def main():
    """Main ingestion function."""
    import argparse

    parser = argparse.ArgumentParser(description="Ingest book content into Qdrant")
    parser.add_argument(
        "--docs-dir", type=str, default="docs", help="Directory containing markdown files"
    )
    parser.add_argument("--force", action="store_true", help="Force re-ingestion (delete existing collection)")

    args = parser.parse_args()

    # Check if docs directory exists
    docs_path = Path(args.docs_dir)
    if not docs_path.exists():
        print(f"✗ Error: Directory '{args.docs_dir}' does not exist")
        sys.exit(1)

    # Force re-ingestion if requested
    if args.force:
        print("⚠ Force flag set - deleting existing collection...")
        try:
            qdrant_service.connect()
            qdrant_service.delete_collection()
            print("✓ Collection deleted")
        except Exception as e:
            print(f"⚠ Could not delete collection: {e}")

    # Run ingestion
    ingestion = BookIngestion(docs_dir=args.docs_dir)
    ingestion.ingest_all()


if __name__ == "__main__":
    main()
