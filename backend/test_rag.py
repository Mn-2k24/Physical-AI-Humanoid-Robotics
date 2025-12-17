#!/usr/bin/env python3
"""
Test RAG functionality directly
"""
import asyncio
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from dotenv import load_dotenv
load_dotenv()

from src.services.rag import rag_service
from src.services.qdrant import qdrant_service


async def test_rag():
    """Test RAG retrieval and answer generation."""

    # Initialize services
    print("=" * 60)
    print("Testing RAG Chatbot Functionality")
    print("=" * 60)

    # Initialize Qdrant client
    print("\nInitializing Qdrant client...")
    qdrant_service.connect()
    print("✓ Qdrant client connected")

    # Test query about Physical AI
    query = "What is physical AI and how does it relate to humanoid robotics?"

    print(f"\nQuery: {query}")
    print("\n" + "-" * 60)

    # Step 1: Retrieve chunks from Qdrant
    print("\n[Step 1] Retrieving relevant chunks from Qdrant...")
    chunks = await rag_service.retrieve_chunks(query=query, top_k=5)

    if not chunks:
        print("❌ No chunks retrieved from Qdrant!")
        return

    print(f"✓ Retrieved {len(chunks)} chunks:")
    for i, chunk in enumerate(chunks, 1):
        print(f"\n  {i}. Score: {chunk['score']:.4f}")
        print(f"     File: {chunk['file_path']}")
        print(f"     Content preview: {chunk['content'][:150]}...")

    # Step 2: Generate answer using Gemini
    print("\n" + "-" * 60)
    print("\n[Step 2] Generating answer with Gemini...")

    try:
        answer, sources = await rag_service.generate_answer(
            query=query,
            chunks=chunks,
            conversation_history=None
        )

        print(f"\n✓ Answer generated successfully!")
        print(f"\n{'=' * 60}")
        print("ANSWER:")
        print("=" * 60)
        print(answer)
        print("=" * 60)

        print(f"\n✓ Used {len(sources)} source chunks")

    except Exception as e:
        print(f"\n❌ Error generating answer: {e}")
        import traceback
        traceback.print_exc()
        return

    print("\n" + "=" * 60)
    print("✅ RAG Test Completed Successfully!")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(test_rag())
