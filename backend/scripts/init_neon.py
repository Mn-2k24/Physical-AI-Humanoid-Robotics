#!/usr/bin/env python3
"""Initialize Neon Postgres database schema."""

import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

import psycopg2
from psycopg2 import sql

from src.core import Config


def main():
    """Create the conversations table and indexes."""
    print("🔗 Connecting to Neon Postgres...")

    try:
        conn = psycopg2.connect(Config.NEON_CONNECTION_STRING)
        conn.autocommit = True
        cursor = conn.cursor()

        print("✅ Connected to Neon Postgres")

        # Create conversations table
        print("📋 Creating conversations table...")

        create_table_query = """
        CREATE TABLE IF NOT EXISTS conversations (
            conversation_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            query_id UUID NOT NULL,
            query_text TEXT NOT NULL,
            answer_text TEXT NOT NULL,
            sources JSONB NOT NULL,
            query_type VARCHAR(20) CHECK (query_type IN ('full_book', 'local')),
            latency_ms INTEGER NOT NULL,
            timestamp TIMESTAMPTZ DEFAULT NOW(),
            user_session_id UUID
        );
        """

        cursor.execute(create_table_query)
        print("✅ conversations table created")

        # Create indexes
        print("📊 Creating indexes...")

        index_queries = [
            "CREATE INDEX IF NOT EXISTS idx_conversations_timestamp ON conversations (timestamp DESC);",
            "CREATE INDEX IF NOT EXISTS idx_conversations_query_type ON conversations (query_type);",
            "CREATE INDEX IF NOT EXISTS idx_conversations_sources ON conversations USING GIN (sources);",
        ]

        for query in index_queries:
            cursor.execute(query)

        print("✅ All indexes created")

        # Check table info
        cursor.execute("""
            SELECT column_name, data_type
            FROM information_schema.columns
            WHERE table_name = 'conversations'
            ORDER BY ordinal_position;
        """)

        columns = cursor.fetchall()
        print("\n📋 Table schema:")
        for col_name, col_type in columns:
            print(f"   - {col_name}: {col_type}")

        cursor.close()
        conn.close()

        print("\n✅ Database initialization complete!")

    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
