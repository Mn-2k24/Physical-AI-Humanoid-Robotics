#!/usr/bin/env python3
"""
Database Initialization Script
Purpose: Run all migrations in order and create indexes
Date: 2025-12-13
"""

import os
import sys
from pathlib import Path

import psycopg
from dotenv import load_dotenv

# Add parent directory to path to import from src
sys.path.insert(0, str(Path(__file__).parent.parent))

# Load environment variables
load_dotenv()

MIGRATIONS_DIR = Path(__file__).parent / "migrations"
MIGRATION_FILES = sorted(MIGRATIONS_DIR.glob("*.sql"))


def get_db_connection():
    """Create database connection from environment variable."""
    connection_string = os.getenv("NEON_CONNECTION_STRING")
    if not connection_string:
        raise ValueError("NEON_CONNECTION_STRING environment variable not set")
    return psycopg.connect(connection_string)


def run_migration(conn, migration_file: Path):
    """Execute a single migration file."""
    print(f"Running migration: {migration_file.name}")

    with open(migration_file, "r") as f:
        sql = f.read()

    with conn.cursor() as cursor:
        cursor.execute(sql)
        conn.commit()

    print(f"✓ {migration_file.name} completed successfully")


def verify_tables(conn):
    """Verify all expected tables exist."""
    expected_tables = [
        "users",
        "user_profiles",
        "user_software_background",
        "user_hardware_background",
        "auth_sessions",
        "conversations",
        "chat_interactions",
        "reading_progress",
        "recommendations",
        "audit_logs",
    ]

    with conn.cursor() as cursor:
        cursor.execute("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
        """)
        existing_tables = [row[0] for row in cursor.fetchall()]

    missing_tables = set(expected_tables) - set(existing_tables)
    if missing_tables:
        print(f"✗ Missing tables: {', '.join(missing_tables)}")
        return False

    print(f"✓ All {len(expected_tables)} tables created successfully")
    return True


def main():
    """Main database initialization function."""
    print("=" * 60)
    print("Physical AI RAG Chatbot - Database Initialization")
    print("=" * 60)
    print()

    try:
        # Connect to database
        print("Connecting to Neon Serverless Postgres...")
        conn = get_db_connection()
        print("✓ Database connection established")
        print()

        # Run migrations in order
        print(f"Running {len(MIGRATION_FILES)} migrations...")
        for migration_file in MIGRATION_FILES:
            run_migration(conn, migration_file)
        print()

        # Verify tables
        print("Verifying database schema...")
        if verify_tables(conn):
            print()
            print("=" * 60)
            print("✓ Database initialization completed successfully!")
            print("=" * 60)
        else:
            print()
            print("=" * 60)
            print("✗ Database initialization completed with warnings")
            print("=" * 60)
            sys.exit(1)

        conn.close()

    except Exception as e:
        print(f"✗ Error during database initialization: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
