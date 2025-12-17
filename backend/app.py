"""
Hugging Face Spaces Entry Point
Purpose: Import and expose the FastAPI app for Hugging Face Spaces deployment
Date: 2025-12-18
"""

from src.main import app

# Expose the app for Hugging Face Spaces
# Hugging Face will automatically detect this and run: uvicorn app:app
__all__ = ["app"]
