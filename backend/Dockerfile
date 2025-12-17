# Multi-stage build for Python 3.11
  FROM python:3.11-slim AS builder

  WORKDIR /app

  # Install dependencies
  COPY requirements.txt .
  RUN pip install --no-cache-dir --user -r requirements.txt

  # Production stage
  FROM python:3.11-slim

  WORKDIR /app

  # Copy dependencies from builder
  COPY --from=builder /root/.local /root/.local
  ENV PATH=/root/.local/bin:$PATH

  # Copy application code
  COPY . .

  # Expose Hugging Face Spaces default port
  EXPOSE 7860

  # Run the application
  CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "7860"]