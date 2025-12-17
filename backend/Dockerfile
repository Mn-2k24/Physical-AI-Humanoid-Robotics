# ============================================================================
# Multi-Stage Dockerfile for FastAPI Backend
# ============================================================================
# Stage 1: Builder - Install dependencies
# Stage 2: Runtime - Copy dependencies and run app
# ============================================================================

# ============================================================================
# Stage 1: Builder
# ============================================================================
FROM python:3.11-alpine AS builder

# Set working directory
WORKDIR /app

# Install build dependencies
# - gcc, musl-dev: For compiling Python packages with C extensions
# - postgresql-dev: For psycopg2 compilation
# - libffi-dev: For cryptography package
RUN apk add --no-cache \
    gcc \
    musl-dev \
    postgresql-dev \
    libffi-dev \
    openssl-dev \
    cargo \
    rust

# Create virtual environment
RUN python -m venv /opt/venv

# Activate virtual environment
ENV PATH="/opt/venv/bin:$PATH"

# Copy requirements file
COPY requirements.txt .

# Install Python dependencies into virtual environment
# --no-cache-dir: Don't cache pip packages (saves space)
# --upgrade: Upgrade pip, setuptools, wheel
RUN pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir -r requirements.txt

# ============================================================================
# Stage 2: Runtime
# ============================================================================
FROM python:3.11-alpine

# Set working directory
WORKDIR /app

# Install runtime dependencies only
# - postgresql-libs: Required by psycopg2 at runtime
# - libffi: Required by cryptography at runtime
RUN apk add --no-cache \
    postgresql-libs \
    libffi

# Copy virtual environment from builder
COPY --from=builder /opt/venv /opt/venv

# Activate virtual environment
ENV PATH="/opt/venv/bin:$PATH"

# Copy application source code
COPY src/ ./src/
COPY scripts/ ./scripts/

# Create non-root user for security
RUN addgroup -g 1000 appuser && \
    adduser -D -u 1000 -G appuser appuser && \
    chown -R appuser:appuser /app

# Switch to non-root user
USER appuser

# Expose backend port
EXPOSE 8000

# Health check
# Checks if the backend API is responding
HEALTHCHECK --interval=30s --timeout=10s --start-period=40s --retries=3 \
    CMD wget --no-verbose --tries=1 --spider http://localhost:8000/health || exit 1

# Environment variables (can be overridden at runtime)
ENV BACKEND_HOST=0.0.0.0
ENV BACKEND_PORT=8000
ENV ENVIRONMENT=production
ENV LOG_LEVEL=INFO

# Start uvicorn server
# --host 0.0.0.0: Listen on all network interfaces
# --port 8000: Bind to port 8000
# --workers: Number of worker processes (default: 1, can override via env var)
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
