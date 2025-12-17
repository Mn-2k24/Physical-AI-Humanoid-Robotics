# ============================================================================
# Multi-Stage Dockerfile for Docusaurus Frontend
# ============================================================================
# Stage 1: Builder - Install dependencies and build production bundle
# Stage 2: Runtime - Serve with nginx
# ============================================================================

# ============================================================================
# Stage 1: Builder
# ============================================================================
FROM node:20-alpine AS builder

# Set working directory
WORKDIR /app

# Copy package files
COPY package.json package-lock.json* ./

# Install dependencies
# --frozen-lockfile: Use exact versions from package-lock.json
# --production=false: Install dev dependencies (needed for build)
RUN npm ci --production=false

# Copy source code
COPY . .

# Build production bundle
# Generates static files in /app/build directory
RUN npm run build

# ============================================================================
# Stage 2: Runtime with Nginx
# ============================================================================
FROM nginx:1.25-alpine

# Copy custom nginx configuration
COPY <<'EOF' /etc/nginx/conf.d/default.conf
server {
    listen 80;
    listen [::]:80;
    server_name _;

    root /usr/share/nginx/html;
    index index.html;

    # Enable gzip compression
    gzip on;
    gzip_vary on;
    gzip_min_length 1024;
    gzip_types text/plain text/css text/xml text/javascript
               application/javascript application/json application/xml+rss
               application/rss+xml font/truetype font/opentype
               application/vnd.ms-fontobject image/svg+xml;

    # Security headers
    add_header X-Content-Type-Options "nosniff" always;
    add_header X-Frame-Options "DENY" always;
    add_header X-XSS-Protection "1; mode=block" always;
    add_header Referrer-Policy "strict-origin-when-cross-origin" always;

    # Docusaurus routing
    # Try to serve file directly, fallback to directory, then to index.html (SPA)
    location / {
        try_files $uri $uri/ $uri.html /index.html;
    }

    # Cache static assets
    location ~* \.(js|css|png|jpg|jpeg|gif|ico|svg|woff|woff2|ttf|eot)$ {
        expires 1y;
        add_header Cache-Control "public, immutable";
    }

    # Don't cache HTML files
    location ~* \.html$ {
        expires -1;
        add_header Cache-Control "no-store, no-cache, must-revalidate, proxy-revalidate, max-age=0";
    }

    # Health check endpoint
    location /health {
        access_log off;
        return 200 "healthy\n";
        add_header Content-Type text/plain;
    }

    # Disable logging for favicon
    location = /favicon.ico {
        access_log off;
        log_not_found off;
    }
}
EOF

# Copy built static files from builder
COPY --from=builder /app/build /usr/share/nginx/html

# Create non-root user
RUN addgroup -g 1000 appuser && \
    adduser -D -u 1000 -G appuser appuser && \
    chown -R appuser:appuser /usr/share/nginx/html && \
    chown -R appuser:appuser /var/cache/nginx && \
    chown -R appuser:appuser /var/log/nginx && \
    touch /var/run/nginx.pid && \
    chown appuser:appuser /var/run/nginx.pid

# Modify nginx to run as non-root user
RUN sed -i 's/user  nginx;/user  appuser;/' /etc/nginx/nginx.conf

# Switch to non-root user
USER appuser

# Expose HTTP port
EXPOSE 80

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD wget --no-verbose --tries=1 --spider http://localhost:80/health || exit 1

# Start nginx
# -g "daemon off;": Run nginx in foreground (required for Docker)
CMD ["nginx", "-g", "daemon off;"]
