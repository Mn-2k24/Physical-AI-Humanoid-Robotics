# Deployment Guide

Comprehensive guide for deploying the Physical AI & Humanoid Robotics interactive book with RAG chatbot and authentication system.

---

## 📋 Table of Contents

- [Prerequisites](#prerequisites)
- [Docker Deployment](#docker-deployment)
- [Cloud Platforms](#cloud-platforms)
  - [Render.com](#rendercom)
  - [Railway.app](#railwayapp)
  - [Hugging Face Spaces](#hugging-face-spaces)
  - [Vercel (Frontend Only)](#vercel-frontend-only)
  - [DigitalOcean App Platform](#digitalocean-app-platform)
- [Environment Configuration](#environment-configuration)
- [Database Setup](#database-setup)
- [Post-Deployment Checklist](#post-deployment-checklist)
- [Monitoring & Maintenance](#monitoring--maintenance)
- [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Required Services

- **Gemini API Key**: https://makersuite.google.com/app/apikey
- **Qdrant Cloud**: https://cloud.qdrant.io/
- **Neon Postgres**: https://neon.tech/

### Local Testing

Before deploying to production, test locally:

```bash
# Backend
cd backend
python3.11 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
uvicorn src.main:app --reload

# Frontend
npm install
npm run build
npm run serve
```

**Verify**:
- ✅ Backend: http://localhost:8000/health returns 200
- ✅ Frontend: http://localhost:3000 loads correctly
- ✅ Authentication: Signup/signin works
- ✅ Chat: Ask question and get response

---

## Docker Deployment

### Option 1: Docker Compose (Recommended for Self-Hosting)

**1. Clone Repository**:
```bash
git clone https://github.com/Mn-2k24/Physical-AI-Humanoid-Robotics.git
cd Physical-AI-Humanoid-Robotics
```

**2. Configure Environment**:
```bash
cp .env.example .env
# Edit .env with your credentials
nano .env
```

**Required Variables**:
```env
GEMINI_API_KEY=your-key-here
QDRANT_API_KEY=your-key-here
QDRANT_ENDPOINT=https://your-cluster.qdrant.io
QDRANT_COLLECTION=physical_ai_book
NEON_CONNECTION_STRING=postgresql://user:pass@host/db
BETTER_AUTH_SECRET=your-secret-min-32-chars
CSRF_SECRET=your-csrf-secret-min-32-chars
FRONTEND_URL=https://yourdomain.com
CORS_ORIGINS=https://yourdomain.com
```

**3. Build and Deploy**:
```bash
docker-compose up -d --build
```

**4. Verify Services**:
```bash
docker-compose ps
docker-compose logs -f
```

**5. Access Application**:
- Frontend: http://localhost (or your domain)
- Backend: http://localhost:8000
- API Docs: http://localhost:8000/docs

**6. Initialize Database**:
```bash
docker exec -it rag-chatbot-backend python scripts/init_db.py
```

**7. Ingest Book Content** (if needed):
```bash
docker exec -it rag-chatbot-backend python scripts/ingest.py
```

---

### Option 2: Separate Docker Images

**Build Images**:
```bash
# Backend
docker build -t rag-backend:latest ./backend

# Frontend
docker build -t rag-frontend:latest .
```

**Run Containers**:
```bash
# Backend
docker run -d \
  --name rag-backend \
  -p 8000:8000 \
  --env-file .env \
  rag-backend:latest

# Frontend
docker run -d \
  --name rag-frontend \
  -p 80:80 \
  -e BACKEND_URL=http://your-backend-url:8000 \
  rag-frontend:latest
```

---

## Cloud Platforms

### Render.com

**Backend Deployment**:

1. **Create New Web Service**:
   - Connect GitHub repository
   - Select `backend/` as root directory
   - Environment: Python 3.11

2. **Configure Build**:
   ```bash
   # Build Command
   pip install -r requirements.txt

   # Start Command
   uvicorn src.main:app --host 0.0.0.0 --port $PORT
   ```

3. **Environment Variables**:
   - Add all variables from `.env.example`
   - Set `PORT` (Render provides this automatically)
   - Set `FRONTEND_URL` to your frontend URL

4. **Initialize Database**:
   ```bash
   # SSH into Render shell
   python scripts/init_db.py
   python scripts/ingest.py
   ```

**Frontend Deployment**:

1. **Create Static Site**:
   - Connect GitHub repository
   - Build Command: `npm run build`
   - Publish Directory: `build`

2. **Environment Variables**:
   - Set `NODE_ENV=production`

3. **Custom Domain** (optional):
   - Add custom domain in Render dashboard
   - Update DNS records

**Example URLs**:
- Backend: `https://your-backend.onrender.com`
- Frontend: `https://your-frontend.onrender.com`

---

### Railway.app

**1. Install Railway CLI**:
```bash
npm install -g @railway/cli
railway login
```

**2. Deploy Backend**:
```bash
cd backend
railway init
railway add  # Add services (Postgres)
railway up
railway variables set GEMINI_API_KEY=...
railway variables set QDRANT_API_KEY=...
# Set all other env vars
```

**3. Deploy Frontend**:
```bash
cd ..
railway init
railway up
railway domain  # Generate railway.app domain
```

**4. Configure Environment**:
```bash
# Backend
railway variables set FRONTEND_URL=https://your-frontend.railway.app

# Frontend
railway variables set BACKEND_URL=https://your-backend.railway.app
```

---

### Hugging Face Spaces

**Backend as Space**:

1. **Create New Space**:
   - SDK: Docker
   - Visibility: Public or Private

2. **Add Dockerfile**:
   ```dockerfile
   FROM python:3.11-slim

   WORKDIR /app
   COPY backend/ .
   RUN pip install -r requirements.txt

   EXPOSE 7860
   CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "7860"]
   ```

3. **Add Secrets**:
   - Go to Settings → Repository secrets
   - Add all environment variables

4. **Push to Space**:
   ```bash
   git remote add space https://huggingface.co/spaces/username/space-name
   git push space main
   ```

**Frontend**:
- Use Vercel or Netlify for frontend
- Point `BACKEND_URL` to HF Space URL

---

### Vercel (Frontend Only)

**1. Install Vercel CLI**:
```bash
npm install -g vercel
vercel login
```

**2. Deploy**:
```bash
vercel --prod
```

**3. Configure Environment**:
- Go to Vercel dashboard → Project → Settings → Environment Variables
- Add `BACKEND_URL` pointing to your backend

**4. Custom Domain**:
- Add custom domain in Vercel dashboard
- Update DNS records

**Automatic Deployments**:
- Connect GitHub repository
- Auto-deploy on push to `main` branch

---

### DigitalOcean App Platform

**Backend App**:

1. **Create New App**:
   - Source: GitHub repository
   - Resource Type: Web Service
   - Dockerfile: `backend/Dockerfile`

2. **Configure**:
   ```yaml
   name: rag-chatbot-backend
   services:
   - name: backend
     dockerfile_path: backend/Dockerfile
     http_port: 8000
     routes:
     - path: /
     envs:
     - key: GEMINI_API_KEY
       value: ${GEMINI_API_KEY}
     # ... add all env vars
   ```

3. **Deploy**:
   - Click "Create Resources"
   - Wait for deployment

**Frontend App**:

1. **Create Static Site**:
   - Build Command: `npm run build`
   - Output Directory: `build`

2. **Environment Variables**:
   - `BACKEND_URL`: Your backend URL

---

## Environment Configuration

### Production Environment Variables

**Backend** (`.env` or platform secrets):

```env
# Required
GEMINI_API_KEY=AIzaSy...
QDRANT_API_KEY=eyJhbGci...
QDRANT_ENDPOINT=https://....qdrant.io
QDRANT_COLLECTION=physical_ai_book
NEON_CONNECTION_STRING=postgresql://...
BETTER_AUTH_SECRET=min-32-char-secret-here
CSRF_SECRET=min-32-char-csrf-secret-here

# CORS
FRONTEND_URL=https://yourdomain.com
CORS_ORIGINS=https://yourdomain.com,https://www.yourdomain.com

# Optional (with defaults)
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000
ENVIRONMENT=production
LOG_LEVEL=INFO
RATE_LIMIT_PER_MINUTE=5
SESSION_EXPIRATION_HOURS=24
MAX_CONVERSATION_MESSAGES=10
RECOMMENDATION_LIMIT=5
```

**Frontend** (`docusaurus.config.ts`):

Update production URL:
```typescript
customFields: {
  apiUrl: process.env.NODE_ENV === 'production'
    ? 'https://your-backend-url.com'  // ← Update this
    : 'http://localhost:8000',
},
```

---

## Database Setup

### Neon Postgres

**1. Create Database**:
- Go to https://neon.tech/
- Create new project
- Copy connection string

**2. Run Migrations**:
```bash
# Local
python backend/scripts/init_db.py

# Docker
docker exec -it rag-chatbot-backend python scripts/init_db.py

# Render/Railway
# Use platform shell or Makefile
```

**3. Verify Tables**:
```sql
\dt  -- List all tables
SELECT * FROM users LIMIT 1;
```

### Qdrant Cloud

**1. Create Cluster**:
- Go to https://cloud.qdrant.io/
- Create cluster (free tier available)
- Copy API key and endpoint

**2. Ingest Book Content**:
```bash
# Local
python backend/scripts/ingest.py

# Docker
docker exec -it rag-chatbot-backend python scripts/ingest.py
```

**3. Verify Collection**:
```bash
curl -X GET "https://your-cluster.qdrant.io/collections/physical_ai_book" \
  -H "api-key: your-api-key"
```

---

## Post-Deployment Checklist

### Functional Testing

- [ ] **Backend Health**: `/health` endpoint returns 200
- [ ] **Frontend Loads**: Homepage renders correctly
- [ ] **Authentication**: Signup and signin work
- [ ] **Chat**: Ask question and receive answer
- [ ] **Text Selection**: Select text → "Ask AI" appears
- [ ] **Dark Mode**: Toggle works without errors
- [ ] **Recommendations**: Sidebar shows recommendations

### Security Checks

- [ ] **HTTPS**: Ensure SSL/TLS enabled (not HTTP)
- [ ] **Cookies**: `Secure` flag set on session cookies
- [ ] **CORS**: Only allowed origins configured
- [ ] **Secrets**: No hardcoded API keys in code
- [ ] **Rate Limiting**: Auth endpoints rate-limited

### Performance

- [ ] **API Response Time**: < 2 seconds
- [ ] **Page Load Time**: < 3 seconds
- [ ] **Lighthouse Score**: ≥90 performance
- [ ] **Resource Usage**: Within platform limits

### Monitoring

- [ ] **Logs**: Backend logs accessible
- [ ] **Errors**: No unhandled exceptions
- [ ] **Database**: Connection pool healthy
- [ ] **Uptime**: Service status green

---

## Monitoring & Maintenance

### Logging

**Backend Logs**:
```bash
# Docker
docker-compose logs -f backend

# Render
# View in dashboard logs tab

# Railway
railway logs
```

**Frontend Logs**:
```bash
# Docker
docker-compose logs -f frontend

# Vercel
vercel logs
```

### Health Monitoring

**Setup Uptime Monitoring**:
- UptimeRobot: https://uptimerobot.com/
- Pingdom: https://www.pingdom.com/
- Better Uptime: https://betteruptime.com/

**Monitor Endpoints**:
- Backend: `https://your-backend.com/health`
- Frontend: `https://your-frontend.com/health`

### Error Tracking (Optional)

**Sentry Integration**:

1. Create Sentry project: https://sentry.io/
2. Add to backend:
   ```python
   import sentry_sdk
   sentry_sdk.init(dsn="your-dsn")
   ```
3. Set `SENTRY_DSN` environment variable

### Database Backups

**Neon Postgres**:
- Automatic backups (free tier: 7 days)
- Manual snapshots in dashboard

**Qdrant**:
- Export collection:
  ```bash
  curl -X POST "https://your-cluster.qdrant.io/collections/physical_ai_book/snapshots" \
    -H "api-key: your-api-key"
  ```

---

## Troubleshooting

### Backend Won't Start

**Symptom**: Container exits immediately

**Check**:
```bash
docker logs rag-chatbot-backend
```

**Common Causes**:
- Missing environment variables
- Invalid database connection string
- Port already in use

**Solution**:
- Verify all env vars set
- Test database connection
- Change port mapping

---

### CORS Errors

**Symptom**: `Access to fetch ... has been blocked by CORS policy`

**Solution**:
1. Add frontend URL to `CORS_ORIGINS`:
   ```env
   CORS_ORIGINS=https://frontend.com,https://www.frontend.com
   ```
2. Restart backend

---

### Authentication Not Persisting

**Symptom**: User logged out after page refresh

**Check**:
- Cookies being set? (DevTools → Application → Cookies)
- `Secure` flag requires HTTPS

**Solution**:
- Ensure HTTPS enabled in production
- Verify `SESSION_EXPIRATION_HOURS` set
- Check cookie domain matches frontend

---

### Database Connection Failed

**Symptom**: `connection to server failed`

**Solution**:
1. Verify `NEON_CONNECTION_STRING` correct
2. Check Neon database is active (not paused)
3. Test connection:
   ```bash
   psql $NEON_CONNECTION_STRING -c "SELECT 1;"
   ```

---

### Qdrant Connection Failed

**Symptom**: `Unable to connect to Qdrant`

**Solution**:
1. Verify `QDRANT_API_KEY` and `QDRANT_ENDPOINT`
2. Check cluster status in Qdrant dashboard
3. Test connection:
   ```bash
   curl "https://your-cluster.qdrant.io/collections" \
     -H "api-key: your-api-key"
   ```

---

### Build Fails

**Frontend Build Error**: `process is not defined`

**Solution**:
- Do NOT use `process.env` in React components
- Use `src/utils/config.ts` utility functions

**Backend Build Error**: Missing packages

**Solution**:
```bash
pip install -r requirements.txt
pip freeze > requirements.txt
```

---

## Deployment Comparison

| Platform | Backend | Frontend | Price | Ease | Performance |
|----------|---------|----------|-------|------|-------------|
| **Docker Compose** | ✅ | ✅ | Self-hosted | Medium | High |
| **Render** | ✅ | ✅ | $7-25/mo | Easy | Medium |
| **Railway** | ✅ | ✅ | $5-20/mo | Easy | High |
| **Vercel + Render** | Render | Vercel | $7/mo | Easy | High |
| **HF Spaces** | ✅ | ❌ | Free-$9/mo | Medium | Medium |
| **DigitalOcean** | ✅ | ✅ | $12-24/mo | Medium | High |

**Recommendation**:
- **Hobby/Learning**: Docker Compose (self-host) or HF Spaces + Vercel
- **Production**: Render or Railway (easiest) or DigitalOcean (best performance)

---

**Last Updated**: 2025-12-14
