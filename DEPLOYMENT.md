# Deployment Guide - RAG Chatbot

Complete guide to deploy your RAG chatbot backend and frontend to production.

## Overview

- **Backend**: Deploy to Render (free tier)
- **Frontend**: Already deployed on Vercel
- **Databases**: Qdrant Cloud & Neon (already cloud-based)

## Prerequisites

✅ Backend running locally
✅ Embeddings uploaded to Qdrant
✅ Database initialized on Neon
✅ GitHub repository up to date

## Step 1: Deploy Backend to Render

### 1.1 Create Render Account

1. Go to https://render.com
2. Sign up with GitHub account
3. Authorize Render to access your repositories

### 1.2 Create New Web Service

1. Click **"New +"** → **"Web Service"**
2. Select your repository: `khan561hunter/Hackathon-One`
3. Click **"Connect"**

### 1.3 Configure Service

Fill in these settings:

```
Name: hackathon-rag-backend
Region: Choose closest to your users (e.g., Oregon - US West)
Branch: main
Root Directory: backend
Runtime: Python 3
```

**Build Command:**
```bash
pip install -r requirements.txt
```

**Start Command:**
```bash
uvicorn main:app --host 0.0.0.0 --port $PORT
```

### 1.4 Add Environment Variables

Click **"Advanced"** → **"Add Environment Variable"** and add these:

| Key | Value | Notes |
|-----|-------|-------|
| `COHERE_API_KEY` | `your_cohere_api_key` | From Cohere dashboard |
| `ANTHROPIC_API_KEY` | `your_claude_api_key` | Optional - for AI answers |
| `QDRANT_API_KEY` | `your_qdrant_key` | From Qdrant Cloud |
| `QDRANT_URL` | `https://xxx.qdrant.io:6333` | Your Qdrant cluster URL |
| `NEON_DB_URL` | `postgresql://user:pass@...` | Full connection string |
| `CORS_ORIGINS` | `https://hackathon-one-seven.vercel.app` | Your Vercel URL |
| `HOST` | `0.0.0.0` | Required for Render |
| `LOG_LEVEL` | `INFO` | Logging level |

**Important CORS_ORIGINS format:**
```
https://hackathon-one-seven.vercel.app,http://localhost:3000
```
(No spaces between URLs!)

### 1.5 Configure Health Check

- **Health Check Path**: `/health`
- Leave other settings as default

### 1.6 Deploy

1. Click **"Create Web Service"**
2. Wait for build to complete (5-10 minutes)
3. Watch the logs for any errors

**Expected log output:**
```
INFO:     Starting up RAG chatbot backend...
INFO:     Connected to Postgres database
INFO:     Database schema initialized successfully
INFO:     Initialized RAG pipeline with Cohere embeddings
INFO:     Backend startup complete!
INFO:     Uvicorn running on http://0.0.0.0:10000
```

### 1.7 Get Your Backend URL

Once deployed, you'll see:
```
Your service is live at https://hackathon-rag-backend.onrender.com
```

**Copy this URL!** You'll need it for the frontend.

### 1.8 Test Backend

```bash
# Health check
curl https://hackathon-rag-backend.onrender.com/health

# Test question
curl -X POST https://hackathon-rag-backend.onrender.com/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?", "stream": false}'
```

## Step 2: Update Frontend Configuration

### 2.1 Update Backend URL in Code

Replace the backend URL in your chatbot component with your actual Render URL.

The file has already been updated with a placeholder. Now update it with your real URL:

```javascript
const BACKEND_URL = typeof window !== 'undefined' && window.location.hostname === 'localhost'
  ? 'http://localhost:8000'
  : 'https://hackathon-rag-backend.onrender.com'; // Replace with YOUR actual URL
```

### 2.2 Commit and Push Changes

```bash
git add src/components/Chatbot/index.jsx
git add backend/render.yaml
git add DEPLOYMENT.md
git commit -m "Add production backend URL and Render deployment config"
git push origin main
```

### 2.3 Vercel Auto-Deploy

Vercel will automatically detect your push and redeploy:

1. Go to https://vercel.com/dashboard
2. Find your project: `hackathon-one-seven`
3. Wait for deployment to complete (2-3 minutes)
4. You'll see: **"Deployment Complete"**

## Step 3: Verify Production Deployment

### 3.1 Test on Desktop

1. Visit: https://hackathon-one-seven.vercel.app
2. Open chat widget (💬 button)
3. Ask a question
4. Verify you get an answer with sources

### 3.2 Test on Mobile

1. Open https://hackathon-one-seven.vercel.app on your phone
2. Open chat widget
3. Ask a question
4. Should now work! ✅

### 3.3 Check Backend Logs

On Render dashboard:
1. Go to your service
2. Click **"Logs"** tab
3. Watch for incoming requests

You should see:
```
INFO:     Embedding question: What is Physical AI...
INFO:     Retrieving top 3 chunks from Qdrant
INFO:     Generating answer with Claude using 3 chunks
```

## Troubleshooting

### Issue: Backend Build Fails on Render

**Error**: `ModuleNotFoundError: No module named 'cohere'`

**Solution:**
1. Verify `requirements.txt` is in `backend/` folder
2. Check Root Directory is set to `backend`
3. Rebuild service

### Issue: Backend Starts But Health Check Fails

**Error**: "Your service is unhealthy"

**Solutions:**
1. Check environment variables are set correctly
2. Verify `NEON_DB_URL` connection string is valid
3. Check logs for database connection errors
4. Ensure health check path is `/health` (not `/`)

### Issue: CORS Errors on Frontend

**Error**: "blocked by CORS policy"

**Solutions:**
1. Add your Vercel URL to `CORS_ORIGINS` on Render
2. Format must be: `https://your-app.vercel.app` (no trailing slash)
3. Multiple origins: `https://app1.com,https://app2.com` (comma-separated, no spaces)
4. Restart backend service after changing env vars

### Issue: "No relevant information found"

**Cause**: Embeddings not uploaded to Qdrant

**Solution:**
1. Upload embeddings locally (they're in the cloud database)
2. Or create a one-time script to upload from Render (advanced)

### Issue: Mobile Still Shows "localhost" Error

**Causes:**
1. Frontend not updated with production backend URL
2. Old cached version on mobile

**Solutions:**
1. Verify line 19 in `index.jsx` has your Render URL
2. Clear browser cache on mobile
3. Hard refresh on desktop (Ctrl+Shift+R)
4. Check Vercel deployed the latest commit

### Issue: Render Service Spins Down (Free Tier)

**Symptom**: First request takes 30-60 seconds

**Explanation**: Render free tier spins down after 15 minutes of inactivity

**Solutions:**
1. Upgrade to paid tier ($7/month) for always-on
2. Accept cold starts (happens once every 15 min)
3. Use a service like UptimeRobot to ping every 14 minutes (free)

## Performance Optimization

### Enable Render Persistent Disk (Optional)

For faster cold starts, add persistent storage:

1. Go to service settings
2. Add **"Disk"**
3. Mount path: `/opt/render/project/.venv`

### Use Production ASGI Server

For better performance, update start command:

```bash
gunicorn main:app --workers 4 --worker-class uvicorn.workers.UvicornWorker --bind 0.0.0.0:$PORT
```

Add to `requirements.txt`:
```
gunicorn==21.2.0
```

### Database Connection Pooling

Already configured in `postgres.py`:
```python
self.pool = await asyncpg.create_pool(
    self.connection_url,
    min_size=1,
    max_size=10,  # Adjust based on load
)
```

## Monitoring

### Render Dashboard

Monitor:
- **Logs**: Real-time request logs
- **Metrics**: CPU, Memory, Bandwidth
- **Events**: Deployments, restarts

### Setup Alerts

1. Go to service settings
2. Add notification emails
3. Get alerts for:
   - Service down
   - Failed health checks
   - High error rates

### Log Rotation

Render keeps logs for 7 days on free tier. For longer retention:
1. Use external logging service (e.g., Logtail, Papertrail)
2. Or upgrade to paid tier

## Cost Breakdown

### Current Setup (Free Tier)

| Service | Free Tier | Limits |
|---------|-----------|--------|
| Render | ✅ Free | 750 hours/month, spins down after 15min idle |
| Vercel | ✅ Free | 100GB bandwidth, unlimited requests |
| Qdrant Cloud | ✅ Free | 1GB storage |
| Neon Postgres | ✅ Free | 512MB storage, 1 project |
| Cohere API | ✅ Free | 100 calls/min |

**Total Monthly Cost: $0** 🎉

### Scaling (Paid Tiers)

When you need more:

| Service | Paid Plan | Cost |
|---------|-----------|------|
| Render Starter | Always-on, more RAM | $7/month |
| Qdrant Cloud 1GB | More storage | $10/month |
| Neon Scale | More storage/compute | $19/month |
| Cohere Production | Higher rate limits | Pay per use |

## Security Best Practices

### Environment Variables

✅ **DO**:
- Store in Render environment variables
- Never commit to Git
- Use different keys for dev/prod

❌ **DON'T**:
- Hardcode API keys
- Share `.env` files
- Use same keys for multiple projects

### CORS Configuration

Only allow your domains:
```
CORS_ORIGINS=https://hackathon-one-seven.vercel.app
```

### HTTPS

✅ Render automatically provides SSL
✅ Vercel automatically provides SSL
✅ Always use `https://` URLs in production

## Updating Your Deployment

### Update Backend Code

1. Make changes locally
2. Test locally: `python main.py`
3. Commit and push to GitHub
4. Render auto-deploys from `main` branch
5. Watch build logs on Render dashboard

### Update Frontend Code

1. Make changes locally
2. Test locally: `npm start`
3. Commit and push to GitHub
4. Vercel auto-deploys
5. Verify at your Vercel URL

### Update Environment Variables

On Render:
1. Go to service → **"Environment"**
2. Edit variables
3. Click **"Save Changes"**
4. Service auto-restarts

### Update Dependencies

Backend (`requirements.txt`):
```bash
# Add new package
pip install new-package
pip freeze > requirements.txt
git commit -am "Add new-package"
git push
```

Frontend (`package.json`):
```bash
npm install new-package
git commit -am "Add new-package"
git push
```

## Rollback

### Rollback Backend

On Render:
1. Go to **"Events"** tab
2. Find successful deployment
3. Click **"Rollback to this deploy"**

### Rollback Frontend

On Vercel:
1. Go to **"Deployments"** tab
2. Find working deployment
3. Click **"Promote to Production"**

### Rollback Database

Neon has automatic backups:
1. Go to Neon dashboard
2. Click **"Restore"**
3. Select restore point

## Next Steps

✅ Backend deployed on Render
✅ Frontend deployed on Vercel
✅ Working on mobile and desktop

Now you can:

1. **Monitor usage**: Check Render/Vercel dashboards
2. **Add features**: See [API.md](./API.md) for extending functionality
3. **Optimize**: Add caching, rate limiting
4. **Scale**: Upgrade to paid tiers when needed

---

## Quick Reference

### Important URLs

- **Backend**: https://hackathon-rag-backend.onrender.com
- **Frontend**: https://hackathon-one-seven.vercel.app
- **Render Dashboard**: https://dashboard.render.com
- **Vercel Dashboard**: https://vercel.com/dashboard

### Commands

```bash
# Test backend health
curl https://your-backend.onrender.com/health

# Test backend question
curl -X POST https://your-backend.onrender.com/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "test"}'

# Deploy changes
git add .
git commit -m "Update"
git push origin main
```

---

**Deployment Complete!** 🚀

Your RAG chatbot is now live and accessible from anywhere in the world!
