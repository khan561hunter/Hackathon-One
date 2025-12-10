# 📱 Mobile Deployment Guide

Complete guide to make your RAG chatbot work on mobile devices by deploying the backend to Render.com.

---

## 🎯 Overview

**Current Setup:**
- ✅ Frontend: Deployed to Vercel (https://hackathon-one-seven.vercel.app)
- ❌ Backend: Running locally (http://localhost:8000) - **NOT accessible from mobile**

**Goal:**
- ✅ Deploy backend to Render.com (publicly accessible)
- ✅ Configure frontend to use deployed backend
- ✅ Chatbot works on mobile and desktop

---

## 🚀 Step-by-Step Deployment

### **Step 1: Deploy Backend to Render.com**

#### 1.1 Create Render Account
1. Go to https://render.com
2. Click **"Get Started for Free"**
3. Sign up with GitHub
4. Authorize Render to access your repositories

#### 1.2 Create New Web Service
1. From Render dashboard, click **"New +"** → **"Web Service"**
2. Select **"Build and deploy from a Git repository"**
3. Connect your GitHub account if not already connected
4. Find and select your repository: `khan561hunter/Hackathon-One`
5. Click **"Connect"**

#### 1.3 Configure Service
Render will auto-detect `backend/render.yaml`. Verify these settings:

**Basic Settings:**
- **Name:** `hackathon-rag-backend`
- **Region:** Choose closest to your users (e.g., Oregon USA)
- **Branch:** `main`
- **Root Directory:** `backend`
- **Runtime:** Python 3
- **Build Command:** `pip install -r requirements.txt`
- **Start Command:** `uvicorn main:app --host 0.0.0.0 --port $PORT`

**Plan:**
- Select **Free** plan (for testing)
- Note: Free plan may spin down with inactivity (cold starts)

#### 1.4 Add Environment Variables

Click **"Advanced"** → **"Add Environment Variable"** for each:

```bash
# AI API Keys
COHERE_API_KEY=<your-cohere-api-key>
CLAUDE_API_KEY=<your-claude-api-key>

# Vector Database
QDRANT_API_KEY=<your-qdrant-api-key>
QDRANT_URL=<your-qdrant-cloud-url>
QDRANT_COLLECTION=book_embeddings

# Postgres Database
NEON_DB_URL=<your-neon-postgres-url>

# Server Configuration (already in render.yaml)
CORS_ORIGINS=https://hackathon-one-seven.vercel.app,http://localhost:3000
HOST=0.0.0.0
PORT=10000
LOG_LEVEL=INFO
```

**Where to get your API keys:**

1. **Cohere API Key:**
   - Go to https://dashboard.cohere.com/api-keys
   - Copy your API key
   - Format: `M07aZwvDSMU12ALuBEvoNBLdz5gJxdEGbeFZyrpY`

2. **Claude API Key:**
   - Go to https://console.anthropic.com/settings/keys
   - Create new key or copy existing
   - Format: `sk-ant-api...`

3. **Qdrant:**
   - Already have: `https://10408476-48ab-4f7a-9768-8010c527acb8.us-east4-0.gcp.cloud.qdrant.io`
   - API Key: Already in your backend/.env

4. **Neon Postgres:**
   - Already have connection string in backend/.env
   - Format: `postgresql://user:pass@host/db?sslmode=require`

#### 1.5 Deploy
1. Click **"Create Web Service"**
2. Wait for deployment (5-10 minutes)
3. Watch build logs for any errors
4. Once deployed, note your backend URL:
   ```
   https://hackathon-rag-backend.onrender.com
   ```

#### 1.6 Verify Backend is Running

Test the health endpoint:
```bash
curl https://hackathon-rag-backend.onrender.com/health
```

Expected response:
```json
{
  "status": "ok",
  "message": "RAG chatbot is running",
  "database_connected": true
}
```

---

### **Step 2: Update Frontend Configuration**

#### 2.1 Get Your Backend URL

After Render deployment completes, copy your backend URL:
```
https://hackathon-rag-backend.onrender.com
```

#### 2.2 Update Docusaurus Config

The file `docusaurus.config.js` has been pre-configured with the expected Render URL.

**If your Render URL is different**, update line 33:

```javascript
customFields: {
  backendUrl: 'https://YOUR-ACTUAL-RENDER-URL.onrender.com',
},
```

#### 2.3 Redeploy Frontend

**Option A: Automatic (if Vercel auto-deploy is enabled)**
- Push changes to GitHub
- Vercel will auto-deploy

**Option B: Manual**
1. Go to Vercel dashboard
2. Find your project: `Hackathon-One`
3. Click **"Redeploy"**

---

### **Step 3: Test on Mobile**

#### 3.1 Open Your Site on Mobile
- Open browser on your phone
- Navigate to: https://hackathon-one-seven.vercel.app
- Look for chatbot widget in bottom-right corner

#### 3.2 Test Chatbot
1. Click chatbot icon
2. Type: "What is Physical AI?"
3. Wait for response (first request may be slow on free plan)
4. Verify answer appears with source citations

#### 3.3 Check Browser Console (if issues)
- On mobile Chrome: Settings → More tools → Remote debugging
- Look for errors related to backend connection

---

## 🔧 Troubleshooting

### Issue 1: "Failed to fetch" error

**Cause:** Backend not accessible or CORS issue

**Solution:**
1. Check backend is running: Visit `https://your-backend.onrender.com/health`
2. Verify CORS_ORIGINS includes your Vercel URL
3. Check Render logs for errors

### Issue 2: Slow response (30+ seconds)

**Cause:** Render free plan cold start

**Solution:**
- First request after inactivity takes 30-60 seconds (normal)
- Subsequent requests are fast
- Upgrade to paid plan to prevent spin-down

### Issue 3: "Backend URL not defined"

**Cause:** Frontend not updated with backend URL

**Solution:**
1. Check `docusaurus.config.js` has correct backend URL
2. Rebuild and redeploy frontend

### Issue 4: Chatbot doesn't open

**Cause:** JavaScript error in browser

**Solution:**
1. Open browser console (F12)
2. Look for errors
3. Check if `process is not defined` error appears
4. Should be fixed with latest commit

---

## 💰 Cost Estimate

### Free Tier (Current Setup)
- **Render Free:** $0/month
  - 750 hours/month
  - Spins down after 15 min inactivity
  - Cold starts: 30-60 seconds

- **Vercel Free:** $0/month
  - 100 GB bandwidth/month
  - Unlimited requests

- **Cohere Free:** $0/month
  - 100 API calls/month

- **Qdrant Cloud Free:** $0/month
  - 1 GB storage
  - 1M vectors

- **Neon Free:** $0/month
  - 512 MB storage

**Total: $0/month** ✅

### Recommended Production Setup
- **Render Starter:** $7/month
  - Always on (no cold starts)
  - Better performance

- **Total: ~$7/month**

---

## 📊 Monitoring

### Check Backend Health
```bash
# Health check
curl https://hackathon-rag-backend.onrender.com/health

# Example query
curl -X POST https://hackathon-rag-backend.onrender.com/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}'
```

### View Logs
1. Go to Render dashboard
2. Select your service
3. Click **"Logs"** tab
4. Monitor for errors or slow queries

---

## 🎉 Success Checklist

- [ ] Backend deployed to Render
- [ ] Backend health check returns 200 OK
- [ ] Frontend `docusaurus.config.js` updated with backend URL
- [ ] Frontend redeployed to Vercel
- [ ] Chatbot opens on mobile browser
- [ ] Test query returns book-related answer
- [ ] Check browser console shows no errors

---

## 📝 Quick Reference

**Your URLs:**
- Frontend: https://hackathon-one-seven.vercel.app
- Backend: https://hackathon-rag-backend.onrender.com
- Backend Health: https://hackathon-rag-backend.onrender.com/health

**Your Services:**
- Render: https://dashboard.render.com
- Vercel: https://vercel.com/dashboard
- Qdrant: https://cloud.qdrant.io
- Neon: https://console.neon.tech

**Support:**
- Render Docs: https://render.com/docs
- Vercel Docs: https://vercel.com/docs
- Project Issues: https://github.com/khan561hunter/Hackathon-One/issues

---

## 🔄 Future Improvements

1. **Add Redis caching** - Faster responses for common queries
2. **Enable streaming** - Real-time response generation
3. **Add analytics** - Track usage and popular questions
4. **Upgrade to paid plans** - Eliminate cold starts
5. **Add rate limiting** - Prevent API abuse
6. **Set up monitoring** - Uptime alerts and error tracking

---

*Last updated: 2025-12-10*
*Questions? Open an issue on GitHub*
