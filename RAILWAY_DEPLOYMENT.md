# 🚂 Deploy Backend to Railway.app (Free)

## Why Railway?
- ✅ **$5 free credits per month** (enough for small projects)
- ✅ No credit card required to start
- ✅ Automatic deployments from GitHub
- ✅ Easy environment variables setup
- ✅ Always-on (no cold starts)
- ✅ Custom domain support

---

## Step-by-Step Deployment

### Step 1: Create Railway Account

1. Go to https://railway.app
2. Click **"Start a New Project"**
3. Sign up with GitHub (easiest option)
4. Authorize Railway to access your repositories

### Step 2: Create New Project

1. Click **"New Project"**
2. Select **"Deploy from GitHub repo"**
3. Choose your repository: `khan561hunter/Hackathon-One`
4. Railway will detect it's a Python project

### Step 3: Configure Project

**Root Directory:**
- Set root directory to `backend`
- Railway Settings → General → Root Directory: `backend`

**Build Command:**
- Railway auto-detects: `pip install -r requirements.txt`
- If not, set manually in Settings

**Start Command:**
```bash
uvicorn main:app --host 0.0.0.0 --port $PORT
```

**Health Check Path:**
```
/health
```

### Step 4: Add Environment Variables

Go to **Variables** tab and add:

```bash
# AI API Keys
COHERE_API_KEY=M07aZwvDSMU12ALuBEvoNBLdz5gJxdEGbeFZyrpY
CLAUDE_API_KEY=sk_cr_JAndxz6VsyVjTw3BP7uLkWnqgjXVCHe4jzECQmcero2H

# Qdrant
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.PS7GPcFQrRXyjbAJlOx-NbCi_NY0BcRFT4_OHTr7l7k
QDRANT_URL=https://10408476-48ab-4f7a-9768-8010c527acb8.us-east4-0.gcp.cloud.qdrant.io
QDRANT_COLLECTION=book_embeddings

# Neon Postgres
NEON_DB_URL=postgresql://neondb_owner:npg_AHvjL6gpK8hD@ep-shy-scene-a4ylx9l0-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require

# CORS (add your Vercel URL)
CORS_ORIGINS=https://hackathon-one-seven.vercel.app,http://localhost:3000,http://localhost:3001

# Server Config (Railway provides PORT automatically)
HOST=0.0.0.0
LOG_LEVEL=INFO
```

### Step 5: Deploy

1. Click **"Deploy"**
2. Wait 2-3 minutes for build
3. Railway will provide a URL like: `https://hackathon-backend-production.up.railway.app`

### Step 6: Get Your Backend URL

1. Go to **Settings** → **Networking**
2. Click **"Generate Domain"**
3. Copy your Railway domain (e.g., `https://hackathon-backend-production.up.railway.app`)

### Step 7: Test Backend

```bash
curl https://YOUR-RAILWAY-URL.up.railway.app/health
```

Expected response:
```json
{
  "status": "ok",
  "message": "RAG chatbot is running",
  "database_connected": true
}
```

### Step 8: Update Frontend

Edit `docusaurus.config.js` line 32:

```javascript
backendUrl: process.env.BACKEND_URL || 'https://YOUR-RAILWAY-URL.up.railway.app',
```

### Step 9: Commit and Push

```bash
git add docusaurus.config.js
git commit -m "Update backend URL to Railway deployment"
git push origin main
```

Vercel will auto-redeploy your frontend with the new backend URL.

---

## Alternative Free Platforms

### Option 2: Fly.io (Free Tier)

**Pros:**
- Free tier: 3 shared-cpu VMs
- Always-on
- Global edge network

**Steps:**
1. Install Fly CLI: `powershell -Command "iwr https://fly.io/install.ps1 -useb | iex"`
2. `fly auth signup`
3. Create `fly.toml` in backend folder
4. `fly launch` (select region closest to you)
5. Set secrets: `fly secrets set COHERE_API_KEY=xxx`
6. `fly deploy`

**fly.toml:**
```toml
app = "hackathon-backend"

[build]
  builder = "paketobuildpacks/builder:base"

[env]
  PORT = "8080"

[[services]]
  internal_port = 8080
  protocol = "tcp"

  [[services.ports]]
    port = 80
    handlers = ["http"]

  [[services.ports]]
    port = 443
    handlers = ["tls", "http"]

  [[services.http_checks]]
    interval = 10000
    timeout = 2000
    grace_period = "5s"
    method = "get"
    path = "/health"
```

---

### Option 3: Vercel Serverless (Same as Frontend)

**Pros:**
- Same platform as your frontend
- Generous free tier
- Zero config for Python

**Cons:**
- 10-second execution limit per request
- Cold starts
- May timeout on slow queries

**Steps:**
1. Create `vercel.json` in project root:
```json
{
  "builds": [
    {
      "src": "backend/main.py",
      "use": "@vercel/python"
    }
  ],
  "routes": [
    {
      "src": "/api/(.*)",
      "dest": "backend/main.py"
    }
  ]
}
```

2. Install Vercel CLI: `npm i -g vercel`
3. `vercel login`
4. `vercel --prod`
5. Add environment variables in Vercel dashboard

---

### Option 4: PythonAnywhere (Free)

**Pros:**
- Specifically for Python apps
- Always free tier
- Easy setup

**Cons:**
- Slower than other options
- Limited to 1 web app

**Steps:**
1. Sign up at https://www.pythonanywhere.com
2. Go to **Web** tab
3. **Add a new web app**
4. Select **Flask** (closest to FastAPI)
5. Upload your code via Git
6. Configure WSGI file
7. Set environment variables in **Environment** tab

---

## Recommended: Railway.app

**Why Railway is best for you:**

1. **$5/month free credits** - More than enough for development
2. **No credit card** to start
3. **Auto-deploy** from GitHub
4. **Always-on** - No cold starts
5. **Easy setup** - Just connect GitHub repo

**Monthly usage estimate:**
- Small FastAPI app: ~$2-3/month
- Your free $5 credits cover this!

---

## Cost Comparison

| Platform | Free Tier | Always On | Cold Start | Setup Difficulty |
|----------|-----------|-----------|------------|------------------|
| **Railway** | $5/month credits | ✅ | None | Easy |
| **Fly.io** | 3 VMs free | ✅ | None | Medium |
| **Render** | 750 hrs/month | ❌ | 30-60s | Easy |
| **Vercel** | Unlimited | ❌ | 1-2s | Easy |
| **PythonAnywhere** | 1 app free | ✅ | None | Medium |

**Best choice: Railway.app** ✅

---

## After Deployment Checklist

- [ ] Railway backend deployed
- [ ] Backend URL copied
- [ ] Environment variables added in Railway
- [ ] Health check returns 200 OK
- [ ] Updated `docusaurus.config.js` with Railway URL
- [ ] Committed and pushed to GitHub
- [ ] Vercel auto-redeployed frontend
- [ ] Tested chatbot on https://hackathon-one-seven.vercel.app
- [ ] Chatbot answers book questions correctly
- [ ] Others can access your chatbot now!

---

## Railway CLI (Optional)

Install Railway CLI for easier management:

```bash
# Install
npm i -g @railway/cli

# Login
railway login

# Link project
railway link

# View logs
railway logs

# Add environment variable
railway variables set COHERE_API_KEY=xxx

# Deploy
railway up
```

---

## Troubleshooting

### Issue: "Failed to build"
**Solution:** Check Railway logs, ensure `requirements.txt` is correct

### Issue: "Application failed to respond"
**Solution:** Verify start command uses `$PORT` variable provided by Railway

### Issue: CORS error
**Solution:** Add Vercel URL to `CORS_ORIGINS` environment variable in Railway

### Issue: Database connection failed
**Solution:** Verify `NEON_DB_URL` is set correctly in Railway variables

---

## Quick Start Commands

```bash
# 1. After Railway deployment, update frontend
cd /path/to/project
code docusaurus.config.js
# Change line 32 to your Railway URL

# 2. Commit and push
git add docusaurus.config.js
git commit -m "Update backend URL to Railway"
git push origin main

# 3. Test
curl https://YOUR-RAILWAY-URL.up.railway.app/health

# 4. Done! Share your Vercel URL with others
```

---

**Your chatbot will now work for everyone! 🎉**
