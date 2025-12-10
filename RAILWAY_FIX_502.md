# 🚨 Fix Railway 502 Error - Backend Not Starting

## Current Status
- ✅ Railway URL: `https://hackathon-one-production.up.railway.app`
- ❌ Health check returns: `502 Application failed to respond`

## Root Cause
Backend application is not starting on Railway. This is usually due to **missing environment variables**.

---

## Fix Steps (Do These Now)

### Step 1: Check Railway Dashboard

1. Go to: https://railway.app/project/9dce8dd9-c5b4-4bf8-ac07-031c80563c16
2. Click on your backend service (Python icon)

### Step 2: Verify Root Directory Setting ⚠️ CRITICAL

Go to **Settings** → **General**:
- **Root Directory** MUST be set to: `backend`
- If it's empty or different, set it to `backend` and redeploy

### Step 3: Add ALL Environment Variables

Click **"Variables"** tab and add these (click "+ New Variable" for each):

```
COHERE_API_KEY=M07aZwvDSMU12ALuBEvoNBLdz5gJxdEGbeFZyrpY
CLAUDE_API_KEY=sk_cr_JAndxz6VsyVjTw3BP7uLkWnqgjXVCHe4jzECQmcero2H
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.PS7GPcFQrRXyjbAJlOx-NbCi_NY0BcRFT4_OHTr7l7k
QDRANT_URL=https://10408476-48ab-4f7a-9768-8010c527acb8.us-east4-0.gcp.cloud.qdrant.io
QDRANT_COLLECTION=book_embeddings
NEON_DB_URL=postgresql://neondb_owner:npg_AHvjL6gpK8hD@ep-shy-scene-a4ylx9l0-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
CORS_ORIGINS=https://hackathon-one-seven.vercel.app,http://localhost:3000
HOST=0.0.0.0
LOG_LEVEL=INFO
```

**Note:** Railway automatically provides the `PORT` variable, so don't add it manually.

### Step 4: Verify Build Configuration

Go to **Settings** → Check:

- **Builder:** NIXPACKS ✅
- **Watch Paths:** Should auto-detect from `nixpacks.toml`
- **Start Command:** `uvicorn main:app --host 0.0.0.0 --port $PORT`

### Step 5: Redeploy

After setting environment variables:
1. Go to **"Deployments"** tab
2. Click the **three dots (⋮)** on latest deployment
3. Click **"Redeploy"**
4. Wait 2-3 minutes

### Step 6: Check Build Logs

While deploying:
1. Click on the deployment (in "Deployments" tab)
2. Click **"View Logs"** or **"Build Logs"**
3. Look for these success messages:
   ```
   ✅ Installing dependencies from requirements.txt
   ✅ Successfully installed packages
   ✅ Starting application
   ✅ Uvicorn running on http://0.0.0.0:XXXX
   ✅ Application startup complete
   ```

4. If you see errors, check for:
   ```
   ❌ ModuleNotFoundError
   ❌ Connection refused
   ❌ PORT not found
   ❌ Environment variable not set
   ```

### Step 7: Test Health Check

After deployment completes (shows green ✅):

```bash
curl https://hackathon-one-production.up.railway.app/health
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

## Common Issues and Solutions

### Issue: Root Directory Not Set
**Symptom:** Railway tries to build from project root (Node.js error)
**Solution:** Settings → General → Root Directory = `backend`

### Issue: Missing Environment Variables
**Symptom:** 502 error, logs show "Environment variable not found"
**Solution:** Add all 9 environment variables listed above

### Issue: Wrong Start Command
**Symptom:** App builds but doesn't start
**Solution:** Verify start command uses `$PORT` (not hardcoded port)

### Issue: Database Connection Error
**Symptom:** Health check fails on database_connected
**Solution:** Verify NEON_DB_URL is correct (no extra spaces)

### Issue: Import Errors
**Symptom:** ModuleNotFoundError in logs
**Solution:** Verify requirements.txt is in backend folder

---

## Verification Checklist

After deployment succeeds, verify these:

- [ ] Health check returns 200 OK
- [ ] `/health` endpoint shows `database_connected: true`
- [ ] Test query works:
  ```bash
  curl -X POST https://hackathon-one-production.up.railway.app/ask \
    -H "Content-Type: application/json" \
    -d '{"question": "What is Physical AI?"}'
  ```
- [ ] Response includes retrieved sources
- [ ] No errors in Railway logs

---

## If Still Not Working

1. **Share Railway logs with me:**
   - Copy the full deployment logs
   - Send them to me so I can diagnose the exact issue

2. **Check Railway status page:**
   - https://railway.statuspage.io/
   - Ensure Railway services are operational

3. **Verify Qdrant/Neon connectivity:**
   - Test if Qdrant API key is still valid
   - Test if Neon database is accessible

---

## After It Works

Once health check returns OK:

1. Update frontend: Edit `docusaurus.config.js` line 32
2. Commit: `git add . && git commit -m "Add Railway backend URL"`
3. Push: `git push origin main`
4. Vercel auto-redeploys frontend
5. Test chatbot on: https://hackathon-one-seven.vercel.app
6. Share with others! 🎉

---

## Quick Debug Commands

```bash
# Test health check
curl https://hackathon-one-production.up.railway.app/health

# Test RAG query
curl -X POST https://hackathon-one-production.up.railway.app/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS2?"}'

# Test CORS (from browser console)
fetch('https://hackathon-one-production.up.railway.app/health')
  .then(r => r.json())
  .then(console.log)
```

---

**Current Action Needed:**
1. Go to Railway dashboard
2. Add all 9 environment variables
3. Verify root directory is set to `backend`
4. Redeploy
5. Check logs for errors
6. Test health check
