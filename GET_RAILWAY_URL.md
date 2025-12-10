# 🚂 Get Your Railway Backend URL

Your Railway project is deployed! Here's how to get the URL.

## Project Information
- **Project ID:** `9dce8dd9-c5b4-4bf8-ac07-031c80563c16`
- **Dashboard Link:** https://railway.app/project/9dce8dd9-c5b4-4bf8-ac07-031c80563c16

---

## Method 1: Railway Dashboard (Easiest - DO THIS NOW)

### Step-by-Step:

1. **Open your Railway project:**
   ```
   https://railway.app/project/9dce8dd9-c5b4-4bf8-ac07-031c80563c16
   ```

2. **Click on your service:**
   - You'll see your backend service (Python icon)
   - Click on it

3. **Go to Settings tab:**
   - Click "Settings" in the top menu

4. **Generate Domain (if not done):**
   - Scroll to **"Networking"** section
   - Click **"Generate Domain"** button
   - Railway will create a public URL like: `https://hackathon-backend-production.up.railway.app`

5. **Copy the URL:**
   - Copy the full URL (including https://)
   - Example: `https://your-project-name.up.railway.app`

6. **Test it:**
   - Open in browser: `https://YOUR-RAILWAY-URL/health`
   - Should return:
     ```json
     {
       "status": "ok",
       "message": "RAG chatbot is running",
       "database_connected": true
     }
     ```

---

## Method 2: Railway CLI (For Future Use)

Railway CLI is now installed on your system.

### Get URL via CLI:

```bash
# Link your project (one-time setup)
railway link 9dce8dd9-c5b4-4bf8-ac07-031c80563c16

# Get deployment info
railway status

# View logs
railway logs

# Get environment variables
railway variables
```

---

## What to Do After Getting the URL

### 1. Test Backend Health Check

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

### 2. Test RAG Query

```bash
curl -X POST https://YOUR-RAILWAY-URL.up.railway.app/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}'
```

### 3. Update Frontend Configuration

Once you have the Railway URL, update your frontend:

```bash
# Edit docusaurus.config.js
code docusaurus.config.js
```

Change line 32:
```javascript
backendUrl: 'https://YOUR-RAILWAY-URL.up.railway.app',
```

### 4. Commit and Push

```bash
git add docusaurus.config.js
git commit -m "Update backend URL to Railway deployment"
git push origin main
```

Vercel will auto-redeploy your frontend with the new backend URL.

---

## Troubleshooting

### Issue: "Generate Domain" button not showing
**Solution:** The domain may already be generated. Look for "Domains" section - your URL will be listed there.

### Issue: Health check returns 404
**Solution:**
1. Check Railway logs: Go to project → Deployments → Click latest deployment → View logs
2. Verify environment variables are set correctly
3. Ensure Root Directory is set to `backend`

### Issue: Backend not starting
**Solution:**
1. Go to Railway dashboard → Your service → Deployments
2. Check build logs for errors
3. Verify all environment variables are set (COHERE_API_KEY, CLAUDE_API_KEY, etc.)

---

## Quick Reference

**Dashboard:** https://railway.app/project/9dce8dd9-c5b4-4bf8-ac07-031c80563c16

**Environment Variables Checklist:**
- ✅ COHERE_API_KEY
- ✅ CLAUDE_API_KEY
- ✅ QDRANT_API_KEY
- ✅ QDRANT_URL
- ✅ QDRANT_COLLECTION
- ✅ NEON_DB_URL
- ✅ CORS_ORIGINS (must include Vercel URL)
- ✅ HOST (0.0.0.0)
- ✅ LOG_LEVEL (INFO)

**Expected URL Format:**
```
https://[your-service-name].up.railway.app
```

---

## Next Steps After Getting URL

1. ✅ Get Railway URL from dashboard
2. ✅ Test `/health` endpoint
3. ✅ Test `/ask` endpoint with book question
4. ✅ Update `docusaurus.config.js` with Railway URL
5. ✅ Push to GitHub (Vercel auto-redeploys)
6. ✅ Test chatbot on https://hackathon-one-seven.vercel.app
7. ✅ Share with others - it will work everywhere! 🎉

---

**Note:** The Railway URL is permanent and will work for everyone once you update your frontend configuration.
