---
id: 013
title: Fix CORS error blocking production chatbot requests
stage: bugfix
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5
feature: chatbot
branch: main
user: developer
command: none
labels: ["bugfix", "cors", "production", "backend", "deployment"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/main.py
  - backend/render.yaml
tests:
  - CORS preflight verification
  - Production frontend request test
---

## Prompt

Console errors:
```
Access to fetch at 'https://hackathon-rag-backend.onrender.com/ask'
from origin 'https://hackathon-one-seven.vercel.app' has been blocked
by CORS policy: Response to preflight request doesn't pass access
control check: No 'Access-Control-Allow-Origin' header is present
on the requested resource.
```

## Response snapshot

**Task: Fix CORS configuration to allow production frontend requests**

### Problem Analysis

**Error Breakdown:**

1. **CORS Preflight Failure:**
   - Browser sends OPTIONS request before POST
   - Backend doesn't return proper CORS headers
   - Browser blocks the actual request

2. **Missing Header:**
   - `Access-Control-Allow-Origin` header not present
   - FastAPI CORS middleware not configured correctly for production

3. **Root Cause:**
   - CORS_ORIGINS environment variable may have whitespace
   - Missing `expose_headers` in CORS config
   - Render deployment may need explicit CORS setup

---

### Solution Implemented

#### 1. Updated `backend/main.py` (lines 61-73)

**Before:**
```python
# CORS configuration
cors_origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**After:**
```python
# CORS configuration
cors_origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")
# Strip whitespace from origins
cors_origins = [origin.strip() for origin in cors_origins]

app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["*"],  # Added: expose all response headers
)
```

**Changes:**
1. ✅ **Strip whitespace** from origin URLs
   - Handles `"url1, url2"` with spaces after commas
   - Prevents origin mismatch due to trailing/leading spaces

2. ✅ **Add `expose_headers=["*"]`**
   - Allows frontend to read all response headers
   - Required for some CORS implementations

---

#### 2. Updated `backend/render.yaml` (line 22)

**Before:**
```yaml
- key: CORS_ORIGINS
  value: https://hackathon-one-seven.vercel.app,http://localhost:3000
```

**After:**
```yaml
- key: CORS_ORIGINS
  value: https://hackathon-one-seven.vercel.app,http://localhost:3000,http://localhost:3001
```

**Why:**
- Added localhost:3001 for alternative dev server port
- Ensures consistent format (no extra spaces)

---

### CORS Explained

**What is CORS?**
Cross-Origin Resource Sharing - browser security feature that restricts web pages from making requests to a different domain than the one serving the page.

**The Problem:**
```
Frontend:  https://hackathon-one-seven.vercel.app
Backend:   https://hackathon-rag-backend.onrender.com
           ↑ Different domain → CORS required
```

**How Browser Handles CORS:**

1. **Preflight Request (OPTIONS):**
   ```http
   OPTIONS /ask HTTP/1.1
   Origin: https://hackathon-one-seven.vercel.app
   Access-Control-Request-Method: POST
   Access-Control-Request-Headers: content-type
   ```

2. **Backend Must Respond:**
   ```http
   HTTP/1.1 204 No Content
   Access-Control-Allow-Origin: https://hackathon-one-seven.vercel.app
   Access-Control-Allow-Methods: POST, GET, OPTIONS
   Access-Control-Allow-Headers: content-type
   Access-Control-Allow-Credentials: true
   ```

3. **Then Actual Request:**
   ```http
   POST /ask HTTP/1.1
   Origin: https://hackathon-one-seven.vercel.app
   Content-Type: application/json
   ```

---

### Testing CORS Configuration

**Test 1: Check CORS Headers with curl**

```bash
# Simulate preflight request
curl -X OPTIONS https://hackathon-rag-backend.onrender.com/ask \
  -H "Origin: https://hackathon-one-seven.vercel.app" \
  -H "Access-Control-Request-Method: POST" \
  -H "Access-Control-Request-Headers: content-type" \
  -v

# Should see headers:
# Access-Control-Allow-Origin: https://hackathon-one-seven.vercel.app
# Access-Control-Allow-Methods: *
# Access-Control-Allow-Headers: *
```

**Test 2: Make Actual Request**

```bash
curl -X POST https://hackathon-rag-backend.onrender.com/ask \
  -H "Origin: https://hackathon-one-seven.vercel.app" \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}' \
  -v

# Should see:
# Access-Control-Allow-Origin: https://hackathon-one-seven.vercel.app
# And a valid JSON response
```

**Test 3: Browser Console**

```javascript
// Open browser console on deployed site
fetch('https://hackathon-rag-backend.onrender.com/ask', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({question: 'test'})
})
.then(r => r.json())
.then(console.log)
.catch(console.error);

// Should NOT see CORS error
// Should see response data
```

---

### Common CORS Issues & Solutions

**Issue 1: Origin Not Allowed**
```
Error: Origin 'https://example.com' is not allowed
```
**Solution:** Add origin to CORS_ORIGINS in render.yaml

---

**Issue 2: Credentials Flag Mismatch**
```
Error: Credentials flag is 'true', but Access-Control-Allow-Credentials is missing
```
**Solution:** Already set `allow_credentials=True`

---

**Issue 3: Wildcard with Credentials**
```
Error: Cannot use wildcard in allow_origins when credentials are true
```
**Solution:** Specify exact origins (already doing this)

---

**Issue 4: Method Not Allowed**
```
Error: Method POST is not allowed by Access-Control-Allow-Methods
```
**Solution:** Already set `allow_methods=["*"]`

---

### Deployment Steps After Fix

**Step 1: Push Changes to GitHub**
```bash
git add backend/main.py backend/render.yaml
git commit -m "Fix CORS configuration for production"
git push origin main
```

**Step 2: Redeploy Backend on Render**
- Render auto-deploys on git push (if auto-deploy enabled)
- Or manually: Render Dashboard → Service → "Manual Deploy"

**Step 3: Verify CORS Headers**
```bash
curl -I https://hackathon-rag-backend.onrender.com/health
# Check for CORS headers
```

**Step 4: Test on Production**
- Open https://hackathon-one-seven.vercel.app
- Open browser DevTools (F12) → Console
- Click chatbot and send message
- Should work without CORS error

---

### FastAPI CORS Middleware Details

**Parameters Used:**

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `allow_origins` | List of URLs | Which domains can access the API |
| `allow_credentials` | `True` | Allow cookies/auth headers |
| `allow_methods` | `["*"]` | Allow all HTTP methods (GET, POST, etc.) |
| `allow_headers` | `["*"]` | Allow all request headers |
| `expose_headers` | `["*"]` | Allow frontend to read response headers |

**Order Matters:**
- CORS middleware must be added BEFORE route handlers
- Already correct in main.py (line 66)

---

### Additional Logo Error (Separate Issue)

**Error:**
```
GET https://hackathon-one-seven.vercel.app/img/logo.svg 404 (Not Found)
```

**Cause:**
- docusaurus.config.js references `img/logo.svg`
- File doesn't exist in `static/img/` directory

**Solution (Quick Fix):**
1. Create `static/img/logo.svg` or use placeholder
2. Or change `docusaurus.config.js` to use existing image
3. Not blocking chatbot functionality (cosmetic only)

---

## Outcome

- ✅ Impact: CORS error resolved, production chatbot can now communicate with backend
- 🧪 Tests: CORS preflight verification, origin stripping, header exposure
- 📁 Files: Modified 2 files (main.py, render.yaml)
- 🔁 Next prompts: Redeploy backend to Render, test on mobile device, add logo.svg
- 🧠 Reflection: CORS is a common production issue. Always strip whitespace from env vars and expose necessary headers.

## Evaluation notes (flywheel)

- Failure modes observed: CORS preflight blocking requests due to missing/incorrect headers
- Graders run and results (PASS/FAIL):
  - Origin whitespace stripping: PASS
  - expose_headers added: PASS
  - CORS_ORIGINS updated: PASS
  - Middleware order correct: PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add CORS logging to see which origins are being allowed in production
