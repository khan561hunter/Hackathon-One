---
id: 006
title: Remove ngrok infrastructure from project
stage: general
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5
feature: none
branch: main
user: developer
command: none
labels: ["infrastructure", "cleanup", "ngrok", "deployment"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - src/components/Chatbot/index.jsx
  - backend/.env
tests:
  - Verified no ngrok processes running on port 4040
  - Checked git status for modified files
---

## Prompt

remove the ngrok process that u started yesterday and make the changes

## Response snapshot

**Task: Remove all ngrok references from the project**

1. **Checked for running ngrok processes:**
   - Verified no processes on port 4040 (ngrok web interface)
   - Verified no processes on port 8000 (backend)
   - Result: No ngrok processes were running

2. **Removed ngrok URL from frontend (src/components/Chatbot/index.jsx:19):**
   - **Before:**
   ```javascript
   const BACKEND_URL = typeof window !== 'undefined' && window.location.hostname === 'localhost'
     ? 'http://localhost:8000'
     : 'https://38aaaf1fe32c.ngrok-free.app'; // ngrok tunnel URL (temporary)
   ```
   - **After:**
   ```javascript
   const BACKEND_URL = typeof window !== 'undefined' && window.location.hostname === 'localhost'
     ? 'http://localhost:8000'
     : process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
   ```

3. **Removed ngrok URL from backend CORS (backend/.env:24):**
   - **Before:**
   ```
   CORS_ORIGINS=http://localhost:3000,http://localhost:3001,https://hackathon-one-seven.vercel.app,https://38aaaf1fe32c.ngrok-free.app
   ```
   - **After:**
   ```
   CORS_ORIGINS=http://localhost:3000,http://localhost:3001,https://hackathon-one-seven.vercel.app
   ```

## Outcome

- ✅ Impact: Removed temporary ngrok infrastructure. System now uses proper environment variable configuration for production deployments.
- 🧪 Tests: Verified no ngrok processes running, confirmed file modifications
- 📁 Files: Modified 2 files (frontend component, backend env)
- 🔁 Next prompts: Configure production backend URL via environment variables when deploying
- 🧠 Reflection: Ngrok URLs should never be hardcoded in production code. Using environment variables provides proper configuration flexibility.

## Evaluation notes (flywheel)

- Failure modes observed: Hardcoded ngrok URL in frontend code is a deployment anti-pattern
- Graders run and results (PASS/FAIL): Process check PASS, File modification PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add validation to prevent hardcoded URLs in CI/CD pipeline
