# Setup Guide - RAG Chatbot

Complete step-by-step setup instructions for development environment.

## Prerequisites

### Required Accounts & API Keys

1. **Cohere API** (for embeddings - PRIMARY)
   - Sign up at: https://cohere.com/
   - Get API key from dashboard
   - Model: `embed-english-v3.0` (1024 dimensions)
   - Free tier: 100 requests/minute

2. **Anthropic API** (for Claude 4.5 Sonnet - OPTIONAL)
   - Sign up at: https://console.anthropic.com/
   - Get API key ($5 free credit for new accounts)
   - Model: `claude-sonnet-4-20250514`
   - Note: System works without this (uses simple fallback generator)

3. **Qdrant Cloud** (vector database - REQUIRED)
   - Sign up at: https://cloud.qdrant.io/
   - Create a free cluster
   - Note: cluster URL and API key
   - Free tier: 1GB storage

4. **Neon Postgres** (metadata storage - REQUIRED)
   - Sign up at: https://neon.tech/
   - Create a free project
   - Copy connection string
   - Free tier: 512MB storage

### Required Software

- Python 3.9 or higher
- Node.js 16 or higher
- Git
- pip (Python package manager)
- npm or yarn (Node package manager)

## Step 1: Clone and Setup Project

```bash
# If not already in the project directory
cd E:\chatbot\practice

# Verify structure
ls
# Should see: backend/, src/, docs/, README.md, etc.
```

## Step 2: Backend Setup

### 2.1 Install Python Dependencies

```bash
cd backend

# Create virtual environment (recommended)
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 2.2 Configure Environment Variables

```bash
# Copy example file
copy .env.example .env  # Windows
# cp .env.example .env  # macOS/Linux

# Edit .env file with your actual credentials
notepad .env  # Windows
# nano .env  # macOS/Linux
```

Fill in your `.env` file:

```env
# Cohere Embeddings API (REQUIRED)
COHERE_API_KEY=your_actual_cohere_api_key_here

# Anthropic Claude API (OPTIONAL - for AI-generated answers)
ANTHROPIC_API_KEY=sk-ant-your_actual_claude_key_here

# Qdrant Cloud (REQUIRED)
QDRANT_API_KEY=your_actual_qdrant_api_key_here
QDRANT_URL=https://your-cluster-id.us-east-1-0.aws.cloud.qdrant.io:6333

# Neon Serverless Postgres (REQUIRED)
NEON_DB_URL=postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/dbname?sslmode=require

# FastAPI Settings
HOST=0.0.0.0
PORT=8000
CORS_ORIGINS=http://localhost:3000,http://localhost:3001,https://hackathon-one-seven.vercel.app

# Logging
LOG_LEVEL=INFO

# Docs Path (adjust to your docs folder location)
DOCS_PATH=../docs
```

### 2.3 Test Database Connection

```bash
# Test Neon Postgres connection
python init_database.py
```

If successful, you should see:
```
INFO:     Connected to Postgres
INFO:     Schema initialized successfully
INFO:     Database initialization complete!
```

### 2.4 Test Qdrant Connection

```bash
python -c "from app.qdrant_setup import get_qdrant_manager; q = get_qdrant_manager(); q.create_collection(); print('✓ Qdrant connected!')"
```

If successful, you should see: `✓ Qdrant connected!`

## Step 3: Upload Documentation Embeddings

This is a **one-time setup** (unless you update your documentation).

### 3.1 Verify Docs Path

Make sure your documentation exists:

```bash
# From backend directory
cd ..
ls docs/  # Should show your .md and .mdx files
cd backend
```

If your docs are elsewhere, update `DOCS_PATH` in `.env`.

### 3.2 Create Upload Script

Create `backend/scripts/upload_embeddings.py`:

```python
"""
Upload documentation embeddings to Qdrant
"""
import asyncio
import os
from pathlib import Path
from app.embeddings_cohere import get_cohere_embeddings
from app.qdrant_setup import get_qdrant_manager
from app.utils import chunk_text, load_markdown_files

async def main():
    print("=" * 60)
    print("RAG Embeddings Upload Script")
    print("=" * 60)

    # Load markdown files
    docs_path = os.getenv("DOCS_PATH", "../docs")
    print(f"Loading markdown files from: {docs_path}")

    docs = load_markdown_files(docs_path)
    print(f"Loaded {len(docs)} markdown files\n")

    # Chunk documents
    all_chunks = []
    all_metadata = []

    for doc in docs:
        chunks = chunk_text(doc["content"], chunk_size=500, overlap=50)
        for i, chunk in enumerate(chunks):
            all_chunks.append(chunk)
            all_metadata.append({
                "doc_name": doc["name"],
                "chunk_index": i,
                "total_chunks": len(chunks)
            })

    print(f"Total chunks: {len(all_chunks)}\n")

    # Generate embeddings
    print("Generating embeddings with Cohere...")
    embeddings = get_cohere_embeddings()
    vectors = embeddings.embed_documents(all_chunks)
    print(f"Generated {len(vectors)} embeddings\n")

    # Upload to Qdrant
    print("Uploading to Qdrant...")
    qdrant = get_qdrant_manager()

    # Create collection
    recreate = input("Recreate collection? (y/N): ").lower() == 'y'
    if recreate:
        try:
            qdrant.delete_collection()
            print("Deleted existing collection")
        except:
            pass

    qdrant.create_collection()

    # Upload embeddings
    qdrant.upload_embeddings(
        embeddings=vectors,
        texts=all_chunks,
        metadata_list=all_metadata
    )

    # Verify
    info = qdrant.get_collection_info()
    print(f"\nCollection info:")
    print(f"  Name: {info['name']}")
    print(f"  Points: {info['points_count']}")
    print(f"  Vectors: {info['vectors_count']}")
    print("\n" + "=" * 60)
    print("✓ SUCCESS! Embeddings uploaded to Qdrant")
    print("=" * 60)

if __name__ == "__main__":
    asyncio.run(main())
```

### 3.3 Create Scripts Directory

```bash
# From backend directory
mkdir scripts
cd scripts
```

### 3.4 Run Upload Script

```bash
python upload_embeddings.py
```

When prompted:
- "Recreate Qdrant collection? (y/N):" → Type `y` for first run

**Expected output:**
```
============================================================
RAG Embeddings Upload Script
============================================================
Loading markdown files from: ../docs
Loaded 25 markdown files

Total chunks: 127

Generating embeddings with Cohere...
Generated 127 embeddings

Uploading to Qdrant...
Recreate collection? (y/N): y
Deleted existing collection

Collection info:
  Name: documentation_chunks
  Points: 127
  Vectors: 127

============================================================
✓ SUCCESS! Embeddings uploaded to Qdrant
============================================================
```

### 3.5 Troubleshooting Upload

**Error: "COHERE_API_KEY not found"**
- Solution: Make sure `.env` file has `COHERE_API_KEY=your_key`

**Error: "No markdown files found"**
- Solution: Check `DOCS_PATH` in `.env` points to correct directory
- Verify docs folder contains .md or .mdx files

**Error: "Cohere API request failed"**
- Solution: Verify Cohere API key is valid
- Check API rate limits (100 requests/minute on free tier)

**Error: "Qdrant connection failed"**
- Solution: Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
- Check Qdrant cluster is running

## Step 4: Start Backend Server

```bash
# From backend directory
cd ..  # if in scripts/
python main.py
```

**Expected output:**
```
INFO:     Starting up RAG chatbot backend...
INFO:     Connected to Postgres database
INFO:     Database schema initialized successfully
INFO:     Initialized RAG pipeline with Cohere embeddings and simple text generation
INFO:     Backend startup complete!
INFO:     Uvicorn running on http://0.0.0.0:8000
```

Backend is now running at: http://localhost:8000

### 4.1 Test Backend Endpoints

Open a new terminal and test:

```bash
# Health check
curl http://localhost:8000/health
# Expected: {"status":"ok","message":"RAG chatbot is running","database_connected":true}

# Test question
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d "{\"question\": \"What is Physical AI?\", \"stream\": false}"

# Expected: JSON response with answer and sources
```

## Step 5: Frontend Setup (Docusaurus)

The chatbot is already integrated into this Docusaurus project!

### 5.1 Install Frontend Dependencies

```bash
# From project root
cd ..  # if in backend/
npm install
```

### 5.2 Start Docusaurus

```bash
npm start
```

Docusaurus runs at: http://localhost:3000

### 5.3 Test Frontend

1. Open http://localhost:3000 in browser
2. Look for floating chat button (💬) in bottom-right corner
3. Click to open chat
4. Type a question and press Enter
5. Highlight text on page → "Ask About Selection" button should appear

## Step 6: Verify Full System

### Test RAG Workflow

1. **Open chat widget** on http://localhost:3000
2. **Ask a question** related to your docs (e.g., "What is Physical AI?")
3. **Wait for response** (should take 2-3 seconds)
4. **Check sources** shown below the answer

Expected flow:
- Question embedded with Cohere ✅
- Top 3 chunks retrieved from Qdrant ✅
- Answer formatted with retrieved context ✅
- Sources displayed with relevance scores ✅

### Test Selected Text Workflow

1. **Highlight text** on any page (at least 10 characters)
2. **Click "Ask About Selection"** button that appears
3. **Type question** about the selected text
4. **Get answer** based only on selection (no retrieval)

Expected flow:
- Selected text + question sent to backend ✅
- Answer generated directly ✅
- No RAG retrieval ✅

### Check Database Logs

```bash
curl http://localhost:8000/stats
# Expected: {"total_chats": X, "total_selections": Y}

curl http://localhost:8000/chat-logs?limit=5
# Expected: JSON array of recent interactions
```

## Common Issues

### Issue: Backend won't start

**Symptoms:**
- Import errors
- Module not found errors

**Solutions:**
1. Ensure virtual environment is activated: `venv\Scripts\activate`
2. Reinstall dependencies: `pip install -r requirements.txt`
3. Check Python version: `python --version` (should be 3.9+)

### Issue: CORS errors in browser

**Symptoms:**
- Console error: "blocked by CORS policy"
- Requests fail from frontend

**Solutions:**
1. Add `http://localhost:3000` to `CORS_ORIGINS` in backend `.env`
2. Restart backend after changing `.env`
3. Clear browser cache

### Issue: "No relevant information found"

**Symptoms:**
- Every question returns "couldn't find relevant information"

**Solutions:**
1. Verify embeddings were uploaded: Check Qdrant collection has points
2. Re-run embeddings upload script
3. Check `DOCS_PATH` points to correct folder
4. Verify Cohere API key is valid

### Issue: Chatbot not visible

**Symptoms:**
- No chat button on page

**Solutions:**
1. Check browser console for errors
2. Verify `src/theme/Root.js` exists and imports Chatbot
3. Clear Docusaurus cache: `npm run clear` or `yarn clear`
4. Restart Docusaurus dev server

### Issue: "Database connection failed"

**Symptoms:**
- Backend fails to start
- Error mentions Postgres or Neon

**Solutions:**
1. Verify `NEON_DB_URL` format is correct
2. Check Neon project is active (not paused)
3. Test connection string in a Postgres client
4. Ensure SSL mode is included: `?sslmode=require`

## Advanced Configuration

### Using Claude for AI-Generated Answers

The system currently uses a simple fallback generator. To enable Claude:

1. Get Anthropic API key from https://console.anthropic.com/
2. Add to `.env`: `ANTHROPIC_API_KEY=sk-ant-your_key_here`
3. Edit `backend/app/rag.py` line 44:
   ```python
   # Change from:
   self.generator = get_simple_generator()

   # To:
   from .llm_claude import get_claude_generator
   self.generator = get_claude_generator()
   ```
4. Restart backend

### Adjusting Retrieval Settings

Edit `backend/app/rag.py` lines 28-30:

```python
def __init__(
    self,
    top_k: int = 3,              # Number of chunks to retrieve
    score_threshold: float = 0.0, # Minimum similarity score (0.0 = no filter)
    max_context_length: int = 4000 # Max characters for context
):
```

### Customizing Chunk Size

Edit `backend/scripts/upload_embeddings.py`:

```python
chunks = chunk_text(doc["content"],
                   chunk_size=500,  # Characters per chunk
                   overlap=50)       # Overlap between chunks
```

## Next Steps

✅ Backend running
✅ Embeddings uploaded
✅ Frontend integrated
✅ System tested

Now you're ready to:
1. **Deploy to production** - See [DEPLOYMENT.md](./DEPLOYMENT.md)
2. **Customize the UI** - See [DOCUSAURUS_INTEGRATION.md](./DOCUSAURUS_INTEGRATION.md)
3. **Learn the API** - See [API.md](./API.md)

## Development Tips

### Hot Reload

- **Backend**: FastAPI auto-reloads on file changes
- **Frontend**: Docusaurus auto-reloads on component changes

### Debugging

**Backend logs:**
```bash
# Backend logs show in terminal where you ran `python main.py`
# Look for INFO, WARNING, ERROR messages
```

**Frontend console:**
```javascript
// Open browser DevTools (F12)
// Check Console tab for errors
// Check Network tab for API requests
```

### Re-uploading Embeddings

When you update documentation:

```bash
cd backend/scripts
python upload_embeddings.py
# Choose 'N' when asked to recreate (updates existing points)
```

### Database Reset

If you need to reset the database:

```bash
cd backend
python init_database.py
```

This recreates tables if they don't exist.

### Qdrant Reset

To completely reset Qdrant:

```bash
cd backend/scripts
python upload_embeddings.py
# Choose 'y' when asked to recreate collection
```

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    User Interface                        │
│              (Docusaurus + React Chatbot)               │
└──────────────────┬──────────────────────────────────────┘
                   │ HTTP Requests
                   ▼
┌─────────────────────────────────────────────────────────┐
│                 FastAPI Backend                          │
│  ┌──────────┐  ┌──────────┐  ┌───────────┐            │
│  │ /ask     │  │/ask-      │  │ /health   │            │
│  │ endpoint │  │selected   │  │ /stats    │            │
│  └────┬─────┘  └─────┬────┘  └───────────┘            │
│       │              │                                   │
│  ┌────▼──────────────▼──────┐                          │
│  │    RAG Pipeline           │                          │
│  │  1. Cohere Embeddings     │                          │
│  │  2. Qdrant Retrieval      │                          │
│  │  3. Simple/Claude Gen     │                          │
│  └────┬──────────────────────┘                          │
└───────┼──────────────────────────────────────────────────┘
        │
   ┌────▼────┐     ┌─────────┐
   │ Qdrant  │     │  Neon   │
   │ Vector  │     │Postgres │
   │   DB    │     │  Logs   │
   └─────────┘     └─────────┘
```

---

**Setup complete!** 🎉

You now have a fully functional RAG chatbot system.

For production deployment, see [DEPLOYMENT.md](./DEPLOYMENT.md).
