# Quick Start Guide

Get the RAG chatbot running in 15 minutes!

## Prerequisites

- Python 3.9+
- API keys ready (Cohere, Claude, Qdrant, Neon)
- Your Docusaurus docs folder

## Step 1: Install Dependencies (2 minutes)

```bash
cd backend
pip install -r requirements.txt
```

## Step 2: Configure Environment (3 minutes)

```bash
# Copy and edit .env
copy .env.example .env  # Windows
# cp .env.example .env  # macOS/Linux

# Edit .env with your API keys
notepad .env
```

Required values:
- `COHERE_API_KEY` - From Cohere Dashboard
- `ANTHROPIC_API_KEY` - From Anthropic Console (optional, for AI-generated answers)
- `QDRANT_API_KEY` + `QDRANT_URL` - From Qdrant Cloud
- `NEON_DB_URL` - From Neon
- `DOCS_PATH` - Path to your docs folder (e.g., `../docs`)

## Step 3: Initialize Database (1 minute)

```bash
python init_database.py
```

Expected output:
```
INFO:     Connected to Postgres
INFO:     Schema initialized successfully
INFO:     Database initialization complete!
```

## Step 4: Upload Embeddings (5 minutes)

**Note:** You need to create an embeddings upload script. Create `backend/scripts/upload_embeddings.py`:

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

Then run it:

```bash
cd scripts
python upload_embeddings.py
```

Type `y` when asked to recreate collection.

Wait for: `✓ SUCCESS! Embeddings uploaded to Qdrant`

## Step 5: Start Backend (1 minute)

```bash
cd ..
python main.py
```

You should see:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
```

## Step 6: Test Backend (1 minute)

Open a new terminal:

```bash
curl http://localhost:8000/health
```

Expected response:
```json
{"status":"ok","message":"RAG chatbot is running","database_connected":true}
```

## Step 7: Integrate Frontend (2 minutes)

### Option A: Quick Test (No Docusaurus Integration)

Test the API directly with curl:

```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d "{\"question\": \"What is this documentation about?\"}"
```

You should get a JSON response with an answer and sources!

### Option B: Full Docusaurus Integration

The chatbot is already integrated in this project. To use it:

1. Start the Docusaurus dev server:
   ```bash
   npm start
   ```

2. Visit http://localhost:3000

3. Click the chat button (💬) in the bottom-right corner!

## Verify It's Working

✅ Backend health check passes
✅ Question returns answer with sources
✅ Chatbot appears in Docusaurus
✅ Can ask questions in chat
✅ Selected text feature works

## Troubleshooting

### "Module not found" errors
```bash
pip install -r requirements.txt
```

### "Database connection failed"
- Check `NEON_DB_URL` is correct
- Verify Neon database is running

### "No relevant information found"
- Re-run embeddings upload script
- Check `DOCS_PATH` points to correct folder

### CORS errors
- Add `http://localhost:3000` to `CORS_ORIGINS` in `.env`
- Restart backend

## Next Steps

✅ **Working locally?** → See [DEPLOYMENT.md](./DEPLOYMENT.md) to deploy to production

✅ **Want to customize?** → See [DOCUSAURUS_INTEGRATION.md](./DOCUSAURUS_INTEGRATION.md) for UI customization

✅ **Need API details?** → See [API.md](./API.md) for complete API reference

## Support

Stuck? Check:
1. [SETUP.md](./SETUP.md) - Detailed setup guide
2. [README.md](./README.md) - Complete overview
3. Backend terminal logs for errors

---

**That's it! You now have a working RAG chatbot!** 🎉
