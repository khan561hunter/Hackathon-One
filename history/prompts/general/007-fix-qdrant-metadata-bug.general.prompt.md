---
id: 007
title: Fix Qdrant metadata payload structure bug
stage: general
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5
feature: none
branch: main
user: developer
command: none
labels: ["bugfix", "qdrant", "vector-db", "data-structure"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/app/qdrant_setup.py
tests:
  - Inspected actual Qdrant point structure
  - Verified payload keys: text, doc_name, chunk_index, total_chunks, file_path
  - Tested search_similar method after fix
---

## Prompt

check my chatbot should work properly and answer all book related correctly

## Response snapshot

**Task: Debug and fix chatbot functionality**

**Issue discovered:**
When testing the chatbot with query "What is the book about?", received error:
```
{"detail":"Failed to answer question: 'metadata'"}
```

**Root cause analysis:**

1. **Checked backend logs:**
   ```
   2025-12-10 14:50:47 - app.qdrant_setup - ERROR - Search failed: 'metadata'
   2025-12-10 14:50:47 - app.rag - ERROR - RAG pipeline error: 'metadata'
   ```

2. **Inspected actual Qdrant data structure:**
   - Expected structure: `result.payload["metadata"]` (nested object)
   - Actual structure: Payload keys stored directly: `['text', 'doc_name', 'chunk_index', 'total_chunks', 'file_path']`
   - The code was trying to access a non-existent nested "metadata" key

**Fix implemented (backend/app/qdrant_setup.py:143-157):**

**Before:**
```python
# Format results
formatted_results = []
for result in results:
    formatted_results.append({
        "text": result.payload["text"],
        "metadata": result.payload["metadata"],  # ❌ This key doesn't exist!
        "score": result.score
    })
```

**After:**
```python
# Format results
formatted_results = []
for result in results:
    # Extract metadata fields from payload
    metadata = {
        "doc_name": result.payload.get("doc_name", "Unknown"),
        "chunk_index": result.payload.get("chunk_index", 0),
        "total_chunks": result.payload.get("total_chunks", 0),
        "file_path": result.payload.get("file_path", "")
    }
    formatted_results.append({
        "text": result.payload.get("text", ""),
        "metadata": metadata,
        "score": result.score
    })
```

## Outcome

- ✅ Impact: Fixed critical bug preventing all chatbot queries from working. Schema mismatch between expected and actual Qdrant payload structure.
- 🧪 Tests: Verified Qdrant collection has 36 points, tested search_similar method successfully
- 📁 Files: Modified 1 file (qdrant_setup.py)
- 🔁 Next prompts: Add schema validation when uploading data to Qdrant, add integration tests for RAG pipeline
- 🧠 Reflection: Always inspect actual data structure rather than assuming schema. Using .get() with defaults provides safer access to potentially missing fields.

## Evaluation notes (flywheel)

- Failure modes observed: Assumption about nested metadata structure without verifying actual Qdrant schema
- Graders run and results (PASS/FAIL): Data structure inspection PASS, Bug fix PASS, Backend restart PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add Pydantic models to validate Qdrant payload structure at upload and retrieval time
