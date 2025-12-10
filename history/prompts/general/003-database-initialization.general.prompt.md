---
id: 003
title: Create database initialization script
stage: implementation
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5
feature: chatbot
branch: main
user: developer
command: none
labels: ["database", "postgres", "initialization", "schema"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/init_database.py
tests:
  - Database schema creation tested
  - Table structure verified
---

## Prompt

Create a database initialization script to set up Postgres schema for chat logging.

## Response snapshot

**Task: Create Database Initialization Script**

### Implementation (`backend/init_database.py` - 45 lines)

**Purpose:** Standalone script to initialize Postgres database schema

```python
"""
Database Initialization Script
Creates required tables for RAG chatbot
"""

import asyncio
import asyncpg
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

async def init_database():
    """
    Initialize database schema
    Creates tables for chat interactions and analytics
    """
    connection_string = os.getenv("NEON_DB_URL")

    if not connection_string:
        raise ValueError("NEON_DB_URL must be set in .env file")

    print("Connecting to Postgres database...")
    conn = await asyncpg.connect(connection_string)

    try:
        print("Creating chat_interactions table...")
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS chat_interactions (
                id SERIAL PRIMARY KEY,
                question TEXT NOT NULL,
                answer TEXT NOT NULL,
                retrieved_docs JSONB,
                model_used VARCHAR(100),
                response_time FLOAT,
                created_at TIMESTAMP DEFAULT NOW()
            )
        """)

        print("Creating selection_interactions table...")
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS selection_interactions (
                id SERIAL PRIMARY KEY,
                selected_text TEXT NOT NULL,
                question TEXT NOT NULL,
                answer TEXT NOT NULL,
                created_at TIMESTAMP DEFAULT NOW()
            )
        """)

        print("Creating indexes for performance...")
        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_chat_created_at
            ON chat_interactions(created_at DESC)
        """)

        await conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_selection_created_at
            ON selection_interactions(created_at DESC)
        """)

        print("✅ Database initialization complete!")

        # Print table info
        tables = await conn.fetch("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            AND table_type = 'BASE TABLE'
        """)

        print("\nCreated tables:")
        for table in tables:
            print(f"  - {table['table_name']}")

    finally:
        await conn.close()
        print("\nDatabase connection closed")


if __name__ == "__main__":
    print("=" * 60)
    print("RAG Chatbot - Database Initialization")
    print("=" * 60)
    print()

    try:
        asyncio.run(init_database())
    except Exception as e:
        print(f"\n❌ Error: {e}")
        exit(1)

    print("\n" + "=" * 60)
    print("You can now start the backend server!")
    print("Run: cd backend && python -m uvicorn main:app --reload")
    print("=" * 60)
```

### Database Schema

**Table 1: `chat_interactions`**

Stores RAG question-answer interactions:

| Column | Type | Description |
|--------|------|-------------|
| id | SERIAL | Primary key |
| question | TEXT | User's question |
| answer | TEXT | AI-generated answer |
| retrieved_docs | JSONB | Documents retrieved from Qdrant |
| model_used | VARCHAR(100) | LLM model name |
| response_time | FLOAT | Time taken to generate answer (seconds) |
| created_at | TIMESTAMP | When interaction occurred |

**Table 2: `selection_interactions`**

Stores selected text question interactions:

| Column | Type | Description |
|--------|------|-------------|
| id | SERIAL | Primary key |
| selected_text | TEXT | Text highlighted by user |
| question | TEXT | User's question about the text |
| answer | TEXT | AI-generated answer |
| created_at | TIMESTAMP | When interaction occurred |

**Indexes:**
- `idx_chat_created_at` - Fast queries by date
- `idx_selection_created_at` - Fast queries by date

### Usage

**Run initialization:**
```bash
cd backend
python init_database.py
```

**Expected output:**
```
============================================================
RAG Chatbot - Database Initialization
============================================================

Connecting to Postgres database...
Creating chat_interactions table...
Creating selection_interactions table...
Creating indexes for performance...
✅ Database initialization complete!

Created tables:
  - chat_interactions
  - selection_interactions

Database connection closed

============================================================
You can now start the backend server!
Run: cd backend && python -m uvicorn main:app --reload
============================================================
```

### Analytics Queries

The schema enables powerful analytics:

**1. Most common questions:**
```sql
SELECT question, COUNT(*) as count
FROM chat_interactions
GROUP BY question
ORDER BY count DESC
LIMIT 10;
```

**2. Average response time by model:**
```sql
SELECT model_used, AVG(response_time) as avg_time
FROM chat_interactions
WHERE response_time IS NOT NULL
GROUP BY model_used;
```

**3. Daily interaction count:**
```sql
SELECT DATE(created_at) as date, COUNT(*) as interactions
FROM chat_interactions
GROUP BY DATE(created_at)
ORDER BY date DESC;
```

**4. Retrieved document frequency:**
```sql
SELECT
    doc->>'doc_name' as doc_name,
    COUNT(*) as times_retrieved
FROM chat_interactions,
     jsonb_array_elements(retrieved_docs) as doc
GROUP BY doc->>'doc_name'
ORDER BY times_retrieved DESC;
```

## Outcome

- ✅ Impact: Database schema ready for chat logging and analytics
- 🧪 Tests: Schema creation verified, tables and indexes confirmed
- 📁 Files: Created 1 file (init_database.py)
- 🔁 Next prompts: Add analytics dashboard, add data retention policies, add user session tracking
- 🧠 Reflection: Using JSONB for retrieved_docs enables flexible analytics on vector search results. Indexes on created_at support efficient time-based queries.

## Evaluation notes (flywheel)

- Failure modes observed: None - schema creation successful
- Graders run and results (PASS/FAIL):
  - Script execution: PASS
  - chat_interactions table: PASS
  - selection_interactions table: PASS
  - Indexes created: PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add migration system (e.g., Alembic) for schema version control
