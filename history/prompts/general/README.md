# General Prompt History Records

This directory contains Prompt History Records (PHRs) documenting the complete development of the RAG chatbot system.

---

## Session: 2025-12-10 (Part 1) - Chatbot Implementation

### Overview
Complete implementation of RAG (Retrieval-Augmented Generation) chatbot system for the Physical AI and Humanoid Robotics book.

### Implementation Records (001-004):

**001. Implement RAG chatbot frontend component**
- **Summary:** React chatbot component with RAG and text selection modes
- **Impact:** Full-featured UI with dark mode, quick questions, and floating widget
- **Files created:** 5 (component, styles, theme wrapper, env example, gitignore)
- **Lines of code:** ~880
- **Features:**
  - Floating widget with expandable chat window
  - RAG question mode (vector search + AI generation)
  - Text selection mode (ask about highlighted text)
  - Quick questions sidebar
  - Dark mode support
  - Real-time messaging with loading states

**002. Implement complete backend RAG system**
- **Summary:** FastAPI backend with complete RAG pipeline
- **Impact:** Production-ready API with Cohere embeddings, Qdrant search, Claude generation
- **Files created:** 13 backend files
- **Lines of code:** ~1,719
- **Components:**
  - FastAPI REST API (main.py - 211 lines)
  - RAG pipeline orchestrator (rag.py - 256 lines)
  - Cohere embeddings (embeddings_cohere.py - 92 lines)
  - Qdrant vector DB manager (qdrant_setup.py - 202 lines)
  - Postgres logging (postgres.py - 301 lines)
  - Claude LLM generator (llm_claude.py - 187 lines)
  - Qwen alternative LLM (llm_qwen.py - 131 lines)
  - Simple fallback generator (llm_simple.py - 95 lines)
  - Utilities (utils.py - 108 lines)
- **Endpoints:**
  - `GET /health` - Health check
  - `POST /ask` - RAG question answering
  - `POST /ask-selected` - Ask about selected text

**003. Create database initialization script**
- **Summary:** Standalone script to set up Postgres schema
- **Impact:** Database ready for chat logging and analytics
- **Files created:** 1 (init_database.py - 45 lines)
- **Tables:**
  - `chat_interactions` - RAG question-answer logs
  - `selection_interactions` - Selected text Q&A logs
- **Features:**
  - Async database initialization
  - Indexes for performance
  - Analytics-ready schema (JSONB for retrieved docs)

**004. Create comprehensive project documentation**
- **Summary:** Setup, quickstart, and production deployment guides
- **Impact:** Complete onboarding documentation for developers
- **Files created:** 3 (SETUP.md, QUICKSTART.md, PRODUCTION.md)
- **Content:**
  - Prerequisites and installation
  - Environment configuration
  - Database setup
  - Running the application
  - Troubleshooting guide
  - Architecture overview
  - Production deployment
  - Security best practices

---

## Session: 2025-12-10 (Part 2) - Ngrok Removal and Debugging

### Overview
Maintenance session removing ngrok infrastructure and fixing critical chatbot functionality bugs.

### Maintenance Records (005-011):

**005. Remove ngrok process and fix chatbot functionality** (Master Record)
- **Summary:** Master record for entire debugging session
- **Impact:** Removed ngrok, fixed metadata bug, verified system
- **Files modified:** 3
- **Overall outcome:** System working correctly with local development setup

**006. Remove ngrok infrastructure from project**
- **Task:** Remove ngrok references from codebase
- **Impact:** Cleaned up temporary deployment infrastructure
- **Files modified:**
  - src/components/Chatbot/index.jsx (removed hardcoded ngrok URL)
  - backend/.env (removed ngrok from CORS_ORIGINS)
- **Changes:**
  - Replaced hardcoded ngrok URL with environment variable fallback
  - Removed ngrok from allowed CORS origins

**007. Fix Qdrant metadata payload structure bug** ⚠️ **CRITICAL**
- **Task:** Fix Qdrant payload structure mismatch
- **Impact:** CRITICAL - Fixed bug preventing all chatbot queries from working
- **Files modified:** backend/app/qdrant_setup.py
- **Root cause:** Code expected nested `result.payload["metadata"]` object, but Qdrant stores metadata fields directly in payload (`doc_name`, `chunk_index`, etc.)
- **Fix:** Extract metadata fields directly from payload and construct metadata dictionary
- **Bug manifestation:** All `/ask` requests returned error: `"Failed to answer question: 'metadata'"`

**008. Verify Qdrant book data and collection status**
- **Task:** Verify book data in Qdrant collection
- **Impact:** Confirmed 36 book embeddings properly stored with correct schema
- **Files modified:** None (verification only)
- **Findings:**
  - Collection: `book_embeddings`
  - Vector dimensions: 1024 (Cohere embed-english-v3.0)
  - Total points: 36
  - Payload structure: `{text, doc_name, chunk_index, total_chunks, file_path}`

**009. Start FastAPI backend server**
- **Task:** Start FastAPI backend on port 8000
- **Impact:** Backend running successfully with database connectivity
- **Files modified:** None (server startup)
- **Services initialized:**
  - FastAPI application
  - Postgres database connection (Neon)
  - CORS middleware
  - Database schema
  - RAG pipeline
- **Health check:** ✅ PASS

**010. Test chatbot book-related queries**
- **Task:** Test chatbot with book-related queries after bug fix
- **Impact:** Verified RAG pipeline working correctly end-to-end
- **Files modified:** None (testing only)
- **Test results:**
  - Query 1: "What is the book about?" → 3 sources (21% relevance each)
  - Query 2: "What is ROS2?" → 3 sources (60%, 53%, 46% relevance)
- **Pipeline verified:**
  - ✅ Cohere embeddings
  - ✅ Qdrant vector search
  - ✅ Source attribution
  - ✅ Relevance scoring

**011. Start Docusaurus frontend development server**
- **Task:** Start Docusaurus frontend on port 3000
- **Impact:** Full system running (frontend + backend + chatbot widget)
- **Files modified:** None (server startup)
- **Services running:**
  - Docusaurus site
  - Chatbot widget component
  - Webpack client compilation
- **Access:** http://localhost:3000

**012. Configure mobile deployment setup**
- **Task:** Configure backend deployment for mobile access
- **Impact:** Enable chatbot to work on mobile devices with deployed backend
- **Files modified:**
  - docusaurus.config.js (updated backend URL for production)
  - backend/render.yaml (fixed API key names, added QDRANT_COLLECTION)
- **Files created:** MOBILE_DEPLOYMENT.md (comprehensive deployment guide)
- **Configuration:**
  - Production backend URL: https://hackathon-rag-backend.onrender.com
  - CORS already includes Vercel URL
  - Environment variables documented
  - Free tier deployment ready

**013. Fix CORS error blocking production requests** ⚠️ **PRODUCTION FIX**
- **Task:** Fix CORS configuration preventing frontend from accessing backend
- **Impact:** CRITICAL - Resolved CORS preflight failure blocking all chatbot requests
- **Files modified:**
  - backend/main.py (added origin whitespace stripping, expose_headers)
  - backend/render.yaml (ensured proper CORS_ORIGINS format)
- **Solution:**
  - Strip whitespace from CORS origins
  - Add `expose_headers=["*"]` to CORS middleware
  - Verify Vercel URL in CORS_ORIGINS

**014. Revert to localhost backend for development**
- **Task:** Remove mobile deployment configuration, use localhost
- **Impact:** Simplified setup for local development
- **Files modified:** docusaurus.config.js (backend URL set to localhost:8000)
- **What was kept:** CORS fixes from PHR 013 (still needed for proper browser security)
- **Verification:** Backend query test passed - "What is Physical AI?" returned 65% relevance
- **Configuration:** Frontend → localhost:3000, Backend → localhost:8000

**015. Add Railway.app deployment guide** 🚂
- **Task:** Provide free permanent backend deployment solution
- **Impact:** Enable chatbot to work on anyone's computer (not just localhost)
- **Files created:**
  - RAILWAY_DEPLOYMENT.md (comprehensive guide with alternatives)
  - backend/railway.json (Railway configuration)
- **Recommended platform:** Railway.app ($5/month free credits, always-on, no cold starts)
- **Alternatives documented:** Fly.io, Vercel Serverless, PythonAnywhere
- **Benefits:** No credit card required, auto-deploy from GitHub, permanent solution

**016. Verify Node.js version and dependencies** ✅
- **Task:** Check if Node.js needs updating
- **Impact:** Verified all versions current and compatible
- **Findings:**
  - Node.js: v24.11.1 (latest - ahead of LTS v22)
  - npm: 11.6.2 (latest)
  - Docusaurus: 3.9.2 (latest)
  - React: 18.3.1 (correct for Docusaurus, React 19 not compatible yet)
- **Result:** No updates needed - system optimal ✅

---

## Project Stats

### Total Implementation (Records 001-004)
- **Files created:** 22
- **Lines of code:** ~2,600
- **Components:** 9 backend modules, 1 frontend component, 3 documentation files
- **Commits:** 4 major commits

### Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    Frontend (Docusaurus)                 │
│  ┌────────────────────────────────────────────────────┐ │
│  │         Chatbot Component (React)                  │ │
│  │  - RAG question mode                               │ │
│  │  - Text selection mode                             │ │
│  │  - Quick questions menu                            │ │
│  │  - Dark mode support                               │ │
│  └────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
                          │ HTTP
                          ▼
┌─────────────────────────────────────────────────────────┐
│                   Backend (FastAPI)                      │
│  ┌────────────────────────────────────────────────────┐ │
│  │              RAG Pipeline                          │ │
│  │  1. Embed question (Cohere)                        │ │
│  │  2. Retrieve chunks (Qdrant)                       │ │
│  │  3. Generate answer (Claude)                       │ │
│  │  4. Log interaction (Postgres)                     │ │
│  └────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
         │              │              │
         ▼              ▼              ▼
    ┌────────┐    ┌─────────┐    ┌──────────┐
    │ Cohere │    │ Qdrant  │    │ Postgres │
    │Embed v3│    │ Vector  │    │  (Neon)  │
    │ 1024d  │    │   DB    │    │ Logging  │
    └────────┘    └─────────┘    └──────────┘
                       │
                       ▼
                  ┌─────────┐
                  │ Claude  │
                  │ Sonnet  │
                  │   4.5   │
                  └─────────┘
```

### Tech Stack

**Frontend:**
- React 18
- Docusaurus 3.6
- CSS Modules
- Text Selection API

**Backend:**
- FastAPI 0.124
- Python 3.10+
- Asyncio/asyncpg
- Pydantic validation

**AI/ML:**
- Cohere embed-english-v3.0 (1024 dimensions)
- Qdrant Cloud (vector database)
- Claude Sonnet 4.5 (text generation)
- Qwen (alternative LLM)

**Data Storage:**
- Qdrant: 36 book embeddings
- Neon Postgres: Chat logs, analytics

---

## Current System State

**Servers running:**
- ✅ Backend: http://localhost:8000
- ✅ Frontend: http://localhost:3000

**Data status:**
- ✅ Qdrant: 36 book embeddings (1024-dimensional Cohere vectors)
- ✅ Postgres: Connected (Neon serverless)
- ✅ Chat logging: Active

**Recent changes:**
- ✅ Removed: Hardcoded ngrok URLs (2 locations)
- ✅ Fixed: Qdrant metadata extraction bug (CRITICAL)
- ✅ Improved: Environment variable fallback for production URLs

---

## Key Learnings

### Implementation Phase (001-004)
1. **Modularity:** Separate embeddings, retrieval, generation, and storage concerns
2. **Flexibility:** Support multiple LLM providers (Claude, Qwen, fallback)
3. **State management:** React hooks (useState, useEffect, useRef) for clean UI state
4. **Type safety:** Pydantic models for request/response validation
5. **Async operations:** asyncpg for non-blocking database operations
6. **Documentation first:** Comprehensive docs enable faster onboarding

### Debugging Phase (005-011)
1. **Infrastructure:** Never hardcode temporary URLs (ngrok) in production code
2. **Data validation:** Always inspect actual data structure rather than assuming schema
3. **Error handling:** Use `.get()` with defaults for safer dictionary access
4. **Testing:** Verify both data layer and code layer when debugging
5. **Logging:** Structured logging enables faster debugging
6. **Systematic approach:** Check processes → review changes → inspect data → fix code → verify

---

## Next Steps

### High Priority
- [ ] Add schema validation for Qdrant uploads (Pydantic models)
- [ ] Add integration tests for RAG pipeline
- [ ] Fix Docusaurus v4 deprecation warning
- [ ] Configure production backend URL via environment variables
- [ ] Add error boundaries for frontend components

### Medium Priority
- [ ] Add streaming responses for real-time UI updates
- [ ] Add keyboard shortcuts (Cmd+K to open chatbot)
- [ ] Add markdown rendering for AI responses with code blocks
- [ ] Add conversation history persistence (local storage)
- [ ] Add voice input support
- [ ] Add source citation links in chatbot responses

### Low Priority
- [ ] Add Redis caching for frequently asked questions
- [ ] Add monitoring dashboard for Qdrant metrics
- [ ] Add rate limiting on API endpoints
- [ ] Add Sentry error tracking
- [ ] Add analytics dashboard for chat logs
- [ ] Add data retention policies
- [ ] Add A/B testing for different prompt strategies

---

## File Index

All PHR files are stored in chronological order in `history/prompts/general/`:

```
history/prompts/general/
├── 001-implement-chatbot-frontend.general.prompt.md (7.2 KB)
├── 002-implement-backend-rag-system.general.prompt.md (18 KB) ⭐
├── 003-database-initialization.general.prompt.md (6.8 KB)
├── 004-create-project-documentation.general.prompt.md (6.3 KB)
├── 005-remove-ngrok-fix-chatbot.general.prompt.md (3.7 KB) [Master]
├── 006-remove-ngrok-infrastructure.general.prompt.md (2.6 KB)
├── 007-fix-qdrant-metadata-bug.general.prompt.md (3.2 KB) ⚠️ CRITICAL
├── 008-verify-qdrant-data.general.prompt.md (2.5 KB)
├── 009-start-backend-server.general.prompt.md (2.4 KB)
├── 010-test-book-queries.general.prompt.md (3.8 KB)
├── 011-start-frontend-server.general.prompt.md (2.6 KB)
├── 012-configure-mobile-deployment.general.prompt.md (5.4 KB) 📱
├── 013-fix-cors-production-error.general.prompt.md (7.8 KB) ⚠️
├── 014-revert-to-localhost-backend.general.prompt.md (3.2 KB)
├── 015-add-railway-deployment-guide.general.prompt.md (6.1 KB) 🚂
├── 016-verify-node-version.general.prompt.md (4.3 KB) ✅
└── README.md (this file - 14 KB)
```

**Total:** 16 PHR files + 1 README (~98 KB documentation)

---

## Usage Guide

### How to read these PHRs:

1. **For understanding implementation:** Read 001-004 in order
2. **For debugging similar issues:** Read 005-011 (especially 007 for critical bugs)
3. **For quick reference:** Check this README for summaries
4. **For specific topics:** Use file index to find relevant PHR

### PHR Structure:

Each PHR contains:
- **Metadata:** ID, title, stage, date, labels, files modified
- **Prompt:** Original user request (verbatim)
- **Response:** Detailed implementation/fix description
- **Code snippets:** Before/after comparisons
- **Outcome:** Impact, tests, next steps, reflection
- **Evaluation:** Failure modes, test results, experiments

---

## Contact & Support

For questions or issues:
1. Check troubleshooting section in SETUP.md
2. Review PHR files for implementation details
3. Check backend logs for error messages
4. Verify API keys are correctly configured in .env
5. Review PHR 007 if encountering Qdrant-related errors

---

## Version History

- **v1.0** (2025-12-10): Initial chatbot implementation (001-004)
- **v1.1** (2025-12-10): Ngrok removal and bug fixes (005-011)

---

*Last updated: 2025-12-10*
*Total PHRs: 16*
*Total lines documented: ~3,600*
*Documentation size: ~98 KB*
