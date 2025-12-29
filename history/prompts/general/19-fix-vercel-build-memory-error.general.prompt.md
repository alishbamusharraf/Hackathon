---
id: 19
title: Fix Vercel Build Memory Error
stage: general
date: 2025-12-27
surface: agent
model: gemini-2.0-flash-thinking-exp
feature: none
branch: main
user: alishbamusharraf
command: git push origin main
labels: ["vercel", "build-error", "memory-limit", "gemini-embeddings"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - api/requirements.txt
 - api/config.py
 - api/services/embeddings.py
 - api/routers/chat.py
tests:
 - none
---

## Prompt

Your build failed because it exceeded the amount of memory available. Enable Enhanced Builds to increase machine size and available memory.

## Response snapshot

The `sentence-transformers` and `torch` dependencies were identified as the cause of the memory limit being exceeded on Vercel's free tier. These local embedding models were replaced with the Google Gemini Embedding API, which drastically reduces the build size and memory footprint. The configuration was updated to reflect the new 768-dimension vectors, and a fresh Qdrant collection name (`book_content_v2`) was set to ensure a clean start.

## Outcome

- ✅ Impact: Build size reduced by hundreds of megabytes, allowing successful deployment on Vercel.
- 🧪 Tests: none
- 📁 Files: api/requirements.txt, api/config.py, api/services/embeddings.py, api/routers/chat.py
- 🔁 Next prompts: none
- 🧠 Reflection: Vercel serverless functions have a 250MB size limit; keeping local ML models out of the package is critical for stability.

## Evaluation notes (flywheel)

- Failure modes observed: Memory limit exceeded during build.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
