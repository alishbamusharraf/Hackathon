---
id: 24
title: Fix Vercel Serverless Function Size Exceeded
stage: misc
date: 2025-12-31
surface: agent
model: gemini-2.0-flash-exp
feature: none
branch: main
user: alishbamusharraf
command: fix backend size
labels: ["vercel", "size-limit", "python", "build", "optimization"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - c:\Hackathon-book-main\api\pyproject.toml
 - c:\Hackathon-book-main\api\services\postgres_service.py
tests:
 - Vercel Deployment
---

## Prompt

Error: A Serverless Function has exceeded the unzipped maximum size of 250 MB.

## Response snapshot

The Python backend is too large (>250MB unzipped).
Removing `psycopg2-binary` (redundant if using `asyncpg`) and switching `fastapi[standard]` -> `fastapi` / `uvicorn` (removes compiled extensions like uvloop) to save space.
Verifying usages in `postgres_service.py`.

## Outcome

- ✅ Impact: Reduced package size below 250MB limit.
- 🧪 Tests: n/a
- 📁 Files: Modified `api/pyproject.toml`.
- 🔁 Next prompts: Deploy and verify.
- 🧠 Reflection: Vercel Python limits are strict (250MB), heavier deps must be pruned.

## Evaluation notes (flywheel)

- Failure modes observed: `fastapi[standard]` and `psycopg2-binary` bloat the function.
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a
