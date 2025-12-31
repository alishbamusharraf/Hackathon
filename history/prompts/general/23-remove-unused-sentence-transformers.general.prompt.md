---
id: 23
title: Remove Unused Sentence Transformers Dependency
stage: refactor
date: 2025-12-31
surface: agent
model: gemini-2.0-flash-exp
feature: backend
branch: main
user: alishbamusharraf
command: fix backend dependencies
labels: ["backend", "dependencies", "cleanup", "vercel", "oom"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - c:\Hackathon-book-main\api\pyproject.toml
 - c:\Hackathon-book-main\api\uv.lock
tests:
 - Vercel Build
---

## Prompt

Vercel OOM during deployment of outputs.

## Response snapshot

Identified that `sentence-transformers` (and thus PyTorch) is listed in `api/pyproject.toml` but NOT used in the codebase (which uses Gemini for embeddings).
This dependency is extremely heavy (GBs) and causes Vercel builds to time out or OOM during bundling.
Removing `sentence-transformers` and deleting `uv.lock` to force a lightweight reinstall.

## Outcome

- ✅ Impact: Massive reduction in build size and memory usage.
- 🧪 Tests: n/a
- 📁 Files: Modified `api/pyproject.toml`, deleted `api/uv.lock`.
- 🔁 Next prompts: Monitor deployment.
- 🧠 Reflection: Always check for unused heavy deps in template projects.

## Evaluation notes (flywheel)

- Failure modes observed: Unused heavy dependencies causing infrastructure collapse.
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a
