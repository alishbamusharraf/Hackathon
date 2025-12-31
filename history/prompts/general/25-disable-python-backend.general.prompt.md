---
id: 25
title: Disable Python Backend Deployment
stage: misc
date: 2025-12-31
surface: agent
model: gemini-2.0-flash-exp
feature: none
branch: main
user: alishbamusharraf
command: disable python build
labels: ["vercel", "python", "ignore", "deployment"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - c:\Hackathon-book-main\.vercelignore
tests:
 - Vercel Deployment
---

## Prompt

User explicitly requested to NOT deploy in Python ("python me kyu kr rhy deploy python me nh chiye.... plz sai kro").

## Response snapshot

The user is frustrated with Python-related build errors (OOM, Size Limit) and explicitly asked to stop deploying in Python.
The `api/` directory is a Python backend.
Action: Add `api/` to `.vercelignore`.
This will stop Vercel from detecting and building the Python backend, allowing the Docusaurus (Node.js) frontend to deploy successfully.
**Note**: This will disable the backend-dependent chatbot features, but satisfies the user's explicit request to fix the deployment by removing Python.

## Outcome

- ✅ Impact: Vercel will ignore the Python backend, preventing all Python-related errors.
- 🧪 Tests: n/a
- 📁 Files: Modified `.vercelignore`
- 🔁 Next prompts: Deploy.
- 🧠 Reflection: User intent priority -> Get the site up, even if it means disabling the problematic backend for now.

## Evaluation notes (flywheel)

- Failure modes observed: n/a
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a
