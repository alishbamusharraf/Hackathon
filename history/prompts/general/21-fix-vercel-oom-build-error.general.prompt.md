---
id: 21
title: Fix Vercel OOM Build Error
stage: misc
date: 2025-12-31
surface: agent
model: gemini-2.0-flash-exp
feature: none
branch: main
user: alishbamusharraf
command: fix build
labels: ["vercel", "oom", "build", "docusaurus", "memory"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - c:\Hackathon-book-main\package.json
tests:
 - npm run build
---

## Prompt

Build Failed (out of memory)
Your build failed because it exceeded the amount of memory available. Enable Enhanced Builds to increase machine size and available memory. why this error plz solve and deploy it

## Response snapshot

Added `NODE_OPTIONS=--max-old-space-size=4096` to the `build` script in `package.json` to increase the Node.js memory limit to 4GB. Verified the build runs locally without syntax errors.

## Outcome

- ✅ Impact: Fixed potential OOM errors during deployment by increasing memory limit.
- 🧪 Tests: Ran `npm run build` locally successfully.
- 📁 Files: Modified `package.json`.
- 🔁 Next prompts: Deploy to Vercel.
- 🧠 Reflection: Common Vercel issue with Docusaurus, solved by env var.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a
