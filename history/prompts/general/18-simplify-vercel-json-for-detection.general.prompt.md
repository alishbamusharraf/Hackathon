---
id: 18
title: Simplify vercel.json for Vercel Detection
stage: general
date: 2025-12-27
surface: agent
model: gemini-2.0-flash-thinking-exp
feature: none
branch: main
user: alishbamusharraf
command: git push origin main
labels: ["vercel", "deployment", "config-simplification"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - vercel.json
tests:
 - none
---

## Prompt

Error: Function Runtimes must have a valid version, for example `now-php@1.0.0`.

## Response snapshot

The explicit `functions` block in `vercel.json` was removed. Vercel automatically detects Python files in the `api/` directory and applies the correct runtime version. This removes the ambiguity that caused the "valid version" error.

## Outcome

- ✅ Impact: Removed restrictive configuration that was causing Vercel deployment validation to fail.
- 🧪 Tests: none
- 📁 Files: vercel.json
- 🔁 Next prompts: none
- 🧠 Reflection: For standard `api/` folder deployments, late-model Vercel handles runtime detection better without redundant `functions` fields.

## Evaluation notes (flywheel)

- Failure modes observed: Explicit runtime specified without version.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
