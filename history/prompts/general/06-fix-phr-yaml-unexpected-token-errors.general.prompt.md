---
id: 6
title: "Fix PHR YAML Unexpected Token Errors"
stage: general
date: 2025-12-25
surface: agent
model: gemini-2.0-flash-exp
feature: none
branch: main
user: alishbamusharraf/Hackathon
command: none
labels: ["yaml", "phr", "syntax", "fix"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - history/prompts/general/01-fix-backend-crash-failed-to-fetch.general.prompt.md
  - history/prompts/general/02-fix-api-import-and-stream-stability.general.prompt.md
tests:
  - "none"
---

## Prompt

Explain what this problem is and help me fix it: Unexpected token 'install'.

## Response snapshot

The error occurred because the YAML parser encountered a space in the `tests` list item without quotes. I have wrapped the multi-word commands in quotes.

## Outcome

- ✅ Impact: Resolved YAML parsing errors in PHR files.
- 🧪 Tests: Manual verification of fixed files.
- 📁 Files: Updated PHR files in history/prompts/general/.
- 🔁 Next prompts: Continue with pending tasks.
- 🧠 Reflection: Quotes are essential for strings with spaces in YAML lists.
