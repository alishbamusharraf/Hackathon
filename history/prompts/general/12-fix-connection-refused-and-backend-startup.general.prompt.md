---
id: 12
title: Fix Connection Refused and Backend Startup
stage: general
date_iso: 2025-12-27
surface: agent
model: gemini-2.0-flash-exp
feature: none
branch: main
user: |
  WebSocketClient.js:14 
   WebSocket connection to 'ws://localhost:3001/ws' failed: Error in connection establishment: net::ERR_CONNECTION_REFUSED
  index.js:602 
   [webpack-dev-server] 
  Event
  
  index.js:170 
   Chat stream error: TypeError: Failed to fetch
      at eval (index.js:109:1)
      at handleSubmit (ChatInput.js:4:1)
      at handleKeyDown (ChatInput.js:19:1)
  robot_logo_v2.png:1 
   
   GET http://localhost:3001/img/robot_logo_v2.png net::ERR_CONNECTION_REFUSED
command: api\.venv\Scripts\python.exe api\main.py
labels:
  - connection-error
  - backend
  - fastapi
  - docusaurus
links: null
files_yaml: []
tests_yaml:
  - netstat -ano | findstr :3000
  - api\.venv\Scripts\python.exe api\main.py
  - curl http://localhost:8000/health
outcome: The backend was not running, and the browser was attempting to connect to the frontend on the wrong port (3001 instead of 3000). I started the FastAPI backend and verified it is healthy.
evaluation: The backend is now reachable, and the chatbot should function correctly when accessed via port 3000.
---

# Prompt History Record - Fix Connection Refused and Backend Startup

## User Request
The user reported multiple connection errors:
1. `net::ERR_CONNECTION_REFUSED` on port 3001 for WebSockets and images.
2. `TypeError: Failed to fetch` in the chatbot component.

## Investigation
- Checked project structure and found a FastAPI backend in `api/` and a Docusaurus frontend.
- Ran `netstat` and found that port 3000 was LISTENING (frontend), but ports 3001 and 8000 (backend) were NOT.
- Verified that the browser tries to connect to the frontend on 3001, which is incorrect as it is running on 3000.
- Verified that the chatbot is hardcoded to use `http://localhost:8000` as its API base.

## Actions Taken
- Started the FastAPI backend using the project's virtual environment: `api\.venv\Scripts\python.exe api\main.py`.
- Verified the backend is operational via a health check to `http://localhost:8000/health`.

## Outcome
- Backend is now running and ready to serve chat requests.
- Identified that the user should use `http://localhost:3000` for the frontend to avoid connection refused errors.
