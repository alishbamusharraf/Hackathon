"""
Chat router with endpoints for conversational AI.
"""

import traceback
from typing import Optional

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
import uuid
import json

from models.chat import (
    ChatRequest,
    ChatResponse,
    SearchRequest,
    SearchResponse,
)
from services.embeddings import get_embeddings_service
from services.qdrant_service import get_qdrant_service
from services.postgres_service import get_postgres_service
from services.agent_service import get_agent_service
from config import settings


router = APIRouter(prefix="/api", tags=["chat"])


@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Main chat endpoint for conversational AI.
    
    - Accepts user message and optional selected text
    - Retrieves relevant context from book
    - Generates response using Gemini agent
    - Stores conversation history
    """
    try:
        # Get services
        embeddings_service = get_embeddings_service()
        qdrant_service = get_qdrant_service()
        postgres_service = get_postgres_service()
        agent_service = get_agent_service()
        print(f"Chat request received: {request.message[:50]}...")

        
        # Generate or use provided session ID
        session_id = request.session_id or str(uuid.uuid4())
        
        # Get or create conversation
        conversation_id = await postgres_service.get_or_create_conversation(session_id)
        print(f"Session ID: {session_id}, Conversation ID: {conversation_id}")

        
        # Get conversation history
        history = await postgres_service.get_conversation_history(session_id, limit=6)
        
        # Store user message
        await postgres_service.add_message(
            conversation_id=conversation_id,
            role="user",
            content=request.message,
            selected_text=request.selected_text
        )
        print("User message stored in database")

        
        # Generate embedding for the query
        # Combine user message with selected text for better retrieval
        search_query = request.message
        if request.selected_text:
            search_query = f"{request.message} Context: {request.selected_text[:500]}"
        
        
        query_embedding = embeddings_service.embed_text(search_query)
        print("Query embedding generated")

        
        # Search for relevant context
        search_results = await qdrant_service.search(
            query_embedding=query_embedding,
            top_k=settings.top_k_results
        )
        print(f"Search completed: {len(search_results)} results found")

        
        # Generate response using agent
        response_text = await agent_service.generate_response(
            user_message=request.message,
            search_results=search_results,
            selected_text=request.selected_text,
            conversation_history=history
        )
        print("Assistant response generated")

        
        # Store assistant response
        await postgres_service.add_message(
            conversation_id=conversation_id,
            role="assistant",
            content=response_text
        )
        
        # Extract source information
        sources = list(set([
            f"{r.module}/{r.chapter}" for r in search_results if r.score > 0.5
        ]))
        
        return ChatResponse(
            message=response_text,
            session_id=session_id,
            sources=sources
        )
        
    except Exception as e:
        print(f"Error in chat: {e}")
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=str(e))



@router.post("/chat/stream")
async def chat_stream(request: ChatRequest):
    """
    Streaming chat endpoint for real-time response.
    
    Returns Server-Sent Events (SSE) with response chunks.
    """
    import time
    start_total = time.time()
    
    try:
        # Get services
        t0 = time.time()
        embeddings_service = get_embeddings_service()
        qdrant_service = get_qdrant_service()
        postgres_service = get_postgres_service()
        agent_service = get_agent_service()
        print(f"[{time.time() - t0:.3f}s] Services loaded")

        print(f"Chat stream request received: {request.message[:50]}...")
        
        # Generate or use provided session ID
        session_id = request.session_id or str(uuid.uuid4())
        
        # Get or create conversation (DB operation 1)
        t1 = time.time()
        conversation_id = await postgres_service.get_or_create_conversation(session_id)
        # Get conversation history (DB operation 2)
        history = await postgres_service.get_conversation_history(session_id, limit=6)
        # Store user message (DB operation 3)
        await postgres_service.add_message(
            conversation_id=conversation_id,
            role="user",
            content=request.message,
            selected_text=request.selected_text
        )
        print(f"[{time.time() - t1:.3f}s] Database operations (Get history + Store message)")

        # Generate embedding for the query (CPU operation)
        search_query = request.message
        if request.selected_text:
            search_query = f"{request.message} Context: {request.selected_text[:500]}"
            
        t2 = time.time()
        query_embedding = embeddings_service.embed_text(search_query)
        print(f"[{time.time() - t2:.3f}s] Embedding generation")

        # Search for relevant context (Network operation - Qdrant)
        t3 = time.time()
        search_results = await qdrant_service.search(
            query_embedding=query_embedding,
            top_k=settings.top_k_results
        )
        print(f"[{time.time() - t3:.3f}s] Qdrant search ({len(search_results)} results)")

        async def generate():
            try:
                t_gen_start = time.time()
                full_response = ""
                
                # Send session ID first
                yield f"data: {json.dumps({'type': 'session', 'session_id': session_id})}\n\n"
                
                # Stream response chunks (Network operation - Gemini)
                print("Starting Gemini streaming...")
                first_token_time = None
                
                async for chunk in agent_service.generate_response_stream(
                    user_message=request.message,
                    search_results=search_results,
                    selected_text=request.selected_text,
                    conversation_history=history
                ):
                    if first_token_time is None:
                        first_token_time = time.time()
                        print(f"[{first_token_time - t_gen_start:.3f}s] Time to First Token (TTFT)")
                    
                    full_response += chunk
                    yield f"data: {json.dumps({'type': 'chunk', 'content': chunk})}\n\n"
                
                total_gen_time = time.time() - t_gen_start
                print(f"[{total_gen_time:.3f}s] Total generation time. Response length: {len(full_response)}")
                
                # Store complete response (DB operation 4)
                await postgres_service.add_message(
                    conversation_id=conversation_id,
                    role="assistant",
                    content=full_response
                )
                
                # Send completion signal with sources
                sources = list(set([
                    f"{r.module}/{r.chapter}" for r in search_results if r.score > 0.5
                ]))
                yield f"data: {json.dumps({'type': 'done', 'sources': sources})}\n\n"
                
                print(f"[{time.time() - start_total:.3f}s] Total Request Time")
                
            except Exception as e:
                print(f"Error during streaming generation: {e}")
                traceback.print_exc()
                yield f"data: {json.dumps({'type': 'error', 'content': str(e)})}\n\n"

        return StreamingResponse(
            generate(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
            }
        )
        
    except Exception as e:
        print(f"Error in chat_stream: {e}")
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=str(e))



@router.post("/search", response_model=SearchResponse)
async def search(request: SearchRequest):
    """
    Semantic search endpoint.
    
    Search the book content for relevant passages.
    """
    try:
        embeddings_service = get_embeddings_service()
        qdrant_service = get_qdrant_service()
        
        # Generate query embedding
        query_embedding = embeddings_service.embed_text(request.query)
        
        # Search
        results = await qdrant_service.search(
            query_embedding=query_embedding,
            top_k=request.top_k
        )
        
        return SearchResponse(
            results=results,
            query=request.query
        )
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/chat/{session_id}")
async def clear_chat(session_id: str):
    """Clear conversation history for a session."""
    try:
        postgres_service = get_postgres_service()
        await postgres_service.clear_conversation(session_id)
        return {"success": True, "message": f"Cleared conversation for session {session_id}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
