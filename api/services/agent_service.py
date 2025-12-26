"""
Google Gemini Agent service for dynamic query understanding and response generation.
Implements a soft-coded agent that understands query schema dynamically.
"""

from typing import List, Optional, AsyncGenerator
import google.generativeai as genai

from config import settings
from models.chat import ChatMessage, SearchResult


# System prompt for the soft-coded agent
AGENT_SYSTEM_PROMPT = """You are a helpful, friendly assistant for the "Physical AI & Humanoid Robotics" book. Your personality is warm, knowledgeable, and conversational.

## Your Core Behaviors:

### 1. Dynamic Understanding
- Understand user queries naturally without relying on hardcoded patterns
- Interpret the user's intent from context, tone, and phrasing
- Adapt your responses to match the user's level of expertise

### 2. Grounded Responses
- ONLY answer questions using the provided book context
- If the context doesn't contain relevant information, politely say so
- Never invent or assume information not present in the context
- Always stay within the scope of the book's content

### 3. Human-Like Communication
- Respond in a warm, conversational tone
- Use natural language, not robotic or formal speech
- Show enthusiasm when discussing interesting topics
- Acknowledge when you're uncertain

### 4. Helpful Formatting
- Use markdown for code examples, lists, and emphasis
- Break down complex explanations into digestible parts
- Provide examples when they help clarify concepts
- Reference the source chapter/module when relevant

### 5. Context Awareness
- If the user has selected specific text, prioritize that context
- Remember the conversation history for coherent multi-turn dialogue
- Connect related concepts across different parts of the book

## Response Guidelines:
- Keep responses concise but complete
- Ask clarifying questions if the query is ambiguous
- Suggest related topics the user might find interesting
- Be encouraging and supportive for learners

Remember: You are a knowledgeable friend helping someone learn about robotics, not a formal documentation system."""


class AgentService:
    """
    Soft-coded Google Gemini Agent for dynamic query understanding.
    
    This agent:
    - Dynamically understands user query schema
    - Generates human-like, conversational responses
    - Only uses provided context from the book
    - Maintains conversation continuity
    """
    
    def __init__(self):
        """Initialize Gemini client."""
        genai.configure(api_key=settings.gemini_api_key)
        self.model = genai.GenerativeModel(
            model_name=settings.gemini_model,
            system_instruction=AGENT_SYSTEM_PROMPT
        )
    
    def _build_context_message(
        self,
        search_results: List[SearchResult],
        selected_text: Optional[str] = None
    ) -> str:
        """
        Build context message from search results and selected text.
        
        Args:
            search_results: Retrieved relevant chunks from the book
            selected_text: User-selected text from the page
            
        Returns:
            Formatted context string
        """
        context_parts = []
        
        # Add selected text if provided
        if selected_text:
            context_parts.append(
                f"## User Selected Text:\n{selected_text}\n"
            )
        
        # Add retrieved context
        if search_results:
            context_parts.append("## Relevant Book Content:\n")
            for i, result in enumerate(search_results, 1):
                source_info = f"[Source: {result.module} - {result.chapter}]"
                context_parts.append(
                    f"### Excerpt {i} {source_info}\n{result.text}\n"
                )
        
        if not context_parts:
            return "No relevant context found in the book."
        
        return "\n".join(context_parts)
    
    def _build_chat_history(
        self,
        conversation_history: List[ChatMessage] = None
    ) -> List[dict]:
        """
        Build chat history for Gemini.
        
        Args:
            conversation_history: Previous messages in conversation
            
        Returns:
            List of message dicts for Gemini
        """
        history = []
        
        if conversation_history:
            for msg in conversation_history[-6:]:
                role = "user" if msg.role == "user" else "model"
                history.append({
                    "role": role,
                    "parts": [msg.content]
                })
        
        return history
    
    async def generate_response(
        self,
        user_message: str,
        search_results: List[SearchResult],
        selected_text: Optional[str] = None,
        conversation_history: List[ChatMessage] = None
    ) -> str:
        """
        Generate a response using the Gemini agent.
        
        Args:
            user_message: User's question or message
            search_results: Retrieved relevant chunks
            selected_text: User-selected text context
            conversation_history: Previous conversation messages
            
        Returns:
            Generated response string
        """
        # Build context from search results
        context = self._build_context_message(search_results, selected_text)
        
        # Build the prompt with context
        full_prompt = f"""# Book Context

{context}

# User Question

{user_message}

Please answer the user's question based ONLY on the book context provided above. If the context doesn't contain the answer, politely say so."""
        
        # Build chat history
        history = self._build_chat_history(conversation_history)
        
        # Create chat session
        chat = self.model.start_chat(history=history)
        
        # Generate response
        response = chat.send_message(
            full_prompt,
            generation_config=genai.types.GenerationConfig(
                temperature=0.7,
                max_output_tokens=1024,
                top_p=0.9,
            )
        )
        
        return response.text
    
    async def generate_response_stream(
        self,
        user_message: str,
        search_results: List[SearchResult],
        selected_text: Optional[str] = None,
        conversation_history: List[ChatMessage] = None
    ) -> AsyncGenerator[str, None]:
        """
        Generate a streaming response using the Gemini agent.
        
        Args:
            user_message: User's question or message
            search_results: Retrieved relevant chunks
            selected_text: User-selected text context
            conversation_history: Previous conversation messages
            
        Yields:
            Response chunks as they're generated
        """
        # Build context from search results
        context = self._build_context_message(search_results, selected_text)
        
        # Build the prompt with context
        full_prompt = f"""# Book Context

{context}

# User Question

{user_message}

Please answer the user's question based ONLY on the book context provided above. If the context doesn't contain the answer, politely say so."""
        
        # Build chat history
        history = self._build_chat_history(conversation_history)
        
        # Create chat session
        chat = self.model.start_chat(history=history)
        
        # Generate streaming response
        response = chat.send_message(
            full_prompt,
            generation_config=genai.types.GenerationConfig(
                temperature=0.7,
                max_output_tokens=1024,
                top_p=0.9,
            ),
            stream=True
        )
        
        for chunk in response:
            if chunk.text:
                yield chunk.text


# Global singleton instance
_agent_service: AgentService = None


def get_agent_service() -> AgentService:
    """Get or create the global Agent service instance."""
    global _agent_service
    if _agent_service is None:
        _agent_service = AgentService()
    return _agent_service
