"""
PostgreSQL service for Neon Serverless database.
Handles conversation history and session management.
"""

from typing import List, Optional, Dict, Any
from datetime import datetime
import asyncpg
import uuid

from config import settings
from models.chat import ChatMessage, ConversationHistory


class PostgresService:
    """Service for Neon Postgres database operations."""
    
    def __init__(self):
        """Initialize with database URL."""
        self.database_url = settings.database_url
        self._pool: asyncpg.Pool = None
    
    async def connect(self) -> None:
        """Create connection pool."""
        if self._pool is None:
            self._pool = await asyncpg.create_pool(
                self.database_url,
                min_size=1,
                max_size=10,
                ssl="require"
            )
            print("Connected to Neon Postgres")
    
    async def disconnect(self) -> None:
        """Close connection pool."""
        if self._pool:
            await self._pool.close()
            self._pool = None
            print("Disconnected from Neon Postgres")
    
    async def init_tables(self) -> None:
        """Initialize database tables if they don't exist."""
        await self.connect()
        
        async with self._pool.acquire() as conn:
            # Create conversations table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS conversations (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    session_id TEXT NOT NULL UNIQUE,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)
            
            # Create messages table
            await conn.execute("""
                CREATE TABLE IF NOT EXISTS messages (
                    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                    conversation_id UUID REFERENCES conversations(id) ON DELETE CASCADE,
                    role TEXT NOT NULL,
                    content TEXT NOT NULL,
                    selected_text TEXT,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)
            
            # Create indexes
            await conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_messages_conversation 
                ON messages(conversation_id)
            """)
            await conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_conversations_session 
                ON conversations(session_id)
            """)
            
            print("Database tables initialized")
    
    async def get_or_create_conversation(self, session_id: str) -> str:
        """
        Get existing conversation or create new one.
        
        Args:
            session_id: Client session identifier
            
        Returns:
            Conversation UUID
        """
        await self.connect()
        
        async with self._pool.acquire() as conn:
            # Try to get existing conversation
            row = await conn.fetchrow(
                "SELECT id FROM conversations WHERE session_id = $1",
                session_id
            )
            
            if row:
                # Update last activity
                await conn.execute(
                    "UPDATE conversations SET updated_at = CURRENT_TIMESTAMP WHERE session_id = $1",
                    session_id
                )
                return str(row['id'])
            
            # Create new conversation
            row = await conn.fetchrow(
                """
                INSERT INTO conversations (session_id) 
                VALUES ($1) 
                RETURNING id
                """,
                session_id
            )
            return str(row['id'])
    
    async def add_message(
        self,
        conversation_id: str,
        role: str,
        content: str,
        selected_text: Optional[str] = None
    ) -> str:
        """
        Add a message to a conversation.
        
        Args:
            conversation_id: UUID of the conversation
            role: 'user' or 'assistant'
            content: Message content
            selected_text: Optional selected text context
            
        Returns:
            Message UUID
        """
        await self.connect()
        
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                INSERT INTO messages (conversation_id, role, content, selected_text) 
                VALUES ($1, $2, $3, $4) 
                RETURNING id
                """,
                uuid.UUID(conversation_id),
                role,
                content,
                selected_text
            )
            return str(row['id'])
    
    async def get_conversation_history(
        self,
        session_id: str,
        limit: int = 10
    ) -> List[ChatMessage]:
        """
        Get conversation history for a session.
        
        Args:
            session_id: Client session identifier
            limit: Maximum number of messages to return
            
        Returns:
            List of ChatMessage objects
        """
        await self.connect()
        
        async with self._pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT m.role, m.content, m.created_at
                FROM messages m
                JOIN conversations c ON m.conversation_id = c.id
                WHERE c.session_id = $1
                ORDER BY m.created_at DESC
                LIMIT $2
                """,
                session_id,
                limit
            )
            
            messages = []
            for row in reversed(rows):  # Reverse to get chronological order
                messages.append(ChatMessage(
                    role=row['role'],
                    content=row['content'],
                    timestamp=row['created_at']
                ))
            
            return messages
    
    async def clear_conversation(self, session_id: str) -> bool:
        """
        Clear all messages in a conversation.
        
        Args:
            session_id: Client session identifier
            
        Returns:
            True if successful
        """
        await self.connect()
        
        async with self._pool.acquire() as conn:
            await conn.execute(
                """
                DELETE FROM messages 
                WHERE conversation_id IN (
                    SELECT id FROM conversations WHERE session_id = $1
                )
                """,
                session_id
            )
            return True


# Global singleton instance
_postgres_service: PostgresService = None


def get_postgres_service() -> PostgresService:
    """Get or create the global Postgres service instance."""
    global _postgres_service
    if _postgres_service is None:
        _postgres_service = PostgresService()
    return _postgres_service
