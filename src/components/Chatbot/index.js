/**
 * Chatbot Component - Main Entry Point
 * Floating chat icon and modal for RAG chatbot
 */

import React, { useState, useCallback } from 'react';
import ChatIcon from './ChatIcon';
import ChatModal from './ChatModal';
import styles from './styles.module.css';

// API URL - change this when deploying to production
const API_BASE_URL = typeof window !== 'undefined' ? window.location.origin : '';

export default function Chatbot() {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState([]);
    const [isLoading, setIsLoading] = useState(false);
    const [sessionId, setSessionId] = useState(null);
    const [selectedText, setSelectedText] = useState('');

    // Toggle chat modal
    const toggleChat = useCallback(() => {
        setIsOpen(prev => !prev);
    }, []);

    // Handle text selection on the document
    const handleTextSelection = useCallback(() => {
        const selection = window.getSelection();
        const text = selection?.toString().trim();
        if (text && text.length > 10) {
            setSelectedText(text);
        }
    }, []);

    // Clear selected text
    const clearSelectedText = useCallback(() => {
        setSelectedText('');
    }, []);

    // Send message to API
    const sendMessage = useCallback(async (message, includeSelection = false) => {
        if (!message.trim()) return;

        const userMessage = {
            role: 'user',
            content: message,
            timestamp: new Date().toISOString(),
        };

        // Add selected text context if available
        const contextText = includeSelection && selectedText ? selectedText : null;

        setMessages(prev => [...prev, userMessage]);
        setIsLoading(true);

        try {
            const response = await fetch(`${API_BASE_URL}/api/chat`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    message: message,
                    session_id: sessionId,
                    selected_text: contextText,
                }),
            });

            if (!response.ok) {
                throw new Error('Failed to get response');
            }

            const data = await response.json();

            // Update session ID
            if (data.session_id) {
                setSessionId(data.session_id);
            }

            // Add assistant message
            const assistantMessage = {
                role: 'assistant',
                content: data.message,
                sources: data.sources,
                timestamp: new Date().toISOString(),
            };

            setMessages(prev => [...prev, assistantMessage]);

            // Clear selected text after using it
            if (includeSelection) {
                setSelectedText('');
            }
        } catch (error) {
            console.error('Chat error:', error);
            const errorMessage = {
                role: 'assistant',
                content: "I'm sorry, I encountered an error. Please try again.",
                isError: true,
                timestamp: new Date().toISOString(),
            };
            setMessages(prev => [...prev, errorMessage]);
        } finally {
            setIsLoading(false);
        }
    }, [sessionId, selectedText]);

    // Send streaming message
    const sendMessageStream = useCallback(async (message, includeSelection = false) => {
        if (!message.trim()) return;

        const userMessage = {
            role: 'user',
            content: message,
            timestamp: new Date().toISOString(),
        };

        const contextText = includeSelection && selectedText ? selectedText : null;

        setMessages(prev => [...prev, userMessage]);
        setIsLoading(true);

        try {
            const response = await fetch(`${API_BASE_URL}/api/chat/stream`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    message: message,
                    session_id: sessionId,
                    selected_text: contextText,
                }),
            });

            if (!response.ok) {
                throw new Error('Failed to get response');
            }

            const reader = response.body.getReader();
            const decoder = new TextDecoder();

            let assistantMessage = {
                role: 'assistant',
                content: '',
                sources: [],
                timestamp: new Date().toISOString(),
            };

            // Add empty assistant message
            setMessages(prev => [...prev, assistantMessage]);

            while (true) {
                const { done, value } = await reader.read();
                if (done) break;

                const chunk = decoder.decode(value);
                const lines = chunk.split('\n');

                for (const line of lines) {
                    if (line.startsWith('data: ')) {
                        try {
                            const data = JSON.parse(line.slice(6));

                            if (data.type === 'session') {
                                setSessionId(data.session_id);
                            } else if (data.type === 'chunk') {
                                assistantMessage.content += data.content;
                                // Update the last message
                                setMessages(prev => {
                                    const updated = [...prev];
                                    updated[updated.length - 1] = { ...assistantMessage };
                                    return updated;
                                });
                            } else if (data.type === 'done') {
                                assistantMessage.sources = data.sources;
                                setMessages(prev => {
                                    const updated = [...prev];
                                    updated[updated.length - 1] = { ...assistantMessage };
                                    return updated;
                                });
                            } else if (data.type === 'error') {
                                assistantMessage.content = data.content;
                                assistantMessage.isError = true;
                                setMessages(prev => {
                                    const updated = [...prev];
                                    updated[updated.length - 1] = { ...assistantMessage };
                                    return updated;
                                });
                            }
                        } catch (e) {
                        }
                    }
                }
            }

            if (includeSelection) {
                setSelectedText('');
            }
        } catch (error) {
            console.error('Chat stream error:', error);
            const errorMessage = {
                role: 'assistant',
                content: "I'm sorry, I encountered an error. Please try again.",
                isError: true,
                timestamp: new Date().toISOString(),
            };
            setMessages(prev => [...prev, errorMessage]);
        } finally {
            setIsLoading(false);
        }
    }, [sessionId, selectedText]);

    // Clear chat history
    const clearChat = useCallback(async () => {
        if (sessionId) {
            try {
                await fetch(`${API_BASE_URL}/api/chat/${sessionId}`, {
                    method: 'DELETE',
                });
            } catch (error) {
                console.error('Failed to clear chat:', error);
            }
        }
        setMessages([]);
        setSessionId(null);
    }, [sessionId]);

    // Listen for text selection
    React.useEffect(() => {
        document.addEventListener('mouseup', handleTextSelection);
        return () => {
            document.removeEventListener('mouseup', handleTextSelection);
        };
    }, [handleTextSelection]);

    return (
        <div className={styles.chatbotContainer}>
            <ChatIcon onClick={toggleChat} isOpen={isOpen} />

            {isOpen && (
                <ChatModal
                    messages={messages}
                    isLoading={isLoading}
                    selectedText={selectedText}
                    onSendMessage={sendMessageStream}
                    onClearChat={clearChat}
                    onClose={toggleChat}
                    onClearSelection={clearSelectedText}
                />
            )}
        </div>
    );
}
