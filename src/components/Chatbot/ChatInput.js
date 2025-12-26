/**
 * Chat Input Component
 * Message input with send button
 */

import React, { useState, useRef } from 'react';
import styles from './styles.module.css';

export default function ChatInput({ onSendMessage, isLoading, hasSelectedText }) {
    const [input, setInput] = useState('');
    const textareaRef = useRef(null);

    const handleSubmit = (e) => {
        e.preventDefault();
        if (input.trim() && !isLoading) {
            onSendMessage(input.trim(), hasSelectedText);
            setInput('');
            // Reset textarea height
            if (textareaRef.current) {
                textareaRef.current.style.height = 'auto';
            }
        }
    };

    const handleKeyDown = (e) => {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            handleSubmit(e);
        }
    };

    const handleInputChange = (e) => {
        setInput(e.target.value);
        // Auto-resize textarea
        if (textareaRef.current) {
            textareaRef.current.style.height = 'auto';
            textareaRef.current.style.height = `${Math.min(textareaRef.current.scrollHeight, 120)}px`;
        }
    };

    return (
        <form className={styles.chatInputContainer} onSubmit={handleSubmit}>
            {hasSelectedText && (
                <div className={styles.inputContextHint}>
                    <svg width="12" height="12" viewBox="0 0 24 24" fill="currentColor">
                        <path d="M3 17.25V21h3.75L17.81 9.94l-3.75-3.75L3 17.25zM20.71 7.04c.39-.39.39-1.02 0-1.41l-2.34-2.34c-.39-.39-1.02-.39-1.41 0l-1.83 1.83 3.75 3.75 1.83-1.83z" />
                    </svg>
                    Will include selected text
                </div>
            )}

            <div className={styles.inputWrapper}>
                <textarea
                    ref={textareaRef}
                    className={styles.chatInput}
                    value={input}
                    onChange={handleInputChange}
                    onKeyDown={handleKeyDown}
                    placeholder={
                        hasSelectedText
                            ? "Ask about the selected text..."
                            : "Ask a question about the book..."
                    }
                    disabled={isLoading}
                    rows={1}
                />

                <button
                    type="submit"
                    className={styles.sendButton}
                    disabled={!input.trim() || isLoading}
                    title="Send message"
                >
                    {isLoading ? (
                        <div className={styles.sendSpinner} />
                    ) : (
                        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                            <line x1="22" y1="2" x2="11" y2="13" />
                            <polygon points="22 2 15 22 11 13 2 9 22 2" />
                        </svg>
                    )}
                </button>
            </div>
        </form>
    );
}
