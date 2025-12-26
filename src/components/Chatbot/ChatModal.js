/**
 * Chat Modal Component
 * Main chat interface with messages and input
 */

import React, { useRef, useEffect } from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import styles from './styles.module.css';

export default function ChatModal({
    messages,
    isLoading,
    selectedText,
    onSendMessage,
    onClearChat,
    onClose,
    onClearSelection,
}) {
    const messagesEndRef = useRef(null);
    const robotImgUrl = useBaseUrl('/img/robot_logo_v2.png');

    // Auto-scroll to bottom when new messages arrive
    useEffect(() => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }, [messages]);

    return (
        <div className={styles.chatModal}>
            {/* Header */}
            <div className={styles.chatHeader}>
                <div className={styles.chatHeaderInfo}>
                    <div className={styles.chatAvatar}>
                        <img src={robotImgUrl} alt="Assistant" className={styles.robotAvatar} />
                    </div>
                    <div>
                        <h3 className={styles.chatTitle}>Book Assistant</h3>
                        <span className={styles.chatSubtitle}>Ask about Physical AI & Robotics</span>
                    </div>
                </div>
                <div className={styles.chatHeaderActions}>
                    <button
                        className={styles.headerButton}
                        onClick={onClearChat}
                        title="Clear chat"
                    >
                        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                            <polyline points="3 6 5 6 21 6" />
                            <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2" />
                        </svg>
                    </button>
                    <button
                        className={styles.headerButton}
                        onClick={onClose}
                        title="Close"
                    >
                        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                            <line x1="18" y1="6" x2="6" y2="18" />
                            <line x1="6" y1="6" x2="18" y2="18" />
                        </svg>
                    </button>
                </div>
            </div>

            {/* Selected Text Indicator */}
            {selectedText && (
                <div className={styles.selectedTextBanner}>
                    <div className={styles.selectedTextContent}>
                        <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor">
                            <path d="M3 17.25V21h3.75L17.81 9.94l-3.75-3.75L3 17.25zM20.71 7.04c.39-.39.39-1.02 0-1.41l-2.34-2.34c-.39-.39-1.02-.39-1.41 0l-1.83 1.83 3.75 3.75 1.83-1.83z" />
                        </svg>
                        <span className={styles.selectedTextLabel}>
                            Selected: "{selectedText.slice(0, 50)}{selectedText.length > 50 ? '...' : ''}"
                        </span>
                    </div>
                    <button
                        className={styles.selectedTextClear}
                        onClick={onClearSelection}
                        title="Clear selection"
                    >
                        <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                            <line x1="18" y1="6" x2="6" y2="18" />
                            <line x1="6" y1="6" x2="18" y2="18" />
                        </svg>
                    </button>
                </div>
            )}

            {/* Messages */}
            <div className={styles.chatMessages}>
                {messages.length === 0 ? (
                    <div className={styles.welcomeMessage}>
                        <div className={styles.welcomeIcon}>
                            <img src={robotImgUrl} alt="Robot" style={{ width: '50px', height: '50px' }} />
                        </div>
                        <h4>Welcome! 👋</h4>
                        <p>
                            I'm your guide to the Physical AI & Humanoid Robotics book.
                            Ask me anything about ROS 2, NVIDIA Isaac, Digital Twins, or any topic covered in the book!
                        </p>
                        <div className={styles.suggestedQuestions}>
                            <span className={styles.suggestLabel}>Try asking:</span>
                            <button
                                className={styles.suggestButton}
                                onClick={() => onSendMessage("What is ROS 2?")}
                            >
                                What is ROS 2?
                            </button>
                            <button
                                className={styles.suggestButton}
                                onClick={() => onSendMessage("Explain nodes and topics")}
                            >
                                Explain nodes and topics
                            </button>
                            <button
                                className={styles.suggestButton}
                                onClick={() => onSendMessage("How do I use Gazebo?")}
                            >
                                How do I use Gazebo?
                            </button>
                        </div>
                    </div>
                ) : (
                    messages.map((msg, index) => (
                        <ChatMessage key={index} message={msg} />
                    ))
                )}

                {isLoading && (
                    <div className={styles.loadingIndicator}>
                        <div className={styles.loadingDot} />
                        <div className={styles.loadingDot} />
                        <div className={styles.loadingDot} />
                    </div>
                )}

                <div ref={messagesEndRef} />
            </div>

            {/* Input */}
            <ChatInput
                onSendMessage={onSendMessage}
                isLoading={isLoading}
                hasSelectedText={!!selectedText}
            />
        </div>
    );
}
