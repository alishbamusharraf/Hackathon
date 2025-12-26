/**
 * Chat Message Component
 * Renders individual chat messages with markdown support
 */

import React from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

// Simple markdown renderer
function renderMarkdown(text) {
    if (!text) return null;

    // Split by code blocks first
    const parts = text.split(/(```[\s\S]*?```)/g);

    return parts.map((part, index) => {
        // Code blocks
        if (part.startsWith('```')) {
            const match = part.match(/```(\w+)?\n?([\s\S]*?)```/);
            if (match) {
                const language = match[1] || '';
                const code = match[2] || '';
                return (
                    <pre key={index} className={styles.codeBlock}>
                        {language && <span className={styles.codeLanguage}>{language}</span>}
                        <code>{code.trim()}</code>
                    </pre>
                );
            }
        }

        // Process inline formatting
        let processed = part;

        // Inline code
        processed = processed.replace(
            /`([^`]+)`/g,
            '<code class="inline-code">$1</code>'
        );

        // Bold
        processed = processed.replace(
            /\*\*([^*]+)\*\*/g,
            '<strong>$1</strong>'
        );

        // Italic
        processed = processed.replace(
            /\*([^*]+)\*/g,
            '<em>$1</em>'
        );

        // Line breaks
        processed = processed.replace(/\n/g, '<br />');

        return (
            <span
                key={index}
                dangerouslySetInnerHTML={{ __html: processed }}
            />
        );
    });
}

export default function ChatMessage({ message }) {
    const isUser = message.role === 'user';
    const isError = message.isError;
    const robotImgUrl = useBaseUrl('/img/robot_logo_v2.png');

    return (
        <div
            className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage
                } ${isError ? styles.errorMessage : ''}`}
        >
            {!isUser && (
                <div className={styles.messageAvatar}>
                    <img src={robotImgUrl} alt="Bot" className={styles.robotAvatar} />
                </div>
            )}

            <div className={styles.messageContent}>
                {renderMarkdown(message.content)}

                {/* Sources */}
                {message.sources && message.sources.length > 0 && (
                    <div className={styles.messageSources}>
                        <span className={styles.sourcesLabel}>Sources:</span>
                        {message.sources.map((source, i) => (
                            <span key={i} className={styles.sourceTag}>
                                {source}
                            </span>
                        ))}
                    </div>
                )}
            </div>
        </div>
    );
}
