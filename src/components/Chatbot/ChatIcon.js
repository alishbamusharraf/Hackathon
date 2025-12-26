/**
 * Floating Chat Icon Component
 * Animated icon that opens the chat modal
 */

import React from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

export default function ChatIcon({ onClick, isOpen }) {
    const robotImgUrl = useBaseUrl('/img/robot_logo_v2.png');

    return (
        <button
            className={`${styles.chatIcon} ${isOpen ? styles.chatIconActive : ''}`}
            onClick={onClick}
            aria-label={isOpen ? 'Close chat' : 'Open chat'}
            title="Chat with the book"
        >
            <div className={styles.chatIconInner}>
                {isOpen ? (
                    // Close icon
                    <svg
                        width="24"
                        height="24"
                        viewBox="0 0 24 24"
                        fill="none"
                        stroke="currentColor"
                        strokeWidth="2"
                        strokeLinecap="round"
                        strokeLinejoin="round"
                    >
                        <line x1="18" y1="6" x2="6" y2="18" />
                        <line x1="6" y1="6" x2="18" y2="18" />
                    </svg>
                ) : (
                    // Chat icon
                    <svg
                        width="24"
                        height="24"
                        viewBox="0 0 24 24"
                        fill="none"
                        stroke="currentColor"
                        strokeWidth="2"
                        strokeLinecap="round"
                        strokeLinejoin="round"
                    >
                        <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
                        <circle cx="9" cy="10" r="1" fill="currentColor" />
                        <circle cx="12" cy="10" r="1" fill="currentColor" />
                        <circle cx="15" cy="10" r="1" fill="currentColor" />
                    </svg>
                )}
            </div>
            {!isOpen && <div className={styles.chatIconPulse} />}
        </button>
    );
}
