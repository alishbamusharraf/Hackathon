/**
 * Chatbot Injector - Client Module
 * Injects the chatbot component on all pages after DOM is ready
 */

import React from 'react';
import { createRoot } from 'react-dom/client';
import Chatbot from './components/Chatbot';

// Inject chatbot when DOM is ready
function injectChatbot() {
    // Check if chatbot container already exists
    if (document.getElementById('rag-chatbot-root')) {
        return;
    }

    // Create container element
    const container = document.createElement('div');
    container.id = 'rag-chatbot-root';
    document.body.appendChild(container);

    // Render chatbot
    const root = createRoot(container);
    root.render(<Chatbot />);
}

// Run on initial load
if (typeof window !== 'undefined') {
    // Wait for DOM to be ready
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', injectChatbot);
    } else {
        // DOM is already ready
        injectChatbot();
    }

    // Also handle client-side navigation in Docusaurus
    // Re-check on route changes
    const observer = new MutationObserver(() => {
        if (!document.getElementById('rag-chatbot-root')) {
            injectChatbot();
        }
    });

    // Start observing once DOM is ready
    const startObserver = () => {
        observer.observe(document.body, {
            childList: true,
            subtree: false,
        });
    };

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', startObserver);
    } else {
        startObserver();
    }
}

export default function onRouteDidUpdate() {
    // This is called by Docusaurus on route changes
    // Ensure chatbot is still present after navigation
    setTimeout(() => {
        if (!document.getElementById('rag-chatbot-root')) {
            injectChatbot();
        }
    }, 100);
}
