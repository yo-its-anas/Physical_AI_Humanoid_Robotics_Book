/**
 * FloatingChat Component
 *
 * Provides a floating chat button and sliding sidebar for the RAG chatbot.
 * Appears on all pages via src/theme/Root.js
 *
 * Features:
 * - Floating button (bottom-right corner, z-index: 1000)
 * - Sliding sidebar (right side, z-index: 999)
 * - Responsive: 400px on desktop, full-screen on mobile
 * - Overlay mode (doesn't push content)
 * - Reuses existing RagChat component
 */

import React, { useState, useEffect } from 'react';
import RagChat from '@site/src/components/RagChat';
import './styles.css';

export default function FloatingChat() {
  const [isOpen, setIsOpen] = useState(false);
  const [isMounted, setIsMounted] = useState(false);

  // Only render on client-side (avoid SSR issues)
  useEffect(() => {
    setIsMounted(true);
  }, []);

  // Prevent body scroll when sidebar is open on mobile
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = '';
    }

    return () => {
      document.body.style.overflow = '';
    };
  }, [isOpen]);

  // Handle escape key to close sidebar
  useEffect(() => {
    const handleEscape = (e) => {
      if (e.key === 'Escape' && isOpen) {
        setIsOpen(false);
      }
    };

    document.addEventListener('keydown', handleEscape);
    return () => document.removeEventListener('keydown', handleEscape);
  }, [isOpen]);

  // Don't render on server-side
  if (!isMounted) {
    return null;
  }

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className={`floating-chat-button ${isOpen ? 'hidden' : ''}`}
        onClick={() => setIsOpen(true)}
        aria-label="Open AI Assistant"
        title="Chat with AI Assistant"
      >
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
          <path d="M8 10h.01M12 10h.01M16 10h.01" />
        </svg>
      </button>

      {/* Backdrop overlay */}
      {isOpen && (
        <div
          className="floating-chat-backdrop"
          onClick={() => setIsOpen(false)}
          aria-hidden="true"
        />
      )}

      {/* Sidebar */}
      <div className={`floating-chat-sidebar ${isOpen ? 'open' : ''}`}>
        {/* Header with close button */}
        <div className="floating-chat-header">
          <h3>AI Assistant</h3>
          <button
            className="floating-chat-close"
            onClick={() => setIsOpen(false)}
            aria-label="Close chat"
            title="Close chat"
          >
            <svg
              width="20"
              height="20"
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
          </button>
        </div>

        {/* Chat content */}
        <div className="floating-chat-content">
          <RagChat />
        </div>
      </div>
    </>
  );
}
