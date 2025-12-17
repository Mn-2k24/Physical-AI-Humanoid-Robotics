/**
 * ChatPanel - Slide-over panel for chat interface
 *
 * Contains header, message list, and input box
 */

import React, { useEffect, useRef, useState } from 'react';
import MessageList from './MessageList';
import InputBox from './InputBox';
import { useChatWidget } from '../../theme/Root';
import styles from './styles.module.css';

interface ChatPanelProps {
  isOpen: boolean;
  onClose: () => void;
  initialQuery?: string;
}

export default function ChatPanel({ isOpen, onClose, initialQuery }: ChatPanelProps) {
  const { messages, isLoading, error, queryMode, sendMessage, clearError } = useChatWidget();
  const inputRef = useRef<HTMLTextAreaElement>(null);
  const [hasAutoSent, setHasAutoSent] = useState(false);

  // Auto-send initial query when panel opens with initialQuery
  useEffect(() => {
    if (isOpen && initialQuery && !hasAutoSent) {
      sendMessage(initialQuery);
      setHasAutoSent(true);
      // Focus input after sending
      setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
    }
  }, [isOpen, initialQuery, hasAutoSent, sendMessage]);

  // Reset hasAutoSent when panel closes
  useEffect(() => {
    if (!isOpen) {
      setHasAutoSent(false);
    }
  }, [isOpen]);

  // Focus input when panel opens (without initialQuery)
  useEffect(() => {
    if (isOpen && !initialQuery) {
      setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
    }
  }, [isOpen, initialQuery]);

  if (!isOpen) return null;

  return (
    <>
      {/* Backdrop */}
      <div className={styles.backdrop} onClick={onClose} />

      {/* Panel */}
      <div className={styles.chatPanel}>
        {/* Header */}
        <div className={styles.header}>
          <h3>Ask AI About the Book</h3>
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close chat"
          >
            ✕
          </button>
        </div>

        {/* Context indicator */}
        <div className={queryMode === 'global' ? styles.modeIndicatorGlobal : styles.modeIndicatorLocal}>
          <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor" style={{ marginRight: '6px' }}>
            {queryMode === 'global' ? (
              <path d="M8 0a8 8 0 1 1 0 16A8 8 0 0 1 8 0zm0 1.5a6.5 6.5 0 1 0 0 13 6.5 6.5 0 0 0 0-13z" />
            ) : (
              <path d="M2 2h12v12H2V2zm1 1v10h10V3H3z" />
            )}
          </svg>
          <span>
            {queryMode === 'global' ? 'Global search' : 'Selected text only'}
          </span>
        </div>

        {/* Error banner */}
        {error && (
          <div className={styles.errorBanner}>
            <span>{error}</span>
            <button onClick={clearError} aria-label="Dismiss error">✕</button>
          </div>
        )}

        {/* Messages */}
        <MessageList messages={messages} />

        {/* Input */}
        <InputBox
          onSend={sendMessage}
          isLoading={isLoading}
          disabled={isLoading}
          inputRef={inputRef}
          initialValue={initialQuery && !hasAutoSent ? initialQuery : ''}
        />
      </div>
    </>
  );
}
