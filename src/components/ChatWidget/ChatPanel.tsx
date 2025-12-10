/**
 * ChatPanel - Slide-over panel for chat interface
 *
 * Contains header, message list, and input box
 */

import React from 'react';
import MessageList from './MessageList';
import InputBox from './InputBox';
import { useChatWidget } from '../../theme/Root';
import styles from './styles.module.css';

interface ChatPanelProps {
  isOpen: boolean;
  onClose: () => void;
}

export default function ChatPanel({ isOpen, onClose }: ChatPanelProps) {
  const { messages, isLoading, error, sendMessage, clearError } = useChatWidget();

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
        />
      </div>
    </>
  );
}
