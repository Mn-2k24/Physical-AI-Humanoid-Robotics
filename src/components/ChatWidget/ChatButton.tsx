/**
 * ChatButton - Floating toggle button for chat widget
 *
 * Fixed position at bottom-right corner with chat icon
 */

import React from 'react';
import styles from './styles.module.css';

interface ChatButtonProps {
  onClick: () => void;
  isOpen: boolean;
}

export default function ChatButton({ onClick, isOpen }: ChatButtonProps) {
  return (
    <button
      className={styles.chatButton}
      onClick={onClick}
      aria-label={isOpen ? 'Close chat' : 'Open chat'}
      title={isOpen ? 'Close chat' : 'Ask AI about the book'}
    >
      {isOpen ? '✕' : '💬'}
    </button>
  );
}
