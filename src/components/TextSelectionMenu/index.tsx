/**
 * TextSelectionMenu - Floating menu for text selection
 *
 * Shows "Ask AI About This" button when user highlights text
 */

import React, { useEffect, useState } from 'react';
import { useChatWidget } from '../../theme/Root';
import styles from './styles.module.css';

interface Position {
  x: number;
  y: number;
}

export default function TextSelectionMenu() {
  const [selectedText, setSelectedText] = useState('');
  const [position, setPosition] = useState<Position | null>(null);
  const { sendLocalQuery } = useChatWidget();

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length >= 10) {
        const range = selection?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();

        if (rect) {
          setSelectedText(text);
          setPosition({
            x: rect.left + rect.width / 2,
            y: rect.top - 10,
          });
        }
      } else {
        setSelectedText('');
        setPosition(null);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
    };
  }, []);

  const handleAskAI = () => {
    if (!selectedText) return;

    // Determine source file path
    const pathname = window.location.pathname;
    const sourceFilePath =
      pathname.replace(/^\/docs\//, 'docs/').replace(/\/$/, '') + '.md';

    // **Send query and automatically open chat**
    sendLocalQuery(selectedText, sourceFilePath);

    // Clear selection
    window.getSelection()?.removeAllRanges();
    setSelectedText('');
    setPosition(null);
  };

  const handleDismiss = () => {
    setSelectedText('');
    setPosition(null);
    window.getSelection()?.removeAllRanges();
  };

  if (!position || !selectedText) return null;

  return (
    <div
      className={styles.menu}
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
      }}
    >
      <button className={styles.askButton} onClick={handleAskAI}>
        <span className={styles.emoji}>🤖</span> Ask AI About This
      </button>
      <button className={styles.dismissButton} onClick={handleDismiss}>
        ✕
      </button>
    </div>
  );
}
