/**
 * InputBox - Chat input field with submit button
 *
 * Textarea for multi-line input, Enter to submit, Shift+Enter for new line
 */

import React, { useState, KeyboardEvent, useEffect } from 'react';
import styles from './styles.module.css';

interface InputBoxProps {
  onSend: (message: string) => void;
  isLoading: boolean;
  disabled: boolean;
  initialValue?: string;
  inputRef?: React.RefObject<HTMLTextAreaElement>;
}

export default function InputBox({ onSend, isLoading, disabled, initialValue = '', inputRef }: InputBoxProps) {
  const [input, setInput] = useState(initialValue);

  // Update input when initialValue changes
  useEffect(() => {
    if (initialValue) {
      setInput(initialValue);
    }
  }, [initialValue]);

  const handleSubmit = () => {
    if (input.trim() && !disabled) {
      onSend(input.trim());
      setInput('');
    }
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  return (
    <div className={styles.inputBox}>
      <textarea
        ref={inputRef}
        className={styles.input}
        value={input}
        onChange={(e) => setInput(e.target.value)}
        onKeyDown={handleKeyDown}
        placeholder="Ask a question about the book..."
        rows={3}
        disabled={disabled}
      />
      <button
        className={styles.sendButton}
        onClick={handleSubmit}
        disabled={disabled || !input.trim()}
        aria-label="Send message"
      >
        {isLoading ? '⏳' : '➤'}
      </button>
    </div>
  );
}
