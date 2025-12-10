/**
 * MessageList - Scrollable message list component
 *
 * Displays user and assistant messages with source citations
 */

import React, { useEffect, useRef } from 'react';
import styles from './styles.module.css';

export interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    file_path: string;
    section: string;
    similarity_score: number;
  }>;
  timestamp: Date;
}

interface MessageListProps {
  messages: Message[];
}

export default function MessageList({ messages }: MessageListProps) {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom on new messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSourceClick = (filePath: string, section: string) => {
    // Convert file path to URL
    // e.g., "docs/module-1-ros2/overview.md" -> "/docs/module-1-ros2/overview"
    const urlPath = filePath
      .replace(/^docs\//, '/docs/')
      .replace(/\.md$/, '');

    // Create section anchor (lowercase, hyphens)
    const sectionAnchor = section
      .toLowerCase()
      .replace(/[^\w\s-]/g, '')
      .replace(/\s+/g, '-');

    const fullUrl = `${urlPath}#${sectionAnchor}`;

    // Navigate and highlight
    window.location.href = fullUrl;

    // Add temporary highlight after navigation
    setTimeout(() => {
      const element = document.getElementById(sectionAnchor);
      if (element) {
        element.scrollIntoView({ behavior: 'smooth', block: 'center' });
        element.classList.add('highlight-flash');
        setTimeout(() => element.classList.remove('highlight-flash'), 2000);
      }
    }, 100);
  };

  if (messages.length === 0) {
    return (
      <div className={styles.messageList}>
        <div className={styles.emptyState}>
          <p>👋 Hi! I'm your AI assistant for the Physical AI & Humanoid Robotics book.</p>
          <p>Ask me anything about the book content!</p>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.messageList}>
      {messages.map((message, index) => (
        <div
          key={index}
          className={`${styles.message} ${styles[message.role]}`}
        >
          <div className={styles.messageContent}>
            {message.content}
          </div>

          {/* Source citations */}
          {message.sources && message.sources.length > 0 && (
            <div className={styles.sources}>
              <strong>Sources:</strong>
              <ul>
                {message.sources.map((source, idx) => (
                  <li key={idx}>
                    <button
                      className={styles.sourceLink}
                      onClick={() => handleSourceClick(source.file_path, source.section)}
                    >
                      {source.section} ({(source.similarity_score * 100).toFixed(0)}% match)
                    </button>
                  </li>
                ))}
              </ul>
            </div>
          )}
        </div>
      ))}
      <div ref={messagesEndRef} />
    </div>
  );
}
