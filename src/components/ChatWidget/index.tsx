/**
 * ChatWidget - Main chat interface component
 *
 * Provides a floating chat button and slide-over panel for RAG-powered Q&A
 */

import React from 'react';
import ChatButton from './ChatButton';
import ChatPanel from './ChatPanel';
import { useChatWidget } from '../../theme/Root';

export default function ChatWidget() {
  const { isOpen, togglePanel } = useChatWidget();

  return (
    <>
    {!isOpen && <ChatButton onClick={togglePanel} isOpen={isOpen} />}
                <ChatPanel isOpen={isOpen} onClose={togglePanel} />

    </>
  );
}
