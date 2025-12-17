/**
 * Root - Docusaurus theme wrapper
 *
 * Provides ChatProvider context and renders ChatWidget + TextSelectionMenu
 */

import React, { createContext, useContext, useState, ReactNode } from 'react';
import ChatWidget from '../components/ChatWidget';
import TextSelectionMenu from '../components/TextSelectionMenu/index';
import { AuthProvider } from '../components/auth/AuthProvider';
import type { Message } from '../components/ChatWidget/MessageList';

interface ChatContextType {
  isOpen: boolean;
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  queryMode: 'global' | 'local';
  togglePanel: () => void;
  sendMessage: (message: string) => Promise<void>;
  sendLocalQuery: (selectedText: string, sourceFilePath: string) => Promise<void>;
  clearError: () => void;
}

const ChatContext = createContext<ChatContextType | undefined>(undefined);

export function useChatWidget() {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChatWidget must be used within ChatProvider');
  }
  return context;
}

interface ChatProviderProps {
  children: ReactNode;
}

function ChatProvider({ children }: ChatProviderProps) {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [queryMode, setQueryMode] = useState<'global' | 'local'>('global');

  const togglePanel = () => {
    setIsOpen(!isOpen);
  };

  const getApiUrl = () => {
    if (typeof window !== 'undefined') {
      // @ts-ignore - customFields from docusaurus.config.ts
      return window.docusaurus?.siteConfig?.customFields?.apiUrl || 'http://localhost:8000';
    }
    return 'http://localhost:8000';
  };

  const sendMessage = async (message: string) => {
    const userMessage: Message = {
      role: 'user',
      content: message,
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);
    setQueryMode('global');

    try {
      const apiUrl = getApiUrl();
      const response = await fetch(`${apiUrl}/chat/global`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include', // Include auth cookies
        body: JSON.stringify({ query: message }),
      });

      if (!response.ok) {
        if (response.status === 401) {
          throw new Error('Please sign in to use the chatbot.');
        }
        if (response.status === 429) {
          throw new Error('Rate limit exceeded. Please try again later.');
        }
        if (response.status === 503) {
          throw new Error('Chatbot temporarily unavailable. Please try again.');
        }
        throw new Error('Failed to get response from chatbot.');
      }

      const data = await response.json();

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred. Please try again.');
      console.error('Chat error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  const sendLocalQuery = async (selectedText: string, sourceFilePath: string) => {
    // **Automatically open chat panel when Ask AI is clicked**
    if (!isOpen) {
      setIsOpen(true);
    }

    const userMessage: Message = {
      role: 'user',
      content: `About selected text: "${selectedText.substring(0, 100)}${
        selectedText.length > 100 ? '...' : ''
      }"`,
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);
    setQueryMode('local');

    try {
      const apiUrl = getApiUrl();
      const response = await fetch(`${apiUrl}/chat/local`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include', // Include auth cookies
        body: JSON.stringify({
          query: `What does this selection explain? ${selectedText}`,
          selected_text: selectedText,
          file_path: sourceFilePath,
        }),
      });

      if (!response.ok) {
        if (response.status === 400) {
          throw new Error('Selected text is too short. Please select at least 50 characters.');
        }
        if (response.status === 401) {
          throw new Error('Please sign in to use the chatbot.');
        }
        if (response.status === 429) {
          throw new Error('Rate limit exceeded. Please try again later.');
        }
        if (response.status === 503) {
          throw new Error('Chatbot temporarily unavailable. Please try again.');
        }
        throw new Error('Failed to get response from chatbot.');
      }

      const data = await response.json();

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred. Please try again.');
      console.error('Chat error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  const clearError = () => {
    setError(null);
  };

  return (
    <ChatContext.Provider
      value={{
        isOpen,
        messages,
        isLoading,
        error,
        queryMode,
        togglePanel,
        sendMessage,
        sendLocalQuery,
        clearError,
      }}
    >
      {children}
      <ChatWidget />
      <TextSelectionMenu />
    </ChatContext.Provider>
  );
}

export default function Root({ children }: { children: ReactNode }) {
  return (
    <AuthProvider>
      <ChatProvider>{children}</ChatProvider>
    </AuthProvider>
  );
}
