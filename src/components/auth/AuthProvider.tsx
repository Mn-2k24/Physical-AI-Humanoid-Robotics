/**
 * Authentication Context Provider
 * Purpose: Manage authentication state, session persistence, and provide useSession hook
 * Date: 2025-12-14
 */

import React, { createContext, useContext, useEffect, useState, useCallback } from 'react';
import { authClient } from '../../lib/auth';
import { registerClearSessionCallback } from '../../utils/sessionHandler';

interface User {
  id: string;
  email: string;
  full_name: string;
  created_at: string;
  experience_level: string;
  programming_languages: string[];
  frameworks: string[];
  available_hardware: string[];
  robotics_hardware: string[];
}

interface AuthContextType {
  user: User | null;
  isLoading: boolean;
  isAuthenticated: boolean;
  refreshSession: () => Promise<void>;
  clearSession: () => void;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

/**
 * Hook to access authentication state
 * @throws Error if used outside AuthProvider
 */
export const useSession = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useSession must be used within AuthProvider');
  }
  return context;
};

interface AuthProviderProps {
  children: React.ReactNode;
}

/**
 * Authentication Provider Component
 *
 * Features:
 * - Fetches current user on mount
 * - Provides session state to all child components
 * - Handles session refresh and expiration
 * - Persists authentication across page reloads
 */
export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  /**
   * Fetch current user from backend
   * Called on mount and after signin/signup
   */
  const refreshSession = useCallback(async () => {
    try {
      setIsLoading(true);
      const userData = await authClient.getCurrentUser();

      if (userData) {
        setUser(userData);
      } else {
        setUser(null);
      }
    } catch (error) {
      console.error('Failed to fetch session:', error);
      setUser(null);
    } finally {
      setIsLoading(false);
    }
  }, []);

  /**
   * Clear session state (called on signout or session expiration)
   */
  const clearSession = useCallback(() => {
    setUser(null);
  }, []);

  /**
   * Initialize session on mount
   */
  useEffect(() => {
    refreshSession();
  }, [refreshSession]);

  /**
   * Register session expiration handler
   */
  useEffect(() => {
    registerClearSessionCallback(clearSession);
  }, [clearSession]);

  /**
   * Session refresh interval (every 23 hours for 24-hour tokens)
   * Prevents session expiration for active users
   */
  useEffect(() => {
    if (!user) return;

    const refreshInterval = setInterval(
      async () => {
        try {
          await authClient.refreshToken();
          await refreshSession();
        } catch (error) {
          console.error('Token refresh failed:', error);
          clearSession();
        }
      },
      23 * 60 * 60 * 1000 // 23 hours
    );

    return () => clearInterval(refreshInterval);
  }, [user, refreshSession, clearSession]);

  const value: AuthContextType = {
    user,
    isLoading,
    isAuthenticated: !!user,
    refreshSession,
    clearSession,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};
