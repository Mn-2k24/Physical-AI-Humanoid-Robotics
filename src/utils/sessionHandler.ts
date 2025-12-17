/**
 * Session Expiration Handler
 * Purpose: Detect 401 responses, auto-signout, and redirect to signin
 * Date: 2025-12-14
 */

import { authClient } from '../lib/auth';

let clearSessionCallback: (() => void) | null = null;

/**
 * Register a callback to clear session state
 * Called by AuthProvider on mount
 */
export function registerClearSessionCallback(callback: () => void) {
  clearSessionCallback = callback;
}

/**
 * Handle 401 Unauthorized responses globally
 * Clears session and redirects to signin
 */
export function handleSessionExpiration(currentPath?: string) {
  if (clearSessionCallback) {
    clearSessionCallback();
  }

  // Redirect to signin with current path as redirect param
  if (typeof window !== 'undefined') {
    const redirectPath = currentPath || window.location.pathname;
    const encodedRedirect = encodeURIComponent(redirectPath);
    window.location.href = `/signin?redirect=${encodedRedirect}`;
  }
}

/**
 * Enhanced fetch wrapper that handles 401 responses
 *
 * Usage:
 * const response = await fetchWithAuth('/api/endpoint', options);
 */
export async function fetchWithAuth(url: string, options?: RequestInit): Promise<Response> {
  const response = await fetch(url, {
    ...options,
    credentials: 'include', // Always include credentials for auth
  });

  // Handle session expiration
  if (response.status === 401) {
    handleSessionExpiration();
    throw new Error('Session expired. Please sign in again.');
  }

  return response;
}

/**
 * Global error boundary for authentication errors
 * Can be used in React error boundaries
 */
export function isAuthenticationError(error: any): boolean {
  return (
    error?.response?.status === 401 ||
    error?.status === 401 ||
    error?.message?.includes('Session expired') ||
    error?.message?.includes('Not authenticated')
  );
}
