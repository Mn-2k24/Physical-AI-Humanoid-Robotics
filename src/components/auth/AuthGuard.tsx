/**
 * Authentication Guard Component
 * Purpose: Protect routes that require authentication
 * Date: 2025-12-14
 */

import React, { useEffect } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import { useSession } from './AuthProvider';

interface AuthGuardProps {
  children: React.ReactNode;
  fallback?: React.ReactNode;
}

/**
 * AuthGuard - Protect routes that require authentication
 *
 * Usage:
 * <AuthGuard>
 *   <ProtectedContent />
 * </AuthGuard>
 *
 * Features:
 * - Redirects to /signin with ?redirect query param if not authenticated
 * - Shows loading state while checking authentication
 * - Preserves destination URL for post-login redirect
 */
export const AuthGuard: React.FC<AuthGuardProps> = ({ children, fallback }) => {
  const history = useHistory();
  const location = useLocation();
  const { isAuthenticated, isLoading } = useSession();

  useEffect(() => {
    if (!isLoading && !isAuthenticated) {
      // Preserve current path for redirect after signin
      const currentPath = location.pathname + location.search + location.hash;
      const encodedRedirect = encodeURIComponent(currentPath);
      history.push(`/signin?redirect=${encodedRedirect}`);
    }
  }, [isAuthenticated, isLoading, history, location]);

  if (isLoading) {
    return (
      fallback || (
        <div style={{ textAlign: 'center', padding: '4rem' }}>
          <p>Loading...</p>
        </div>
      )
    );
  }

  if (!isAuthenticated) {
    return null; // Will redirect in useEffect
  }

  return <>{children}</>;
};
