/**
 * Sidebar Authentication Component
 * Purpose: Show auth state and logout button in doc sidebar
 * Date: 2025-12-14
 * Updated: 2025-12-24 - T139: Mobile responsive implemented
 */

import React from 'react';
import { useHistory } from '@docusaurus/router';
import { useSession } from './AuthProvider';
import { useSignout } from '../../hooks/useAuth';
import styles from './SidebarAuth.module.css';

export const SidebarAuth: React.FC = () => {
  const history = useHistory();
  const { user, isLoading, isAuthenticated } = useSession();
  const { signout, isLoading: isSigningOut } = useSignout();

  const handleSignout = async () => {
    await signout();
    history.push('/');
  };

  // Debug: Log component render
  React.useEffect(() => {
    console.log('[SidebarAuth] Rendered - isAuthenticated:', isAuthenticated, 'isLoading:', isLoading);
  }, [isAuthenticated, isLoading]);

  if (isLoading) {
    return (
      <div className={styles.container}>
        <div className={styles.loading}>Loading...</div>
      </div>
    );
  }

  if (!isAuthenticated) {
    return (
      <div className={styles.container} style={{
        display: 'block',
        visibility: 'visible',
        opacity: 1,
        background: '#f0f0f0',
        padding: '1rem',
        minHeight: '100px'
      }}>
        {/* DEBUG: Visible marker */}
        <div style={{
          background: 'red',
          color: 'white',
          padding: '0.5rem',
          marginBottom: '0.5rem',
          fontSize: '12px',
          textAlign: 'center'
        }}>
          🔴 AUTH SECTION - Should be visible
        </div>

        <div className={styles.unauthenticated} style={{
          display: 'flex',
          flexDirection: 'column',
          gap: '1rem'
        }}>
          <p className={styles.message} style={{
            display: 'block',
            visibility: 'visible'
          }}>
            Sign in to access personalized features
          </p>
          <div className={styles.buttons} style={{
            display: 'flex',
            flexDirection: 'column',
            gap: '0.5rem',
            width: '100%'
          }}>
            <a
              href="/auth/signin"
              className={styles.btnSignin}
              style={{
                display: 'block',
                visibility: 'visible',
                opacity: 1,
                minHeight: '44px',
                width: '100%'
              }}
            >
              Sign In
            </a>
            <a
              href="/auth/signup"
              className={styles.btnSignup}
              style={{
                display: 'block',
                visibility: 'visible',
                opacity: 1,
                minHeight: '44px',
                width: '100%'
              }}
            >
              Sign Up
            </a>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <div className={styles.authenticated}>
        <div className={styles.userInfo}>
          <div className={styles.userName}>{user?.full_name}</div>
          <div className={styles.userEmail}>{user?.email}</div>
          <div className={styles.userBadge}>{user?.experience_level}</div>
        </div>

        {/* T143: Add Profile link */}
        <a href="/profile" className={styles.btnProfile}>
          Profile
        </a>

        <button
          onClick={handleSignout}
          className={styles.btnSignout}
          disabled={isSigningOut}
        >
          {isSigningOut ? 'Signing out...' : 'Sign Out'}
        </button>
      </div>
    </div>
  );
};
