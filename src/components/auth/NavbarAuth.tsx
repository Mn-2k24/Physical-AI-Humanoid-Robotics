/**
 * Navbar Authentication Component
 * Purpose: Show signin/signup when unauthenticated, user menu when authenticated
 * Date: 2025-12-14
 * Updated: 2025-12-24 - T140: Hide auth buttons on mobile, show in sidebar instead
 */

import React, { useState, useEffect } from 'react';
import { useHistory } from '@docusaurus/router';
import { useSession } from './AuthProvider';
import { useSignout } from '../../hooks/useAuth';
import styles from './NavbarAuth.module.css';

export const NavbarAuth: React.FC = () => {
  const history = useHistory();
  const { user, isLoading, isAuthenticated } = useSession();
  const { signout, isLoading: isSigningOut } = useSignout();
  const [showMenu, setShowMenu] = useState(false);
  const [isMobile, setIsMobile] = useState(false);

  // T140: Detect mobile viewport (<768px)
  useEffect(() => {
    const checkMobile = () => {
      setIsMobile(window.innerWidth < 768);
    };

    // Check on mount
    checkMobile();

    // Check on resize
    window.addEventListener('resize', checkMobile);

    return () => {
      window.removeEventListener('resize', checkMobile);
    };
  }, []);

  const handleSignout = async () => {
    await signout();
    setShowMenu(false);
    history.push('/');
  };

  const toggleMenu = () => {
    setShowMenu((prev) => !prev);
  };

  if (isLoading) {
    return <div className={styles.loading}>Loading...</div>;
  }

  // T140: Hide auth buttons on mobile (shown in sidebar instead)
  if (!isAuthenticated) {
  
    return (
      <div className={styles.authButtons}>
        <a href="/auth/signin" className={styles.btnSignin}>
          Sign In
        </a>
        <a href="/auth/signup" className={styles.btnSignup}>
          Sign Up
        </a>
      </div>
    );
  }

  return (
    <div className={styles.userMenu}>
      <button className={styles.userButton} onClick={toggleMenu}>
        <span className={styles.userName}>{user?.full_name || user?.email}</span>
        <svg
          width="12"
          height="12"
          viewBox="0 0 12 12"
          fill="currentColor"
          className={styles.chevron}
        >
          <path d="M6 9L1 4h10z" />
        </svg>
      </button>

      {showMenu && (
        <div className={styles.dropdown}>
          <div className={styles.dropdownHeader}>
            <div className={styles.userEmail}>{user?.email}</div>
            <div className={styles.userExperience}>{user?.experience_level}</div>
          </div>

          <div className={styles.dropdownDivider} />

          <a href="/profile" className={styles.dropdownItem}>
            Profile
          </a>

          <div className={styles.dropdownDivider} />

          <button
            onClick={handleSignout}
            className={styles.dropdownItem}
            disabled={isSigningOut}
          >
            {isSigningOut ? 'Signing out...' : 'Sign Out'}
          </button>
        </div>
      )}
    </div>
  );
};
