/**
 * User Profile Component
 * Purpose: Display actual user data from /auth/me endpoint
 * Date: 2025-12-24
 * Task: T142
 */

import React from 'react';
import { useCurrentUser } from '../../hooks/useAuth';
import styles from './UserProfile.module.css';

export const UserProfile: React.FC = () => {
  const { user, isLoading, isAuthenticated } = useCurrentUser();

  if (isLoading) {
    return (
      <div className={styles.container}>
        <div className={styles.loading}>
          <div className={styles.spinner}></div>
          <p>Loading profile...</p>
        </div>
      </div>
    );
  }

  if (!isAuthenticated || !user) {
    return (
      <div className={styles.container}>
        <div className={styles.error}>
          <p>Please sign in to view your profile</p>
          <a href="/auth/signin" className={styles.signinLink}>
            Go to Sign In
          </a>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h1 className={styles.title}>My Profile</h1>
      </div>

      <div className={styles.section}>
        <h2 className={styles.sectionTitle}>Account Information</h2>
        <div className={styles.infoGrid}>
          <div className={styles.infoItem}>
            <span className={styles.label}>Full Name</span>
            <span className={styles.value}>{user.full_name}</span>
          </div>
          <div className={styles.infoItem}>
            <span className={styles.label}>Email</span>
            <span className={styles.value}>{user.email}</span>
          </div>
          <div className={styles.infoItem}>
            <span className={styles.label}>Experience Level</span>
            <span className={styles.badge}>{user.experience_level}</span>
          </div>
        </div>
      </div>

      {user.programming_languages && user.programming_languages.length > 0 && (
        <div className={styles.section}>
          <h2 className={styles.sectionTitle}>Software Background</h2>
          <div className={styles.infoItem}>
            <span className={styles.label}>Programming Languages</span>
            <div className={styles.tags}>
              {user.programming_languages.map((lang, index) => (
                <span key={index} className={styles.tag}>
                  {lang}
                </span>
              ))}
            </div>
          </div>
          {user.frameworks && user.frameworks.length > 0 && (
            <div className={styles.infoItem}>
              <span className={styles.label}>Frameworks</span>
              <div className={styles.tags}>
                {user.frameworks.map((framework, index) => (
                  <span key={index} className={styles.tag}>
                    {framework}
                  </span>
                ))}
              </div>
            </div>
          )}
        </div>
      )}

      {user.available_hardware && user.available_hardware.length > 0 && (
        <div className={styles.section}>
          <h2 className={styles.sectionTitle}>Hardware Background</h2>
          <div className={styles.infoItem}>
            <span className={styles.label}>Available Hardware</span>
            <div className={styles.tags}>
              {user.available_hardware.map((hardware, index) => (
                <span key={index} className={styles.tag}>
                  {hardware}
                </span>
              ))}
            </div>
          </div>
          {user.robotics_hardware && user.robotics_hardware.length > 0 && (
            <div className={styles.infoItem}>
              <span className={styles.label}>Robotics Hardware</span>
              <div className={styles.tags}>
                {user.robotics_hardware.map((hardware, index) => (
                  <span key={index} className={styles.tag}>
                    {hardware}
                  </span>
                ))}
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
};
