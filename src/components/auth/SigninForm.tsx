/**
 * Signin Form Component
 * Purpose: Email/password authentication with "Remember Me" option
 * Date: 2025-12-14
 */

import React, { useState } from 'react';
import { authClient } from '../../lib/auth';
import { useSession } from './AuthProvider';
import styles from './AuthForms.module.css';

interface SigninFormProps {
  onSuccess?: () => void;
  destinationUrl?: string;
}

export const SigninForm: React.FC<SigninFormProps> = ({ onSuccess, destinationUrl }) => {
  const { refreshSession } = useSession();
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    remember_me: false,
  });
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  /**
   * Handle input changes
   */
  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value, type, checked } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: type === 'checkbox' ? checked : value,
    }));
    setError('');
  };

  /**
   * Handle form submission
   */
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    // Validation
    if (!formData.email || !formData.password) {
      setError('Email and password are required');
      return;
    }

    try {
      setIsLoading(true);
      setError('');

      await authClient.signin(formData);

      // Refresh session to get user data
      await refreshSession();

      // Call success callback
      if (onSuccess) {
        onSuccess();
      }
    } catch (err: any) {
      setError(err.message || 'Signin failed. Please check your credentials.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.formContainer}>
      <div className={styles.formHeader}>
        <h2>Sign In</h2>
        {destinationUrl && (
          <p className={styles.destinationHint}>
            Sign in to continue to {destinationUrl}
          </p>
        )}
      </div>

      <form onSubmit={handleSubmit} className={styles.form}>
        <div className={styles.formGroup}>
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            placeholder="Enter your email"
            required
            autoFocus
          />
        </div>

        <div className={styles.formGroup}>
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            placeholder="Enter your password"
            required
          />
        </div>

        <div className={styles.formGroup}>
          <label className={styles.checkboxLabel}>
            <input
              type="checkbox"
              name="remember_me"
              checked={formData.remember_me}
              onChange={handleChange}
            />
            Remember me (30 days)
          </label>
        </div>

        {error && <div className={styles.error}>{error}</div>}

        <button type="submit" className={styles.btnPrimary} disabled={isLoading}>
          {isLoading ? 'Signing in...' : 'Sign In'}
        </button>

        <div className={styles.formFooter}>
          <p>
            Don't have an account? <a href="/auth/signup">Sign up</a>
          </p>
        </div>
      </form>
    </div>
  );
};
