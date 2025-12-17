/**
 * Signup Page
 * Purpose: Render SignupForm and redirect to home after successful registration
 * Date: 2025-12-14
 */

import React, { useEffect } from 'react';
import { useHistory } from '@docusaurus/router';
import Layout from '@theme/Layout';
import { SignupForm } from '../../components/auth/SignupForm';
import { useSession } from '../../components/auth/AuthProvider';

export default function SignupPage(): JSX.Element {
  const history = useHistory();
  const { isAuthenticated, isLoading } = useSession();

  /**
   * Redirect to home if already authenticated
   */
  useEffect(() => {
    if (!isLoading && isAuthenticated) {
      history.push('/');
    }
  }, [isAuthenticated, isLoading, history]);

  /**
   * Handle successful signup
   */
  const handleSignupSuccess = () => {
    // Redirect to home page after successful signup
    history.push('/');
  };

  if (isLoading) {
    return (
      <Layout title="Sign Up">
        <div style={{ textAlign: 'center', padding: '4rem' }}>
          <p>Loading...</p>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Sign Up" description="Create your Physical AI account">
      <main style={{ padding: '2rem 0', minHeight: '80vh' }}>
        <div className="container">
          <SignupForm onSuccess={handleSignupSuccess} />
        </div>
      </main>
    </Layout>
  );
}
