/**
 * Signin Page
 * Purpose: Render SigninForm and redirect to destination or home after successful login
 * Date: 2025-12-14
 */

import React, { useEffect } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import Layout from '@theme/Layout';
import { SigninForm } from '../../components/auth/SigninForm';
import { useSession } from '../../components/auth/AuthProvider';

export default function SigninPage(): JSX.Element {
  const history = useHistory();
  const location = useLocation();
  const { isAuthenticated, isLoading } = useSession();

  /**
   * Get redirect destination from query params
   * Example: /signin?redirect=/book/chapter-1
   */
  const getRedirectUrl = (): string => {
    const params = new URLSearchParams(location.search);
    return params.get('redirect') || '/';
  };

  /**
   * Redirect to destination if already authenticated
   */
  useEffect(() => {
    if (!isLoading && isAuthenticated) {
      const redirectUrl = getRedirectUrl();
      history.push(redirectUrl);
    }
  }, [isAuthenticated, isLoading, history]);

  /**
   * Handle successful signin
   */
  const handleSigninSuccess = () => {
    const redirectUrl = getRedirectUrl();
    history.push(redirectUrl);
  };

  if (isLoading) {
    return (
      <Layout title="Sign In">
        <div style={{ textAlign: 'center', padding: '4rem' }}>
          <p>Loading...</p>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Sign In" description="Sign in to your Physical AI account">
      <main style={{ padding: '2rem 0', minHeight: '80vh' }}>
        <div className="container">
          <SigninForm
            onSuccess={handleSigninSuccess}
            destinationUrl={getRedirectUrl() !== '/' ? getRedirectUrl() : undefined}
          />
        </div>
      </main>
    </Layout>
  );
}
