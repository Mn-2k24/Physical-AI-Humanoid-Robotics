/**
 * Authentication Hooks
 * Purpose: Provide useSignup, useSignin, useSignout, useCurrentUser hooks
 * Date: 2025-12-14
 */

import { useState, useCallback } from 'react';
import { authClient } from '../lib/auth';
import { useSession } from '../components/auth/AuthProvider';

/**
 * Signup hook with loading and error states
 */
export const useSignup = () => {
  const { refreshSession } = useSession();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const signup = useCallback(
    async (data: {
      email: string;
      password: string;
      full_name: string;
      experience_level: string;
      programming_languages: string[];
      frameworks?: string[];
      available_hardware?: string[];
      robotics_hardware?: string[];
    }) => {
      try {
        setIsLoading(true);
        setError(null);

        const result = await authClient.signup(data);

        // Refresh session to populate user data
        await refreshSession();

        return result;
      } catch (err: any) {
        const errorMessage = err.message || 'Signup failed. Please try again.';
        setError(errorMessage);
        throw err;
      } finally {
        setIsLoading(false);
      }
    },
    [refreshSession]
  );

  return { signup, isLoading, error };
};

/**
 * Signin hook with loading and error states
 */
export const useSignin = () => {
  const { refreshSession } = useSession();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const signin = useCallback(
    async (data: { email: string; password: string; remember_me?: boolean }) => {
      try {
        setIsLoading(true);
        setError(null);

        const result = await authClient.signin(data);

        // Refresh session to populate user data
        await refreshSession();

        return result;
      } catch (err: any) {
        const errorMessage = err.message || 'Signin failed. Please check your credentials.';
        setError(errorMessage);
        throw err;
      } finally {
        setIsLoading(false);
      }
    },
    [refreshSession]
  );

  return { signin, isLoading, error };
};

/**
 * Signout hook with loading state
 */
export const useSignout = () => {
  const { clearSession } = useSession();
  const [isLoading, setIsLoading] = useState(false);

  const signout = useCallback(async () => {
    try {
      setIsLoading(true);

      await authClient.signout();

      // Clear session state
      clearSession();
    } catch (err: any) {
      console.error('Signout error:', err);
      // Still clear session even if API call fails
      clearSession();
    } finally {
      setIsLoading(false);
    }
  }, [clearSession]);

  return { signout, isLoading };
};

/**
 * Current user hook - shorthand for useSession
 */
export const useCurrentUser = () => {
  const { user, isLoading, isAuthenticated } = useSession();

  return {
    user,
    isLoading,
    isAuthenticated,
  };
};
