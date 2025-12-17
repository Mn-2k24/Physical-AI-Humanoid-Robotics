/**
 * Better Auth Client
 * Purpose: Configure Better Auth with baseURL, credentials: "include"
 * Date: 2025-12-13
 * Updated: 2025-12-14 - Fixed process.env usage for browser compatibility
 */

import { getBackendUrl } from '../utils/config';

const BACKEND_URL = getBackendUrl();

/**
 * Auth client configuration for Better Auth integration
 */
export const authConfig = {
  baseURL: BACKEND_URL,
  credentials: 'include' as RequestCredentials, // Enable HTTP-only cookies
};

/**
 * API client for authentication requests
 */
class AuthClient {
  private baseURL: string;
  private credentials: RequestCredentials;

  constructor(config: typeof authConfig) {
    this.baseURL = config.baseURL;
    this.credentials = config.credentials;
  }

  /**
   * Sign up new user
   */
  async signup(data: {
    email: string;
    password: string;
    full_name: string;
    experience_level: string;
    programming_languages: string[];
    frameworks?: string[];
    available_hardware?: string[];
    robotics_hardware?: string[];
  }) {
    const response = await fetch(`${this.baseURL}/auth/signup`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: this.credentials,
      body: JSON.stringify(data),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Signup failed');
    }

    return response.json();
  }

  /**
   * Sign in existing user
   */
  async signin(data: { email: string; password: string; remember_me?: boolean }) {
    const response = await fetch(`${this.baseURL}/auth/signin`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: this.credentials,
      body: JSON.stringify(data),
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Signin failed');
    }

    return response.json();
  }

  /**
   * Sign out current user
   */
  async signout() {
    const response = await fetch(`${this.baseURL}/auth/signout`, {
      method: 'POST',
      credentials: this.credentials,
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Signout failed');
    }

    return response.json();
  }

  /**
   * Get current user data
   */
  async getCurrentUser() {
    const response = await fetch(`${this.baseURL}/auth/me`, {
      method: 'GET',
      credentials: this.credentials,
    });

    if (!response.ok) {
      if (response.status === 401) {
        return null; // Not authenticated
      }
      const error = await response.json();
      throw new Error(error.detail || 'Failed to get user data');
    }

    return response.json();
  }

  /**
   * Refresh session token
   */
  async refreshToken() {
    const response = await fetch(`${this.baseURL}/auth/refresh`, {
      method: 'POST',
      credentials: this.credentials,
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Token refresh failed');
    }

    return response.json();
  }
}

// Export singleton instance
export const authClient = new AuthClient(authConfig);
