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
 * Token storage key
 */
const TOKEN_KEY = 'auth_token';

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
   * Get stored auth token
   */
  getToken(): string | null {
    if (typeof window === 'undefined') return null;
    return localStorage.getItem(TOKEN_KEY);
  }

  /**
   * Store auth token
   */
  private setToken(token: string): void {
    if (typeof window !== 'undefined') {
      localStorage.setItem(TOKEN_KEY, token);
    }
  }

  /**
   * Remove auth token
   */
  private clearToken(): void {
    if (typeof window !== 'undefined') {
      localStorage.removeItem(TOKEN_KEY);
    }
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

    const result = await response.json();

    // Store token for cross-origin requests
    if (result.session?.token) {
      this.setToken(result.session.token);
    }

    return result;
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

    const result = await response.json();

    // Store token for cross-origin requests
    if (result.session?.token) {
      this.setToken(result.session.token);
    }

    return result;
  }

  /**
   * Sign out current user
   */
  async signout() {
    const token = this.getToken();
    const headers: HeadersInit = token ? { 'Authorization': `Bearer ${token}` } : {};

    const response = await fetch(`${this.baseURL}/auth/signout`, {
      method: 'POST',
      headers,
      credentials: this.credentials,
    });

    // Clear token regardless of response
    this.clearToken();

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
    const token = this.getToken();
    const headers: HeadersInit = token ? { 'Authorization': `Bearer ${token}` } : {};

    const response = await fetch(`${this.baseURL}/auth/me`, {
      method: 'GET',
      headers,
      credentials: this.credentials,
    });

    if (!response.ok) {
      if (response.status === 401) {
        this.clearToken(); // Clear invalid token
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
