/**
 * Configuration Utility
 * Purpose: Safely access Docusaurus customFields without using process.env in browser
 * Date: 2025-12-14
 */

import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

/**
 * Get backend API URL from Docusaurus config
 * This is safe for browser usage (no process.env)
 */
export function useBackendUrl(): string {
  const { siteConfig } = useDocusaurusContext();
  return (siteConfig.customFields?.apiUrl as string) || "https://mn-2k24-physical-ai-humanoid-robotics-backend.hf.space";
}

/**
 * Direct access to backend URL (for non-hook contexts)
 * Note: This will be replaced by webpack during build timegit status
 */
export const getBackendUrl = (): string => {
  // In browser, access via window if available
  if (typeof window !== 'undefined' && (window as any).__DOCUSAURUS_BACKEND_URL__) {
    return (window as any).__DOCUSAURUS_BACKEND_URL__;
  }
  // Fallback to production Hugging Face Spaces URL
  return "https://mn-2k24-physical-ai-humanoid-robotics-backend.hf.space";
};
