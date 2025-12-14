import { createAuthClient } from "better-auth/react";

// Auth server URLs
const AUTH_SERVER_URLS = {
  development: "http://localhost:3001",
  // This will be updated after deploying auth-server to Railway
  production: "https://auth-server-production.up.railway.app", // PLACEHOLDER - will update
};

// Determine auth server URL based on environment
const getAuthBaseURL = () => {
  if (typeof window === "undefined") {
    return AUTH_SERVER_URLS.development; // SSR fallback
  }

  // Development (localhost)
  if (window.location.hostname === "localhost") {
    return AUTH_SERVER_URLS.development;
  }

  // Production
  return AUTH_SERVER_URLS.production;
};

export const authClient = createAuthClient({
  baseURL: getAuthBaseURL(),
});

// Export commonly used functions and hooks
export const { signIn, signUp, signOut, useSession, getSession } = authClient;
