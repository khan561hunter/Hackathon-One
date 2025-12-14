import { createAuthClient } from "better-auth/react";

// Determine auth server URL based on environment
const getAuthBaseURL = () => {
  if (typeof window === "undefined") {
    return "http://localhost:3001"; // SSR fallback
  }

  // Development
  if (window.location.hostname === "localhost") {
    return "http://localhost:3001";
  }

  // Production - adjust to your auth server URL
  return process.env.AUTH_SERVER_URL || "http://localhost:3001";
};

export const authClient = createAuthClient({
  baseURL: getAuthBaseURL(),
});

// Export commonly used functions and hooks
export const { signIn, signUp, signOut, useSession, getSession } = authClient;
