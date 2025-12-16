import React, { useEffect, useState } from "react";
import { useAuth } from "@site/src/contexts/AuthContext";
import LoginModal from "./LoginModal";
import SignupModal from "./SignupModal";

/**
 * ProtectedRoute wrapper for Docusaurus docs
 * Shows auth modal when unauthenticated user tries to access protected content
 */
export default function ProtectedRoute({
  children,
  requireAuth = true,
  contextMessage = null,
  pageTitle = "this content"
}) {
  const { isAuthenticated, isLoading, storeIntendedRoute } = useAuth();
  const [showLoginModal, setShowLoginModal] = useState(false);
  const [showSignupModal, setShowSignupModal] = useState(false);
  const [hasCheckedAuth, setHasCheckedAuth] = useState(false);

  useEffect(() => {
    // Don't check until loading is complete
    if (isLoading) return;

    // Only check once after loading completes
    if (!hasCheckedAuth) {
      setHasCheckedAuth(true);

      if (requireAuth && !isAuthenticated) {
        // Store the current route for redirect after auth
        if (typeof window !== "undefined") {
          storeIntendedRoute(window.location.pathname + window.location.search);
        }

        // Show signup modal by default
        setShowSignupModal(true);
      }
    }
  }, [isAuthenticated, isLoading, requireAuth, hasCheckedAuth, storeIntendedRoute]);

  // Generate personalized context message
  const getContextMessage = () => {
    if (contextMessage) return contextMessage;

    return {
      signup: `Sign up to unlock ${pageTitle} and track your learning progress`,
      login: `Sign in to continue reading ${pageTitle}`,
    };
  };

  const handleCloseModal = () => {
    setShowLoginModal(false);
    setShowSignupModal(false);
  };

  const handleSwitchToLogin = () => {
    setShowSignupModal(false);
    setShowLoginModal(true);
  };

  const handleSwitchToSignup = () => {
    setShowLoginModal(false);
    setShowSignupModal(true);
  };

  // Show loading state
  if (isLoading) {
    return (
      <div style={{
        padding: "40px",
        textAlign: "center",
        color: "var(--ifm-color-content)"
      }}>
        Loading...
      </div>
    );
  }

  // If not requiring auth, just render children
  if (!requireAuth) {
    return children;
  }

  // If authenticated, render children
  if (isAuthenticated) {
    return children;
  }

  // Not authenticated - show modal and placeholder content
  return (
    <>
      <div style={{
        padding: "60px 20px",
        textAlign: "center",
        background: "var(--ifm-background-surface-color)",
        borderRadius: "12px",
        margin: "20px 0"
      }}>
        <div style={{
          fontSize: "48px",
          marginBottom: "20px",
          opacity: 0.5
        }}>
          🔒
        </div>
        <h2 style={{
          marginBottom: "16px",
          color: "var(--ifm-color-content)"
        }}>
          Authentication Required
        </h2>
        <p style={{
          fontSize: "16px",
          color: "var(--ifm-color-content-secondary)",
          marginBottom: "24px",
          maxWidth: "500px",
          margin: "0 auto"
        }}>
          {getContextMessage().signup}
        </p>
        <button
          onClick={() => setShowSignupModal(true)}
          style={{
            padding: "12px 32px",
            fontSize: "16px",
            fontWeight: "600",
            color: "white",
            background: "var(--ifm-color-primary)",
            border: "none",
            borderRadius: "8px",
            cursor: "pointer",
            marginRight: "12px"
          }}
        >
          Sign Up to Continue
        </button>
        <button
          onClick={() => setShowLoginModal(true)}
          style={{
            padding: "12px 32px",
            fontSize: "16px",
            fontWeight: "600",
            color: "var(--ifm-color-primary)",
            background: "transparent",
            border: "2px solid var(--ifm-color-primary)",
            borderRadius: "8px",
            cursor: "pointer"
          }}
        >
          Already have an account? Sign In
        </button>
      </div>

      {showLoginModal && (
        <LoginModal
          onClose={handleCloseModal}
          onSwitchToSignup={handleSwitchToSignup}
          contextMessage={getContextMessage().login}
        />
      )}

      {showSignupModal && (
        <SignupModal
          onClose={handleCloseModal}
          onSwitchToLogin={handleSwitchToLogin}
          contextMessage={getContextMessage().signup}
        />
      )}
    </>
  );
}
