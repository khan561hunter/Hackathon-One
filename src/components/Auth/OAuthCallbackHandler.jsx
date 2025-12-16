import React, { useEffect, useState } from "react";
import { useAuth } from "@site/src/contexts/AuthContext";
import LevelSelectionModal from "./LevelSelectionModal";

/**
 * Handles OAuth callback and shows level selection for new users
 * This component should be rendered in Root.js to catch OAuth returns
 */
const OAUTH_CHECKED_KEY = "oauth_level_check_done";

export default function OAuthCallbackHandler() {
  const { user, isAuthenticated, userProfile, profileLoaded } = useAuth();
  const [showLevelSelection, setShowLevelSelection] = useState(false);

  useEffect(() => {
    // Only check once after authentication AND profile is loaded
    if (!isAuthenticated || !user || !profileLoaded) {
      return;
    }

    // Check if we've already shown level selection for this session
    const checkKey = `${OAUTH_CHECKED_KEY}_${user.id}`;
    const alreadyChecked = sessionStorage.getItem(checkKey);
    if (alreadyChecked) {
      return;
    }

    // Check if this is a new user without a profile
    if (!userProfile || !userProfile.onboardingComplete) {
      // Mark as checked for this user's session
      sessionStorage.setItem(checkKey, "true");

      // Small delay to ensure everything is loaded
      setTimeout(() => {
        setShowLevelSelection(true);
      }, 300);
    } else {
      // Mark as checked so we don't show again
      sessionStorage.setItem(checkKey, "true");
    }
  }, [isAuthenticated, user, userProfile, profileLoaded]);

  const handleLevelSelectionComplete = () => {
    setShowLevelSelection(false);

    // Clear the session flag since we're redirecting
    sessionStorage.removeItem(OAUTH_CHECKED_KEY);

    // Redirect to homepage
    window.location.href = "/";
  };

  if (!showLevelSelection) return null;

  return (
    <div
      style={{
        position: 'fixed',
        top: 0,
        left: 0,
        right: 0,
        bottom: 0,
        zIndex: 999999,
        backgroundColor: 'rgba(0, 0, 0, 0.7)',
        backdropFilter: 'blur(4px)',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center'
      }}
    >
      <LevelSelectionModal
        onComplete={handleLevelSelectionComplete}
        userName={user?.name || user?.email?.split('@')[0]}
      />
    </div>
  );
}
