import React, { useState } from "react";
import { useAuth } from "../../contexts/AuthContext";
import LoginModal from "./LoginModal";
import SignupModal from "./SignupModal";
import EditProfileModal from "./EditProfileModal";
import UserMenu from "./UserMenu";
import styles from "./styles.module.css";

export default function AuthButton() {
  const { user, isLoading, isAuthenticated, userProfile, profileLoaded } = useAuth();
  const [showLogin, setShowLogin] = useState(false);
  const [showSignup, setShowSignup] = useState(false);
  const [showEditProfile, setShowEditProfile] = useState(false);

  if (isLoading) {
    return <div className={styles.authLoading}>...</div>;
  }

  // IMPORTANT: Keep showing signup modal until onboarding is complete
  // This prevents the modal from disappearing when user becomes authenticated
  if (isAuthenticated && user) {
    // Wait for profile to load before making decisions
    if (!profileLoaded) {
      return <div className={styles.authLoading}>...</div>;
    }

    // If user just signed up and hasn't completed level selection yet
    // Don't show UserMenu - let the signup flow complete first
    if (!userProfile || !userProfile.onboardingComplete) {
      // Keep showing the auth buttons area (which includes the open modal)
      // Don't return null - continue to render the signup modal below
    } else {
      // User is authenticated and onboarding is complete
      return (
        <>
          <UserMenu user={user} onEditProfile={() => setShowEditProfile(true)} />
          {showEditProfile && (
            <EditProfileModal onClose={() => setShowEditProfile(false)} />
          )}
        </>
      );
    }
  }

  return (
    <>
      <div className={styles.authButtons}>
        <button
          className={styles.loginButton}
          onClick={() => setShowLogin(true)}
        >
          Sign In
        </button>
        <button
          className={styles.signupButton}
          onClick={() => setShowSignup(true)}
        >
          Sign Up
        </button>
      </div>

      {showLogin && (
        <LoginModal
          onClose={() => setShowLogin(false)}
          onSwitchToSignup={() => {
            setShowLogin(false);
            setShowSignup(true);
          }}
        />
      )}

      {showSignup && (
        <SignupModal
          onClose={() => setShowSignup(false)}
          onSwitchToLogin={() => {
            setShowSignup(false);
            setShowLogin(true);
          }}
        />
      )}
    </>
  );
}
