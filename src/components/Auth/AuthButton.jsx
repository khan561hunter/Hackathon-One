import React, { useState } from "react";
import { useAuth } from "../../contexts/AuthContext";
import LoginModal from "./LoginModal";
import SignupModal from "./SignupModal";
import UserMenu from "./UserMenu";
import styles from "./styles.module.css";

export default function AuthButton() {
  const { user, isLoading, isAuthenticated } = useAuth();
  const [showLogin, setShowLogin] = useState(false);
  const [showSignup, setShowSignup] = useState(false);

  if (isLoading) {
    return <div className={styles.authLoading}>...</div>;
  }

  if (isAuthenticated && user) {
    return <UserMenu user={user} />;
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
