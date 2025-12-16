import React, { useState, useEffect, useRef } from "react";
import { signOut } from "../../lib/auth-client";
import { useAuth } from "@site/src/contexts/AuthContext";
import styles from "./styles.module.css";

const LEVEL_ICONS = {
  beginner: "🌱",
  intermediate: "⚡",
  advanced: "🚀",
};

const LEVEL_LABELS = {
  beginner: "Beginner",
  intermediate: "Intermediate",
  advanced: "Advanced",
};

export default function UserMenu({ user, onEditProfile }) {
  const [isOpen, setIsOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const menuRef = useRef(null);
  const { userProfile } = useAuth();

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (menuRef.current && !menuRef.current.contains(event.target)) {
        setIsOpen(false);
      }
    };

    document.addEventListener("mousedown", handleClickOutside);
    return () => document.removeEventListener("mousedown", handleClickOutside);
  }, []);

  const handleSignOut = async () => {
    setIsLoading(true);
    try {
      await signOut({
        fetchOptions: {
          credentials: "include",
        },
      });
      window.location.reload();
    } catch (err) {
      console.error("Sign out failed:", err);
      // Still reload to clear local state even if server call fails
      window.location.reload();
    }
  };

  return (
    <div className={styles.userMenuContainer} ref={menuRef}>
      <button
        className={styles.userButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
        aria-haspopup="true"
      >
        {user.image ? (
          <img src={user.image} alt={user.name || "User"} />
        ) : (
          <div className={styles.userInitial}>
            {(user.name || user.email)[0].toUpperCase()}
          </div>
        )}
        <span className={styles.userName}>
          {user.name || user.email.split("@")[0]}
        </span>
      </button>

      {isOpen && (
        <div className={styles.dropdown}>
          <div className={styles.profileSection}>
            <p className={styles.dropdownEmail}>{user.email}</p>
            {userProfile?.difficultyLevel && (
              <div className={styles.profileLevel}>
                <span className={styles.profileLevelIcon}>
                  {LEVEL_ICONS[userProfile.difficultyLevel]}
                </span>
                <span className={styles.profileLevelText}>
                  {LEVEL_LABELS[userProfile.difficultyLevel]}
                </span>
              </div>
            )}
          </div>
          <div className={styles.dropdownDivider} />
          <button
            onClick={() => {
              setIsOpen(false);
              onEditProfile?.();
            }}
            className={styles.dropdownItem}
          >
            Edit Profile
          </button>
          <button
            onClick={handleSignOut}
            disabled={isLoading}
            className={styles.dropdownItem}
          >
            {isLoading ? "Signing out..." : "Sign Out"}
          </button>
        </div>
      )}
    </div>
  );
}
