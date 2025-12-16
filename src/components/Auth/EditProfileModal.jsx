import React, { useState } from "react";
import { useAuth } from "@site/src/contexts/AuthContext";
import styles from "./styles.module.css";

const DIFFICULTY_LEVELS = [
  {
    id: "beginner",
    label: "Beginner",
    icon: "🌱",
    description: "New to robotics and AI",
  },
  {
    id: "intermediate",
    label: "Intermediate",
    icon: "⚡",
    description: "Some programming experience",
  },
  {
    id: "advanced",
    label: "Advanced",
    icon: "🚀",
    description: "Experienced developer",
  },
];

export default function EditProfileModal({ onClose }) {
  const { user, userProfile, updateUserProfile } = useAuth();
  const [selectedLevel, setSelectedLevel] = useState(userProfile?.difficultyLevel || null);
  const [isLoading, setIsLoading] = useState(false);

  const handleSave = async () => {
    setIsLoading(true);

    try {
      await updateUserProfile({
        difficultyLevel: selectedLevel,
      });

      onClose();
      // Optionally reload to refresh UI
      setTimeout(() => window.location.reload(), 100);
    } catch (err) {
      console.error("Failed to save profile:", err);
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.modalOverlay} onClick={onClose}>
      <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={onClose}>
          &times;
        </button>

        <h2 className={styles.modalTitle}>Edit Profile</h2>

        <div className={styles.profileInfo}>
          <div className={styles.profileInfoRow}>
            <span className={styles.profileInfoLabel}>Name:</span>
            <span className={styles.profileInfoValue}>{user?.name || "Not set"}</span>
          </div>
          <div className={styles.profileInfoRow}>
            <span className={styles.profileInfoLabel}>Email:</span>
            <span className={styles.profileInfoValue}>{user?.email}</span>
          </div>
        </div>

        <div style={{ marginTop: "24px" }}>
          <label className={styles.formLabel}>Difficulty Level</label>
          <div className={styles.levelGrid}>
            {DIFFICULTY_LEVELS.map((level) => (
              <button
                key={level.id}
                className={`${styles.levelCard} ${
                  selectedLevel === level.id ? styles.levelCardSelected : ""
                }`}
                onClick={() => setSelectedLevel(level.id)}
              >
                <div className={styles.levelIcon}>{level.icon}</div>
                <div className={styles.levelLabel}>{level.label}</div>
                <div className={styles.levelDescription}>{level.description}</div>
              </button>
            ))}
          </div>
        </div>

        <div className={styles.modalActions}>
          <button
            onClick={onClose}
            className={styles.secondaryButton}
            disabled={isLoading}
          >
            Cancel
          </button>
          <button
            onClick={handleSave}
            disabled={!selectedLevel || isLoading}
            className={styles.primaryButton}
          >
            {isLoading ? "Saving..." : "Save Changes"}
          </button>
        </div>
      </div>
    </div>
  );
}
