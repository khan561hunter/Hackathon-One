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

export default function LevelSelectionModal({ onComplete, userName }) {
  const [selectedLevel, setSelectedLevel] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const { updateUserProfile } = useAuth();

  const handleLevelSelect = async () => {
    if (!selectedLevel) return;

    setIsLoading(true);

    try {
      // Save level to user profile
      await updateUserProfile({
        difficultyLevel: selectedLevel,
        onboardingComplete: true,
      });

      // Complete onboarding
      onComplete();
    } catch (err) {
      console.error("Failed to save level:", err);
      setIsLoading(false);
    }
  };

  return (
    <div
      className={styles.modal}
      onClick={(e) => e.stopPropagation()}
      style={{
        position: 'relative',
        zIndex: 1000000,
        maxWidth: '500px',
        width: '90%',
        maxHeight: '90vh',
        overflowY: 'auto'
      }}
    >
      <h2 className={styles.modalTitle}>
        Welcome, {userName || "there"}! 👋
      </h2>

        <p className={styles.contextMessage}>
          Let's personalize your learning experience. What's your current level?
        </p>

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

        <button
          onClick={handleLevelSelect}
          disabled={!selectedLevel || isLoading}
          className={styles.primaryButton}
          style={{ marginTop: "24px" }}
        >
          {isLoading ? "Setting up..." : "Continue"}
        </button>

      <p className={styles.skipText}>
        Don't worry, you can change this later in your profile
      </p>
    </div>
  );
}
