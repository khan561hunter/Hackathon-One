import React from "react";
import { useAuth } from "@site/src/contexts/AuthContext";
import Link from "@docusaurus/Link";
import styles from "./HomeWelcome.module.css";

const LEVEL_INFO = {
  beginner: {
    icon: "🌱",
    label: "Beginner",
    description: "Perfect for those new to robotics and AI",
    recommendations: [
      { title: "Introduction", link: "/docs/intro" },
      { title: "ROS 2 Basics", link: "/docs/ros2-intro" },
    ],
  },
  intermediate: {
    icon: "⚡",
    label: "Intermediate",
    description: "For developers with some programming experience",
    recommendations: [
      { title: "ROS 2 Joint Control", link: "/docs/ros2-joint-control" },
      { title: "Digital Twin Simulation", link: "/docs/digital-twin-intro" },
    ],
  },
  advanced: {
    icon: "🚀",
    label: "Advanced",
    description: "For experienced developers ready for complex topics",
    recommendations: [
      { title: "Isaac Locomotion Training", link: "/docs/isaac-locomotion-training" },
      { title: "Advanced Topics", link: "/docs/advanced-topics" },
    ],
  },
};

export default function HomeWelcome() {
  const { user, isAuthenticated, userProfile, userProgress } = useAuth();

  if (!isAuthenticated || !user) return null;

  const firstName = user.name?.split(" ")[0] || user.email?.split("@")[0];
  const level = userProfile?.difficultyLevel || null;
  const levelData = level ? LEVEL_INFO[level] : null;

  const getGreeting = () => {
    const hour = new Date().getHours();
    if (hour < 12) return "Good morning";
    if (hour < 18) return "Good afternoon";
    return "Good evening";
  };

  return (
    <div className={styles.welcomeContainer}>
      <div className={styles.welcomeHeader}>
        <h2 className={styles.welcomeTitle}>
          {getGreeting()}, {firstName}! 👋
        </h2>
        <p className={styles.welcomeSubtitle}>
          Welcome back to your Physical AI learning journey
        </p>
      </div>

      {levelData && (
        <div className={styles.levelSection}>
          <div className={styles.levelBadge}>
            <span className={styles.levelIcon}>{levelData.icon}</span>
            <div>
              <div className={styles.levelLabel}>{levelData.label} Level</div>
              <div className={styles.levelDescription}>
                {levelData.description}
              </div>
            </div>
          </div>

          <div className={styles.recommendations}>
            <h3 className={styles.recommendationsTitle}>
              Recommended for you:
            </h3>
            <div className={styles.recommendationCards}>
              {levelData.recommendations.map((rec) => (
                <Link
                  key={rec.link}
                  to={rec.link}
                  className={styles.recommendationCard}
                >
                  <span className={styles.cardIcon}>📚</span>
                  <span className={styles.cardTitle}>{rec.title}</span>
                  <span className={styles.cardArrow}>→</span>
                </Link>
              ))}
            </div>
          </div>
        </div>
      )}

      {userProgress?.lastVisited && (
        <div className={styles.continueSection}>
          <h3 className={styles.continueTitle}>Continue where you left off</h3>
          <Link
            to={`/docs/${userProgress.lastVisited.pageId}`}
            className={styles.continueCard}
          >
            <div>
              <div className={styles.continueCardTitle}>
                {userProgress.lastVisited.pageTitle}
              </div>
              <div className={styles.continueCardTime}>
                Last visited:{" "}
                {new Date(userProgress.lastVisited.timestamp).toLocaleDateString()}
              </div>
            </div>
            <span className={styles.continueArrow}>→</span>
          </Link>
        </div>
      )}

      {!level && (
        <div className={styles.setupPrompt}>
          <p>
            👋 Complete your profile to get personalized recommendations!
          </p>
          <p className={styles.setupHint}>
            Click your profile menu and select "Edit Profile"
          </p>
        </div>
      )}
    </div>
  );
}
