import React from "react";
import { useAuth } from "@site/src/contexts/AuthContext";
import styles from "./styles.module.css";

/**
 * Welcome banner for authenticated users
 * Shows personalized greeting and last visited page
 */
export default function WelcomeBanner() {
  const { user, userProgress } = useAuth();

  if (!user) return null;

  const getGreeting = () => {
    const hour = new Date().getHours();
    if (hour < 12) return "Good morning";
    if (hour < 18) return "Good afternoon";
    return "Good evening";
  };

  const firstName = user.name?.split(" ")[0] || user.email?.split("@")[0];

  return (
    <div className={styles.welcomeBanner}>
      <div className={styles.welcomeContent}>
        <h3 className={styles.welcomeTitle}>
          {getGreeting()}, {firstName}!
        </h3>
        {userProgress?.lastVisited && (
          <p className={styles.welcomeSubtitle}>
            Continue where you left off:{" "}
            <a
              href={`/docs/${userProgress.lastVisited.pageId}`}
              className={styles.welcomeLink}
            >
              {userProgress.lastVisited.pageTitle}
            </a>
          </p>
        )}
        {!userProgress?.lastVisited && (
          <p className={styles.welcomeSubtitle}>
            Ready to start your learning journey?
          </p>
        )}
      </div>
    </div>
  );
}
