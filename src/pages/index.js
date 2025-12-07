import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>
            {siteConfig.title}
          </h1>
          <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
          <p className={styles.heroDescription}>
            A comprehensive guide to building intelligent humanoid robots with
            ROS 2, AI perception, and reinforcement learning
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--primary button--lg', styles.button)}
              to="/docs/intro">
              Start Learning
            </Link>
            <Link
              className={clsx('button button--secondary button--lg', styles.button)}
              to="/docs/ros2-intro">
              ROS 2 Basics
            </Link>
          </div>
        </div>
        <div className={styles.heroAnimation}>
          <div className={styles.robotIcon}>🤖</div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Embodied Intelligence in the Real World - Learn Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <section className={styles.learningPath}>
          <div className="container">
            <h2 className={styles.sectionTitle}>Your Learning Journey</h2>
            <div className={styles.pathGrid}>
              <div className={styles.pathItem}>
                <div className={styles.pathNumber}>1</div>
                <h3>ROS 2 Basic Joint Control</h3>
                <p>Understand fundamentals through simple joint manipulation</p>
              </div>
              <div className={styles.pathArrow}>→</div>
              <div className={styles.pathItem}>
                <div className={styles.pathNumber}>2</div>
                <h3>Humanoid Navigation</h3>
                <p>Simulate robot navigation with obstacle avoidance</p>
              </div>
              <div className={styles.pathArrow}>→</div>
              <div className={styles.pathItem}>
                <div className={styles.pathNumber}>3</div>
                <h3>AI Brain Training</h3>
                <p>Train locomotion policies using reinforcement learning</p>
              </div>
              <div className={styles.pathArrow}>→</div>
              <div className={styles.pathItem}>
                <div className={styles.pathNumber}>4</div>
                <h3>Voice-Controlled Manipulation</h3>
                <p>Capstone project integrating all concepts</p>
              </div>
            </div>
          </div>
        </section>
        <section className={styles.ctaSection}>
          <div className="container">
            <div className={styles.ctaCard}>
              <h2>Ready to Build the Future?</h2>
              <p>
                Join thousands of developers learning to build intelligent robots
                that interact with the physical world.
              </p>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Get Started Now
              </Link>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
