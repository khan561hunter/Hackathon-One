import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import HomeWelcome from '@site/src/components/HomeWelcome';
import Translate, {translate} from '@docusaurus/Translate';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>
            <Translate id="homepage.title">Physical AI & Humanoid Robotics</Translate>
          </h1>
          <p className={styles.heroSubtitle}>
            <Translate id="homepage.tagline">Embodied Intelligence in the Real World</Translate>
          </p>
          <p className={styles.heroDescription}>
            <Translate id="homepage.description">
              Master the future of robotics by learning how to build intelligent humanoid robots that can perceive, think, and act in the physical world.
            </Translate>
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--primary button--lg', styles.button)}
              to="/docs/intro">
              <Translate id="homepage.startLearning">Start Learning</Translate>
            </Link>
            <Link
              className={clsx('button button--secondary button--lg', styles.button)}
              to="/docs/ros2-intro">
              <Translate id="homepage.ros2Basics">ROS 2 Basics</Translate>
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
      title={translate({id: 'homepage.title', message: 'Physical AI & Humanoid Robotics'})}
      description={translate({id: 'homepage.tagline', message: 'Embodied Intelligence in the Real World'})}>
      <HomepageHeader />
      <main>
        <div className="container">
          <HomeWelcome />
        </div>
        <HomepageFeatures />
        <section className={styles.learningPath}>
          <div className="container">
            <h2 className={styles.sectionTitle}>
              <Translate id="homepage.learningJourney">Your Learning Journey</Translate>
            </h2>
            <div className={styles.pathGrid}>
              <div className={styles.pathItem}>
                <div className={styles.pathNumber}>1</div>
                <h3><Translate id="homepage.step1Title">Learn ROS 2</Translate></h3>
                <p><Translate id="homepage.step1Desc">Master the Robot Operating System fundamentals</Translate></p>
              </div>
              <div className={styles.pathArrow}>→</div>
              <div className={styles.pathItem}>
                <div className={styles.pathNumber}>2</div>
                <h3><Translate id="homepage.step2Title">Build Digital Twins</Translate></h3>
                <p><Translate id="homepage.step2Desc">Create realistic simulations of humanoid robots</Translate></p>
              </div>
              <div className={styles.pathArrow}>→</div>
              <div className={styles.pathItem}>
                <div className={styles.pathNumber}>3</div>
                <h3><Translate id="homepage.step3Title">Train AI Behaviors</Translate></h3>
                <p><Translate id="homepage.step3Desc">Use reinforcement learning for locomotion and manipulation</Translate></p>
              </div>
              <div className={styles.pathArrow}>→</div>
              <div className={styles.pathItem}>
                <div className={styles.pathNumber}>4</div>
                <h3><Translate id="homepage.step4Title">Deploy to Hardware</Translate></h3>
                <p><Translate id="homepage.step4Desc">Bring your AI to life on real robots</Translate></p>
              </div>
            </div>
          </div>
        </section>
        <section className={styles.ctaSection}>
          <div className="container">
            <div className={styles.ctaCard}>
              <h2><Translate id="homepage.ctaTitle">Ready to Build the Future?</Translate></h2>
              <p><Translate id="homepage.ctaDescription">Join us on this exciting journey into Physical AI and Humanoid Robotics</Translate></p>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                <Translate id="homepage.getStarted">Get Started</Translate>
              </Link>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
