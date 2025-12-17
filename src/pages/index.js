import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import HomeWelcome from '@site/src/components/HomeWelcome';
import { useTranslation } from '../hooks/useTranslation';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  const { t } = useTranslation();

  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>
            {t('homepage.title')}
          </h1>
          <p className={styles.heroSubtitle}>{t('homepage.tagline')}</p>
          <p className={styles.heroDescription}>
            {t('homepage.description')}
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--primary button--lg', styles.button)}
              to="/docs/intro">
              {t('homepage.startLearning')}
            </Link>
            <Link
              className={clsx('button button--secondary button--lg', styles.button)}
              to="/docs/ros2-intro">
              {t('homepage.ros2Basics')}
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
  const { t } = useTranslation();

  return (
    <Layout
      title={t('homepage.title')}
      description={t('homepage.tagline')}>
      <HomepageHeader />
      <main>
        <div className="container">
          <HomeWelcome />
        </div>
        <HomepageFeatures />
        <section className={styles.learningPath}>
          <div className="container">
            <h2 className={styles.sectionTitle}>{t('homepage.learningJourney')}</h2>
            <div className={styles.pathGrid}>
              <div className={styles.pathItem}>
                <div className={styles.pathNumber}>1</div>
                <h3>{t('homepage.step1Title')}</h3>
                <p>{t('homepage.step1Desc')}</p>
              </div>
              <div className={styles.pathArrow}>→</div>
              <div className={styles.pathItem}>
                <div className={styles.pathNumber}>2</div>
                <h3>{t('homepage.step2Title')}</h3>
                <p>{t('homepage.step2Desc')}</p>
              </div>
              <div className={styles.pathArrow}>→</div>
              <div className={styles.pathItem}>
                <div className={styles.pathNumber}>3</div>
                <h3>{t('homepage.step3Title')}</h3>
                <p>{t('homepage.step3Desc')}</p>
              </div>
              <div className={styles.pathArrow}>→</div>
              <div className={styles.pathItem}>
                <div className={styles.pathNumber}>4</div>
                <h3>{t('homepage.step4Title')}</h3>
                <p>{t('homepage.step4Desc')}</p>
              </div>
            </div>
          </div>
        </section>
        <section className={styles.ctaSection}>
          <div className="container">
            <div className={styles.ctaCard}>
              <h2>{t('homepage.ctaTitle')}</h2>
              <p>{t('homepage.ctaDescription')}</p>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                {t('homepage.getStarted')}
              </Link>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
