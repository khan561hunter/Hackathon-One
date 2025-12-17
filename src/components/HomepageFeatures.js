import React from 'react';
import styles from './HomepageFeatures.module.css';
import { useTranslation } from '../hooks/useTranslation';

function Feature({icon, title, description}) {
  return (
    <div className={styles.feature}>
      <div className={styles.featureIcon}>
        <span className={styles.icon}>{icon}</span>
      </div>
      <div className={styles.featureContent}>
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  const { t } = useTranslation();

  const FeatureList = [
    {
      title: t('homepage.features.ros2Title'),
      icon: '🤖',
      description: t('homepage.features.ros2Desc'),
    },
    {
      title: t('homepage.features.digitalTwinTitle'),
      icon: '🎮',
      description: t('homepage.features.digitalTwinDesc'),
    },
    {
      title: t('homepage.features.aiPerceptionTitle'),
      icon: '👁️',
      description: t('homepage.features.aiPerceptionDesc'),
    },
    {
      title: t('homepage.features.vlaTitle'),
      icon: '🗣️',
      description: t('homepage.features.vlaDesc'),
    },
    {
      title: t('homepage.features.rlTitle'),
      icon: '🧠',
      description: t('homepage.features.rlDesc'),
    },
    {
      title: t('homepage.features.hardwareTitle'),
      icon: '🚀',
      description: t('homepage.features.hardwareDesc'),
    },
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featuresGrid}>
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
