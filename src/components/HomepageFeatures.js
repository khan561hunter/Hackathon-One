import React from 'react';
import styles from './HomepageFeatures.module.css';
import Translate from '@docusaurus/Translate';

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
  const FeatureList = [
    {
      title: <Translate id="homepage.features.ros2Title">ROS 2 Fundamentals</Translate>,
      icon: '🤖',
      description: <Translate id="homepage.features.ros2Desc">Learn the basics of ROS 2, the Robot Operating System that powers modern robotics applications.</Translate>,
    },
    {
      title: <Translate id="homepage.features.digitalTwinTitle">Digital Twin Simulation</Translate>,
      icon: '🎮',
      description: <Translate id="homepage.features.digitalTwinDesc">Build and test your robots in realistic simulations before deploying to real hardware.</Translate>,
    },
    {
      title: <Translate id="homepage.features.aiPerceptionTitle">AI Perception</Translate>,
      icon: '👁️',
      description: <Translate id="homepage.features.aiPerceptionDesc">Implement computer vision and sensor fusion for robot perception and understanding.</Translate>,
    },
    {
      title: <Translate id="homepage.features.vlaTitle">Voice-Language-Action AI</Translate>,
      icon: '🗣️',
      description: <Translate id="homepage.features.vlaDesc">Build intelligent robots that understand natural language commands and execute them.</Translate>,
    },
    {
      title: <Translate id="homepage.features.rlTitle">Reinforcement Learning</Translate>,
      icon: '🧠',
      description: <Translate id="homepage.features.rlDesc">Train robot behaviors using RL in simulation with NVIDIA Isaac Gym.</Translate>,
    },
    {
      title: <Translate id="homepage.features.hardwareTitle">Hardware Integration</Translate>,
      icon: '🚀',
      description: <Translate id="homepage.features.hardwareDesc">Deploy your AI models to real humanoid robots and edge devices.</Translate>,
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
