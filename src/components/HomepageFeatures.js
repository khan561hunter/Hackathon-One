import React from 'react';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'ROS 2 Fundamentals',
    icon: '🤖',
    description: (
      <>
        Master the Robot Operating System 2, the backbone of modern robotics.
        Learn nodes, topics, services, and more through hands-on tutorials.
      </>
    ),
  },
  {
    title: 'Digital Twin Simulation',
    icon: '🎮',
    description: (
      <>
        Create and control virtual robots in Gazebo and Unity. Build realistic
        simulations before deploying to physical hardware.
      </>
    ),
  },
  {
    title: 'AI-Powered Perception',
    icon: '👁️',
    description: (
      <>
        Integrate NVIDIA Isaac Sim for advanced computer vision, navigation,
        and AI-driven decision making in robotics.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action',
    icon: '🗣️',
    description: (
      <>
        Build robots that understand and act on natural language commands.
        Bridge the gap between human communication and robot actions.
      </>
    ),
  },
  {
    title: 'Reinforcement Learning',
    icon: '🧠',
    description: (
      <>
        Train intelligent locomotion policies using cutting-edge RL techniques.
        Watch your robots learn to walk, run, and navigate autonomously.
      </>
    ),
  },
  {
    title: 'Real-World Deployment',
    icon: '🚀',
    description: (
      <>
        Deploy AI models on NVIDIA Jetson for real robots. Complete capstone
        project with voice-controlled object manipulation.
      </>
    ),
  },
];

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
