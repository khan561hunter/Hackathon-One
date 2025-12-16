/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'ROS 2 Fundamentals',
      items: [
        'ros2-intro',
        'ros2-joint-control',
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin Simulation',
      items: [
        'digital-twin-intro',
        'humanoid-navigation',
      ],
    },
    {
      type: 'category',
      label: 'AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'isaac-locomotion-training',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone-vla-manipulation',
      ],
    },
    {
      type: 'category',
      label: 'Hardware Setup',
      items: [
        'hardware-setup',
      ],
    },
    {
      type: 'doc',
      id: 'advanced-topics',
      label: 'Advanced Topics',
    },
  ],
};

module.exports = sidebars;
