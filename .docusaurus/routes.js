import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'c28'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '015'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'ccd'),
            routes: [
              {
                path: '/docs/capstone-vla-manipulation',
                component: ComponentCreator('/docs/capstone-vla-manipulation', '660'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/digital-twin-intro',
                component: ComponentCreator('/docs/digital-twin-intro', 'b48'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/hardware-setup',
                component: ComponentCreator('/docs/hardware-setup', 'df5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/humanoid-navigation',
                component: ComponentCreator('/docs/humanoid-navigation', '950'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-locomotion-training',
                component: ComponentCreator('/docs/isaac-locomotion-training', 'dbc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-intro',
                component: ComponentCreator('/docs/ros2-intro', '765'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-joint-control',
                component: ComponentCreator('/docs/ros2-joint-control', '3fd'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
