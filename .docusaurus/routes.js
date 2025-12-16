import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/docs',
    component: ComponentCreator('/docs', '03c'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '304'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'baa'),
            routes: [
              {
                path: '/docs/advanced-topics',
                component: ComponentCreator('/docs/advanced-topics', '700'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
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
