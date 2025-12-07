import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Chapter 1: The Robotic Nervous System (ROS 2)',
      items: [
        'chapter1/understanding-robot-nervous-system',
        'chapter1/basic-robot-communication',
        'chapter1/high-level-ai-integration',
        'chapter1/robot-description-format',
        'chapter1/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2: The Digital Twin',
      items: [
        'module-2-digital-twin/chapter-2-intro',
        'module-2-digital-twin/gazebo-physics',
        'module-2-digital-twin/ros2-control-integration',
        'module-2-digital-twin/unity-rendering',
        'module-2-digital-twin/sensor-simulation',
        'module-2-digital-twin/full-pipeline',
        'module-2-digital-twin/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 3: The AI-Robot Brain',
      items: [
        'module-3-ai-robot-brain/3.1-ai-brain-fundamentals',
        'module-3-ai-robot-brain/3.2-isaac-sim-synthetic-data',
        'module-3-ai-robot-brain/3.3-isaac-ros-vslam',
        'module-3-ai-robot-brain/3.4-nav2-humanoid-planning',
        'module-3-ai-robot-brain/3.5-complete-pipeline',
      ],
    },
  ],
};

export default sidebars;
