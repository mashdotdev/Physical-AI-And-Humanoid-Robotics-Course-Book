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
  ],
};

export default sidebars;
