// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  tutorialSidebar: [
    'intro', // Our custom intro.md
    'hardware', // Our custom hardware.md
    {
      type: 'category',
      label: 'ROS 2 Fundamentals',
      items: [
        'ros2/fundamentals',
        'ros2/rclpy-agents',
        'ros2/urdf-modeling',
      ],
    },
    {
      type: 'category',
      label: 'Digital Twins',
      items: [
        'digital-twin/overview',
        'digital-twin/physics',
        'digital-twin/rendering',
        'digital-twin/sensors',
      ],
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac',
      items: [
        'nvidia-isaac/overview',
        'nvidia-isaac/isaac-sim',
        'nvidia-isaac/isaac-ros',
        'nvidia-isaac/nav2',
      ],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA)',
      items: [
        'vla/overview',
        'vla/whisper',
        'vla/llm-planning',
        'vla/multi-step-reasoning',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/overview',
        'capstone/code',
      ],
    },
    'references', // Our custom references.md
  ],
};

export default sidebars;
