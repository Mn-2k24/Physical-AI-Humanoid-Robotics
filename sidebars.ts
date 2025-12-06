import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// Sidebar configuration for Physical AI & Humanoid Robotics Book
// Organized by 4 modules with progressive learning path

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Middleware',
      collapsed: false,
      items: [
        'module-1-ros2/index',
        'module-1-ros2/physical-ai-intro',
        'module-1-ros2/ros2-fundamentals',
        'module-1-ros2/nodes-topics-services',
        'module-1-ros2/urdf-models',
        'module-1-ros2/tutorial-01-hello-world',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Simulation)',
      collapsed: true,
      items: [
        'module-2-simulation/index',
        'module-2-simulation/simulation-basics',
        'module-2-simulation/gazebo-physics',
        'module-2-simulation/unity-rendering',
        'module-2-simulation/sensor-simulation',
        'module-2-simulation/tutorial-02-gazebo-humanoid',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Platform',
      collapsed: true,
      items: [
        'module-3-isaac/index',
        'module-3-isaac/isaac-sim-intro',
        'module-3-isaac/synthetic-data',
        'module-3-isaac/isaac-ros-perception',
        'module-3-isaac/nav2-navigation',
        'module-3-isaac/tutorial-03-isaac-vslam',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsed: true,
      items: [
        'module-4-vla/index',
        'module-4-vla/vla-overview',
        'module-4-vla/speech-recognition',
        'module-4-vla/cognitive-planning',
        'module-4-vla/capstone-project',
        'module-4-vla/tutorial-04-voice-control',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      collapsed: true,
      items: [
        'appendices/hardware-setup',
        'appendices/software-installation',
        'appendices/troubleshooting',
        'appendices/glossary',
        'appendices/resources',
      ],
    },
  ],
};

export default sidebars;
