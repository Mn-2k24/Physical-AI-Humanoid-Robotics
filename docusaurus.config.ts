import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Teaching students to control humanoid robots in simulation and real-world environments',
  favicon: 'img/booklogo.png',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://physical-ai-humanoid-robotics-zeta.vercel.app/',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'mn-2k24', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-Humanoid-Robotics', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Custom fields for RAG backend API
  customFields: {
    apiUrl: process.env.NODE_ENV === 'production'
      ? 'https://mn-2k24-physical-ai-humanoid-robotics-backend.hf.space'  // Hugging Face Spaces backend
      : 'http://localhost:8000',
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Edit this page links point to the book repository
          editUrl:
            'https://github.com/Mn-2k24/Physical-AI-Humanoid-Robotics/tree/main/',
        },
        blog: false, // Blog disabled for book-focused content
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/social-card.png',
    // Phase 9 US8: Dark/Light mode support with persistence
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true, // Respect system preference
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/booklogo.svg',
         
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Text-Book',
        },
        {
          type: 'dropdown',
          label: 'Modules',
          position: 'left',
          items: [
            {label: 'Module 1: ROS 2', to: '/docs/module-1-ros2'},
            {label: 'Module 2: Simulation', to: '/docs/module-2-simulation'},
            {label: 'Module 3: Isaac', to: '/docs/module-3-isaac'},
            {label: 'Module 4: VLA', to: '/docs/module-4-vla'},
          ],
        },
       
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            {
              label: 'Module 1: ROS 2',
              to: '/docs/module-1-ros2',
            },
            {
              label: 'Module 2: Simulation',
              to: '/docs/module-2-simulation',
            },
            {
              label: 'Module 3: Isaac',
              to: '/docs/module-3-isaac',
            },
            {
              label: 'Module 4: VLA',
              to: '/docs/module-4-vla',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Code Repository',
              href: 'https://github.com/Mn-2k24/Physical-AI-Humanoid-Robotics',
            },
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/humble/',
            },
            {
              label: 'Gazebo Documentation',
              href: 'https://gazebosim.org/docs',
            },
            {
              label: 'Isaac Sim Documentation',
              href: 'https://docs.omniverse.nvidia.com/isaacsim/latest/',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub (Book)',
              href: 'https://physical-ai-humanoid-robotics-zeta.vercel.app/',
            },
            {
              label: 'GitHub (Code)',
              href: 'https://github.com/Mn-2k24/Physical-AI-Humanoid-Robotics',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.<br><span style="font-size:0.9em; color:#ccc;">Created with ❤️ by Nizam ul din</span>`
      ,
      
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
