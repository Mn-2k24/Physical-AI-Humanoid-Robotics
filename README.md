# Physical AI & Humanoid Robotics

> **A comprehensive educational resource for building, simulating, and deploying humanoid robots using modern AI and robotics tools**

[![Live Site](https://img.shields.io/badge/Live-Site-blue)](https://physical-ai-humanoid-robotics-zeta.vercel.app/)
[![Built with Docusaurus](https://img.shields.io/badge/Built%20with-Docusaurus-green.svg)](https://docusaurus.io/)
[![Node Version](https://img.shields.io/badge/node-%3E%3D20.0-brightgreen)](https://nodejs.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

---

## 📖 About

This is an **interactive educational book and documentation website** that teaches students and developers how to build, simulate, and deploy humanoid robots using cutting-edge Physical AI technologies. The project takes a **simulation-first approach**, eliminating the need for expensive hardware while providing production-ready code and industry-relevant tools.

**🌐 Live Site**: [https://physical-ai-humanoid-robotics-zeta.vercel.app/](https://physical-ai-humanoid-robotics-zeta.vercel.app/)

---

## ✨ Key Features

- 🤖 **Simulation-First**: Learn without a $50,000 robot - use free, open-source simulation tools
- 🎯 **Production-Ready Code**: Complete implementations, not pseudocode
- 🚀 **Modern Stack (2024-2025)**:
  - ROS 2 Humble (not ROS 1)
  - NVIDIA Isaac Sim for photorealistic simulation
  - Vision-Language-Action (VLA) models
  - Jetson Orin edge deployment
- 📚 **Comprehensive Coverage**: 34 chapters across 4 progressive modules
- 🔬 **Academic Rigor**: 14 peer-reviewed citations, BibTeX database, structured learning objectives
- 🛠️ **Practical Tutorials**: 4 hands-on projects with runnable code and verification scripts
- 🌉 **Sim-to-Real Bridge**: Explicit coverage of domain randomization and edge deployment
- 🧩 **Modular Learning**: Each module is self-contained and can be studied independently

---

## 📚 Content Structure

### **Module 1: ROS 2 Middleware** (2-3 weeks)
The robotic nervous system - communication infrastructure for robotics.
- Physical AI Introduction
- ROS 2 Fundamentals
- Nodes, Topics, Services
- URDF Robot Models
- **Tutorial 1**: ROS 2 Hello World

### **Module 2: Digital Twin (Simulation)** (2-3 weeks)
Safe, cost-effective development in virtual environments.
- Simulation Basics
- Gazebo Physics Engine
- Unity Rendering
- Sensor Simulation
- **Tutorial 2**: Gazebo Humanoid with Balance Control

### **Module 3: NVIDIA Isaac Platform** (3-4 weeks)
GPU-accelerated AI and photorealistic simulation.
- Isaac Sim Introduction
- Synthetic Data Generation
- Isaac ROS Perception
- Nav2 Navigation
- **Tutorial 3**: Visual SLAM with Isaac ROS

### **Module 4: Vision-Language-Action (VLA)** (3-4 weeks)
Natural language control of robots.
- VLA Overview
- Speech Recognition (Whisper)
- Cognitive Planning with LLMs
- **Tutorial 4**: Voice-Controlled Humanoid (Capstone Project)

### **Appendices**
- Hardware Setup Guide
- Software Installation (575+ lines of detailed instructions)
- Sim-to-Real Deployment
- Troubleshooting (799 lines)
- Glossary
- Resources

---

## 🎓 Target Audience

- Graduate students in robotics, computer science, or AI
- Software engineers transitioning to robotics
- Researchers needing rapid prototyping capabilities
- Hobbyists with programming experience

### Prerequisites
- **Programming**: Intermediate Python
- **Systems**: Basic Linux command-line
- **Math**: 3D coordinate systems familiarity
- **Hardware**: NVIDIA GPU (RTX 3050+ recommended for Isaac Sim)

---

## 🚀 Quick Start

### Prerequisites

- **Node.js**: >= 20.0
- **npm** or **yarn**
- **Git**

### Installation

Clone the repository:

```bash
git clone https://github.com/Mn-2k24/Physical-AI-Humanoid-Robotics.git
cd Physical-AI-Humanoid-Robotics
```

Install dependencies:

```bash
npm install
# or
yarn install
```

### Local Development

Start the development server:

```bash
npm start
# or
yarn start
```

This command starts a local development server and opens up a browser window at `http://localhost:3000`. Most changes are reflected live without having to restart the server.

### Build

Generate static content for production:

```bash
npm run build
# or
yarn build
```

This command generates static content into the `build/` directory that can be served using any static hosting service.

### Serve Locally

Test the production build locally:

```bash
npm run serve
# or
yarn serve
```

---

## 📁 Project Structure

```
Physical-AI-Humanoid-Robotics/
├── docs/                          # Main documentation content (34 .md files)
│   ├── intro.md                   # Welcome page
│   ├── preface.md                 # Book philosophy and approach
│   ├── learning-objectives.md     # Detailed learning outcomes
│   ├── references.md              # Academic citations
│   ├── module-1-ros2/            # 6 files - ROS 2 fundamentals
│   ├── module-2-simulation/      # 6 files - Gazebo & Unity
│   ├── module-3-isaac/           # 6 files - NVIDIA Isaac platform
│   ├── module-4-vla/             # 6 files - Vision-Language-Action
│   ├── appendices/               # 6 files - Installation, troubleshooting
│   └── assets/                   # Diagrams and images
│
├── src/                          # React/TypeScript customizations
│   ├── components/               # Homepage features component
│   ├── pages/                    # Custom pages (index.tsx)
│   └── css/                      # Custom styling
│
├── static/                       # Static assets
│   └── img/                      # Logo, favicon, social cards
│
├── .specify/                     # Spec-Driven Development framework
│   ├── memory/constitution.md    # Project principles
│   ├── templates/                # PHR templates
│   └── scripts/                  # Automation scripts
│
├── docusaurus.config.ts          # Main configuration
├── sidebars.ts                   # Navigation structure
├── package.json                  # Dependencies
├── tsconfig.json                 # TypeScript configuration
└── README.md                     # This file
```

---

## 🛠️ Technology Stack

- **Framework**: [Docusaurus](https://docusaurus.io/) 3.9.2
- **Frontend**: React 19.0.0 + TypeScript 5.6.2
- **Build System**: Node.js 20+, npm/yarn
- **Deployment**: Vercel (primary) + GitHub Pages support
- **Documentation**: Markdown with MDX support
- **Styling**: Custom CSS with dark mode support

---

## 🌐 Deployment

### Vercel (Recommended)

This project is optimized for Vercel deployment:

1. Push your code to GitHub
2. Import the repository in Vercel
3. Vercel will auto-detect Docusaurus and deploy

The site is currently live at: [https://physical-ai-humanoid-robotics-zeta.vercel.app/](https://physical-ai-humanoid-robotics-zeta.vercel.app/)

### GitHub Pages

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Using HTTPS:

```bash
GIT_USER=<Mn-2k24> yarn deploy
```

This command builds the website and pushes to the `gh-pages` branch.

---

## 📊 Documentation Statistics

- **Total Markdown Files**: 34
- **Total Content**: ~15,000 lines
- **Average File Length**: ~430 lines
- **Longest Sections**:
  - Troubleshooting: 799 lines
  - Software Installation: 575 lines
  - Synthetic Data Generation: 1,060 lines
  - Nav2 Navigation: 831 lines

---

## 🎯 What Makes This Unique

Unlike typical robotics tutorials, this project offers:

1. **No Hardware Required Initially**: Complete simulation-first approach
2. **Production Patterns**: Launch files, error handling, industry best practices
3. **Latest Stack**: Modern ROS 2, Isaac Sim, VLA models (2024-2025)
4. **Complete Examples**: Full working systems, not just code snippets
5. **Troubleshooting Included**: 799 lines addressing common errors and solutions
6. **Sim-to-Real Coverage**: Domain randomization, edge deployment strategies
7. **AI Integration**: Whisper speech recognition, LLM cognitive planning

---

## 🤝 Contributing

We welcome contributions! Here's how you can help:

1. **Report Issues**: Found a bug or have a suggestion? Open an issue
2. **Improve Documentation**: Fix typos, clarify explanations, add examples
3. **Add Tutorials**: Create new hands-on projects
4. **Enhance Code**: Improve existing examples or add new ones

### Development Workflow

This project follows **Spec-Driven Development (SDD)**:
- Review `.specify/memory/constitution.md` for project principles
- Check `specs/` for feature specifications
- Create Prompt History Records (PHRs) in `history/prompts/`
- Document architectural decisions in `history/adr/`

---

## 📜 License

This project is licensed under the **Apache License 2.0** - see the [LICENSE](LICENSE) file for details.

All code examples, datasets, and simulation environments use permissive licenses (Apache 2.0, MIT).

---

## 🙏 Acknowledgments

This project builds upon the work of the open-source robotics community:

- **ROS 2**: Open Robotics Foundation
- **Gazebo**: Open Source Robotics Foundation
- **NVIDIA Isaac**: NVIDIA Corporation
- **Docusaurus**: Meta Open Source

### Academic References

This educational resource cites 14 peer-reviewed sources from leading conferences and journals in robotics and AI. See the [References](https://physical-ai-humanoid-robotics-zeta.vercel.app/references) page for the complete bibliography.

---

## 📞 Contact & Support

- **Documentation**: [https://physical-ai-humanoid-robotics-zeta.vercel.app/](https://physical-ai-humanoid-robotics-zeta.vercel.app/)
- **Issues**: [GitHub Issues](https://github.com/Mn-2k24/Physical-AI-Humanoid-Robotics/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Mn-2k24/Physical-AI-Humanoid-Robotics/discussions)

---

## 🗺️ Roadmap

- [ ] Add video tutorials for each module
- [ ] Create Jupyter notebook versions of tutorials
- [ ] Expand VLA section with more model examples
- [ ] Add real-world deployment case studies
- [ ] Develop assessment quizzes for each module
- [ ] Create Docker containers for consistent development environments

---

## 📈 Project Status

**Status**: Active Development 🚧

This project is actively maintained and regularly updated with new content, bug fixes, and improvements.

**Latest Updates**:
- Responsive UI design improvements
- Logo and branding updates
- Vercel deployment optimization
- Social media card configuration

---

<div align="center">

**Built with ❤️ by Nizam ul din for the robotics and AI community**

⭐ **Star this repository** if you find it helpful!

</div>
