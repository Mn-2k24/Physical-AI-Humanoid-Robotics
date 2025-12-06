# Quickstart Guide: Physical AI & Humanoid Robotics Book

**Feature**: Physical AI & Humanoid Robotics Book
**Created**: 2025-12-04
**Purpose**: Onboarding guide for readers, contributors, and authors
**Audience**: Students, educators, and technical contributors

---

## Table of Contents

1. [Reader Quickstart](#reader-quickstart) - How to use the book
2. [Tutorial Setup](#tutorial-setup) - Running hands-on exercises
3. [Contributor Quickstart](#contributor-quickstart) - Contributing improvements
4. [Author Workflow](#author-workflow) - Content development process
5. [Diagram Creation](#diagram-creation) - Creating visual content
6. [Citation Management](#citation-management) - Managing references

---

## Reader Quickstart

### Accessing the Book

**Online Version**: [https://your-username.github.io/physical-ai-book](https://your-username.github.io/physical-ai-book)

**Local Development** (optional):
```bash
# Clone repository
git clone https://github.com/your-username/physical-ai-book.git
cd physical-ai-book

# Install dependencies
npm install

# Start development server
npm start
```

Browser will open at `http://localhost:3000`

### Navigating the Content

**Module Structure**:
- **Module 1**: ROS 2 Middleware (Chapters 1-4) - Foundation
- **Module 2**: Gazebo & Unity Simulation (Chapters 5-8) - Digital Twin
- **Module 3**: NVIDIA Isaac Platform (Chapters 9-12) - AI Perception
- **Module 4**: Vision-Language-Action (Chapters 13-16) - Advanced Integration

**Learning Path**:
1. Start with Module 1, Chapter 1 (Introduction to Physical AI)
2. Progress sequentially through chapters within each module
3. Complete tutorials to reinforce concepts
4. Modules build on each other - don't skip ahead

**Navigation Features**:
- **Sidebar**: Hierarchical module/chapter navigation
- **Previous/Next Buttons**: Bottom of each chapter
- **Search**: Press `/` to search all content
- **Breadcrumbs**: Top of page shows current location

### Reading Tips

**Time Commitment**:
- **Per Chapter**: 1-2 hours reading + exercises
- **Per Module**: 8-12 hours total
- **Full Book**: 30-40 hours over 12-14 weeks

**Active Learning Strategy**:
1. Read chapter to understand concepts
2. Complete hands-on tutorial
3. Run verification script to confirm understanding
4. Experiment with code examples
5. Review references for deeper exploration

**Prerequisites**:
- Basic Linux command line knowledge
- Python programming fundamentals
- Undergraduate-level mathematics (linear algebra, calculus)
- Curiosity about robotics and AI!

---

## Tutorial Setup

### System Requirements

**Minimum Hardware**:
- CPU: 4-core processor (Intel i5/AMD Ryzen 5)
- RAM: 16GB
- GPU: NVIDIA RTX 3060 (6GB VRAM) or equivalent
- Storage: 100GB free space (20GB for software, 80GB for datasets)

**Recommended Hardware**:
- CPU: 8-core processor (Intel i7/AMD Ryzen 7)
- RAM: 32GB
- GPU: NVIDIA RTX 4070 (12GB VRAM) or better
- Storage: 250GB SSD

**Supported Operating Systems**:
- **Primary**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Supported**: Ubuntu 20.04 LTS, Windows 11 + WSL2, macOS (via Docker)

### Software Installation

#### 1. Install ROS 2 Humble

```bash
# Set up sources
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS 2 environment (add to ~/.bashrc for persistence)
source /opt/ros/humble/setup.bash
```

#### 2. Install Gazebo Garden

```bash
# Add Gazebo repository
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Install Gazebo Garden
sudo apt update
sudo apt install -y gz-garden

# Install ROS 2 Gazebo bridge
sudo apt install -y ros-humble-ros-gz
```

#### 3. Install NVIDIA Isaac Sim (Optional - Requires RTX GPU)

**Prerequisites**: NVIDIA GPU Driver 525+ and CUDA 11.8+

```bash
# Download Isaac Sim from Omniverse Launcher
# https://developer.nvidia.com/isaac-sim

# Or use Docker image (recommended)
docker pull nvcr.io/nvidia/isaac-sim:2023.1.0

# Run Isaac Sim container
docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all \
  -e "ACCEPT_EULA=Y" \
  --network=host \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  nvcr.io/nvidia/isaac-sim:2023.1.0
```

#### 4. Clone Tutorial Code Repository

```bash
# Clone companion code repository
git clone https://github.com/your-username/physical-ai-code.git
cd physical-ai-code

# Install Python dependencies
pip install -r requirements.txt

# Verify environment setup
bash scripts/verify-environment.sh
```

### Running Your First Tutorial

**Tutorial 1: ROS 2 Hello World**

1. **Navigate to tutorial directory**:
   ```bash
   cd ~/physical-ai-code/tutorials/module-1-ros2/01-hello-world
   ```

2. **Run setup script**:
   ```bash
   bash setup.sh
   ```

3. **Follow tutorial instructions** in the book: [Tutorial 1: ROS 2 Hello World](https://your-username.github.io/physical-ai-book/module-1-ros2/tutorial-01-hello-world)

4. **Verify completion**:
   ```bash
   python3 verify.py
   ```

   **Expected Output**:
   ```
   ✅ PASS: ROS 2 nodes communicating successfully
      - Publisher node: hello_world_pub [RUNNING]
      - Subscriber node: hello_world_sub [RUNNING]
      - Messages received: 10/10
   ```

### Troubleshooting

**Issue**: `ros2: command not found`
**Solution**: Source ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc for persistence:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

**Issue**: GPU not detected by Isaac Sim
**Solution**: Verify NVIDIA drivers:
```bash
nvidia-smi  # Should show GPU info
```
If not working, reinstall NVIDIA drivers: [NVIDIA Driver Installation Guide](https://docs.nvidia.com/datacenter/tesla/tesla-installation-notes/index.html)

**Issue**: Gazebo not launching
**Solution**: Check DISPLAY environment variable:
```bash
echo $DISPLAY  # Should show :0 or :1
export DISPLAY=:0  # If empty
```

**More Help**: See [Appendix C: Troubleshooting Common Issues](https://your-username.github.io/physical-ai-book/appendices/troubleshooting)

---

## Contributor Quickstart

### How to Contribute

We welcome contributions! You can help by:
- **Reporting Errors**: Open GitHub issue with error details
- **Improving Documentation**: Fix typos, clarify explanations
- **Enhancing Code Examples**: Optimize code, add comments
- **Creating Diagrams**: Improve visual clarity
- **Adding Tutorials**: Propose new hands-on exercises

### Setting Up Development Environment

1. **Fork repositories**:
   - Book: `https://github.com/your-username/physical-ai-book`
   - Code: `https://github.com/your-username/physical-ai-code`

2. **Clone your forks**:
   ```bash
   git clone https://github.com/YOUR-USERNAME/physical-ai-book.git
   git clone https://github.com/YOUR-USERNAME/physical-ai-code.git
   ```

3. **Install development dependencies**:
   ```bash
   cd physical-ai-book
   npm install
   npm start  # Preview changes at localhost:3000
   ```

4. **Create feature branch**:
   ```bash
   git checkout -b fix-chapter-5-typo
   ```

### Making Changes

**Editing Chapter Content**:
1. Navigate to `docs/module-X/chapter-Y.md`
2. Edit Markdown content
3. Preview changes locally (`npm start`)
4. Commit with descriptive message:
   ```bash
   git commit -m "Fix typo in Chapter 5: Gazebo Physics Engine"
   ```

**Adding Code Examples**:
1. Add code to `physical-ai-code/examples/` or `tutorials/`
2. Write verification script (`verify.py`)
3. Test locally:
   ```bash
   python3 verify.py  # Must pass
   ```
4. Update chapter to reference new code

**Creating Diagrams**:
1. Create source file in `docs/assets/diagrams/source/`
2. Export rendered version to `docs/assets/diagrams/rendered/`
3. Embed in chapter with alt text and caption

### Submitting Pull Request

1. **Push changes to your fork**:
   ```bash
   git push origin fix-chapter-5-typo
   ```

2. **Open Pull Request** on GitHub:
   - Compare your branch against `main`
   - Describe changes clearly
   - Reference related issues (if any)

3. **Respond to review feedback**:
   - Address reviewer comments
   - Update PR with fixes
   - Request re-review when ready

4. **Merge**: Once approved, maintainer will merge PR

### Contribution Guidelines

**Code Quality**:
- All Python code follows PEP 8 style
- All code examples include comments
- Verification scripts must pass CI/CD checks

**Content Quality**:
- Flesch-Kincaid readability grade 11-14
- All claims cited with references
- No plagiarism (0% tolerance)

**Diagram Quality**:
- Source files committed (editable format)
- Alt text for accessibility
- Consistent visual style

---

## Author Workflow

### Content Development Pipeline

```
Research → Draft → Review → Test → Publish
```

### Phase 1: Research

**Objective**: Gather authoritative sources for chapter content.

1. **Literature Search**:
   - IEEE Xplore, ACM Digital Library, arXiv
   - Target: 15-20 peer-reviewed papers (50% of 30 citations)

2. **Technical Validation**:
   - Test code examples
   - Verify simulation workflows
   - Document known limitations

3. **Add to Zotero**:
   ```bash
   # Zotero Better BibTeX workflow
   1. Install Zotero and Better BibTeX plugin
   2. Create collection: "Physical AI Book"
   3. Add references with tags (e.g., "ros2", "gazebo", "vla")
   4. Export to BibTeX: File → Export Library → Better BibTeX
   ```

### Phase 2: Drafting

**Content Structure** (per chapter):
```markdown
---
id: chapter-slug
title: Chapter Title
---

# Chapter Title

## Learning Objectives

- Objective 1
- Objective 2
- Objective 3

## Prerequisites

- Prior knowledge or chapters

## Section 1: Concept

[Explanation with diagrams and code examples]

## Section 2: Implementation

[Step-by-step guide]

## Section 3: Validation

[How to verify understanding]

## Summary

[Key takeaways]

## References

[1] Author et al., "Paper Title", Conference/Journal, Year
```

**Writing Guidelines**:
- **Target Readability**: Flesch-Kincaid grade 11-14
- **Paragraph Length**: 3-5 sentences
- **Code Snippets**: 10-30 lines embedded, link to full code
- **Diagrams**: 2-3 per chapter minimum
- **Citations**: Every major claim needs authoritative source

### Phase 3: Review

**Self-Review Checklist**:
- [ ] Learning objectives clearly stated
- [ ] Prerequisites explicitly listed
- [ ] All claims cited with references
- [ ] Code examples tested and functional
- [ ] Diagrams have alt text
- [ ] Readability checked (paste into https://readable.com/)
- [ ] No plagiarism (run through Turnitin)

**Peer Review**:
1. Share draft with 2-3 technical reviewers
2. Request feedback on:
   - Technical accuracy
   - Clarity of explanations
   - Tutorial reproducibility
3. Incorporate feedback

### Phase 4: Testing

**Tutorial Verification**:
1. Beta test with target audience (3-5 readers)
2. Track completion success rate (target: 90%)
3. Collect feedback on:
   - Confusing steps
   - Missing prerequisites
   - Error messages encountered
4. Revise based on feedback

**Code Repository Testing**:
```bash
# Run all verification scripts in CI/CD
cd physical-ai-code
bash scripts/run-all-tests.sh

# Expected: All tests pass
```

### Phase 5: Publishing

**Build and Deploy**:
```bash
# Local preview
npm run build
npm run serve  # Preview production build

# Deploy to GitHub Pages (automated via GitHub Actions)
git push origin main
```

**Post-Publication**:
- Monitor GitHub Issues for reader feedback
- Address errors and clarifications promptly
- Update content as tools evolve

---

## Diagram Creation

### Tool Selection

| Tool | Use Case | Pros | Cons |
|------|----------|------|------|
| **Mermaid.js** | Flowcharts, sequence diagrams | Markdown-native, version control friendly | Limited styling |
| **draw.io** | Architecture diagrams, UML | Feature-rich, web/desktop | Binary XML format |
| **TikZ** (LaTeX) | Mathematical schematics | Publication-quality | Steep learning curve |
| **Inkscape** | Vector illustrations | Professional graphics | Manual layout |

### Mermaid.js Workflow

**Example: ROS 2 Architecture Diagram**

1. **Create source file**: `docs/assets/diagrams/source/ros2-architecture.mmd`
   ```mermaid
   graph TB
       A[ROS 2 Application] --> B[rclpy/rclcpp API]
       B --> C[ROS 2 Middleware]
       C --> D[DDS Layer]
       D --> E[Network Transport]

       style A fill:#e1f5ff
       style D fill:#fff3e0
   ```

2. **Embed in chapter**:
   ```markdown
   ![ROS 2 Architecture showing layers from application to network transport](../../assets/diagrams/source/ros2-architecture.mmd)

   *Figure 1: ROS 2 Architecture - Application layer communicates through middleware to DDS for distributed systems*
   ```

3. **Export rendered version** (Mermaid CLI):
   ```bash
   npm install -g @mermaid-js/mermaid-cli
   mmdc -i ros2-architecture.mmd -o ros2-architecture.svg
   mv ros2-architecture.svg docs/assets/diagrams/rendered/
   ```

### draw.io Workflow

**Example: Gazebo Simulation Pipeline**

1. **Create diagram**: Open draw.io Desktop or Web (https://app.diagrams.net)
2. **Save source**: Export as `.drawio` XML to `docs/assets/diagrams/source/`
3. **Export PNG/SVG**: File → Export as → SVG (recommended for web)
4. **Save rendered output**: To `docs/assets/diagrams/rendered/`
5. **Embed in chapter**:
   ```markdown
   ![Gazebo simulation pipeline from URDF to physics engine to sensors](../../assets/diagrams/rendered/gazebo-pipeline.svg)

   *Figure 2: Gazebo Simulation Pipeline - URDF model loaded into physics engine, sensors publish to ROS 2 topics*
   ```

### TikZ Workflow (Advanced)

**Example: VLA Action Sequence**

1. **Create LaTeX file**: `docs/assets/diagrams/source/vla-sequence.tex`
   ```latex
   \documentclass{standalone}
   \usepackage{tikz}
   \usetikzlibrary{arrows,automata}
   \begin{document}
   \begin{tikzpicture}[>=stealth',shorten >=1pt,auto,node distance=3cm]
     \node[state] (voice) {Voice Input};
     \node[state] (llm) [right of=voice] {LLM Planner};
     \node[state] (ros) [right of=llm] {ROS Actions};
     \path[->] (voice) edge node {Whisper} (llm)
               (llm) edge node {Plan} (ros);
   \end{tikzpicture}
   \end{document}
   ```

2. **Compile to PDF**:
   ```bash
   pdflatex vla-sequence.tex
   ```

3. **Convert to SVG**:
   ```bash
   pdf2svg vla-sequence.pdf vla-sequence.svg
   mv vla-sequence.svg docs/assets/diagrams/rendered/
   ```

### Diagram Quality Checklist

- [ ] Source file committed (editable format)
- [ ] Rendered output high-resolution (300 DPI for print, SVG for web)
- [ ] Alt text describes diagram content
- [ ] Caption explains diagram purpose
- [ ] Consistent color scheme (defined in style guide)
- [ ] Labels readable at standard zoom level (12pt+ font)

---

## Citation Management

### Zotero Setup

1. **Install Zotero**:
   - Download: https://www.zotero.org/download/
   - Install Better BibTeX plugin: https://retorque.re/zotero-better-bibtex/

2. **Create Collection**:
   - Library → New Collection → "Physical AI Book"
   - Create subcollections: "ROS 2", "Gazebo", "Isaac", "VLA", "Other"

3. **Configure Better BibTeX**:
   - Preferences → Better BibTeX → Citation key format:
     ```
     [auth:lower][year][shorttitle:lower]
     ```
   - Example: `smith2022ros2`

### Adding References

**Method 1: Browser Extension**
1. Install Zotero Connector (Chrome/Firefox)
2. Navigate to paper (IEEE Xplore, ACM DL, arXiv)
3. Click Zotero icon in browser → Save to "Physical AI Book" collection

**Method 2: DOI Import**
1. Zotero → Add Item by Identifier (magic wand icon)
2. Paste DOI: `10.1109/ICRA.2022.1234567`
3. Move to appropriate collection

**Method 3: Manual Entry**
1. Right-click collection → New Item → Conference Paper/Journal Article
2. Fill fields: Title, Authors, Year, Venue, DOI

### Exporting to BibTeX

1. **Select collection**: "Physical AI Book"
2. **Export**: File → Export Library
3. **Format**: Better BibTeX
4. **Options**: Keep updated (checkbox)
5. **Save**: `docs/references.bib`

**Example BibTeX Entry**:
```bibtex
@inproceedings{smith2022ros2,
  title = {ROS 2 Design and Implementation for Real-Time Robotics},
  author = {Smith, John and Doe, Jane and Lee, Alice},
  booktitle = {2022 IEEE International Conference on Robotics and Automation (ICRA)},
  year = {2022},
  pages = {1234--1241},
  doi = {10.1109/ICRA.2022.1234567},
  keywords = {ros2, middleware, real-time}
}
```

### Citing in Markdown

**Inline Citation** (IEEE format):
```markdown
ROS 2 Humble uses Data Distribution Service (DDS) for inter-process communication [1].
```

**References Section** (end of chapter):
```markdown
## References

[1] J. Smith, J. Doe, and A. Lee, "ROS 2 Design and Implementation for Real-Time Robotics,"
    in *2022 IEEE International Conference on Robotics and Automation (ICRA)*, 2022, pp. 1234-1241.
    doi: 10.1109/ICRA.2022.1234567
```

### Citation Quality Checklist

- [ ] All major claims have citations
- [ ] 30+ total citations in book
- [ ] 50%+ citations are peer-reviewed (track in Zotero with "peer-reviewed" tag)
- [ ] Citations follow IEEE/ACM format
- [ ] DOIs verified and accessible
- [ ] URLs checked for link rot

### Tracking Citation Metrics

**Zotero Tags for Tracking**:
- `peer-reviewed` - Counts toward 50% quota
- `documentation` - Official technical docs
- `book` - Textbooks and reference books
- `cited-in-ch1` - Chapter 1 reference
- `cited-in-ch2` - Chapter 2 reference
- etc.

**Generate Citation Report**:
1. Zotero → Tools → Generate Report from Collection
2. Review distribution of source types
3. Verify 50% peer-reviewed ratio

---

## Getting Help

### Documentation Resources

- **Book Website**: https://your-username.github.io/physical-ai-book
- **Code Repository**: https://github.com/your-username/physical-ai-code
- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **Gazebo Documentation**: https://gazebosim.org/docs
- **Isaac Sim Documentation**: https://docs.omniverse.nvidia.com/isaacsim/latest/

### Community Support

- **GitHub Issues**: Report bugs, request clarifications
  - Book issues: https://github.com/your-username/physical-ai-book/issues
  - Code issues: https://github.com/your-username/physical-ai-code/issues
- **Discussions**: Ask questions, share projects
  - https://github.com/your-username/physical-ai-book/discussions

### Contact

- **Author Email**: your-email@example.com
- **Office Hours**: [If applicable]

---

## Appendices

### Recommended Learning Resources

**Books**:
- "A Concise Introduction to Robot Programming with ROS 2" (O'Reilly, 2022)
- "Effective Robotics Programming with ROS" (Packt, 2023)
- "Deep Learning for Robotics" (MIT Press, 2024)

**Online Courses**:
- ROS 2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- NVIDIA Isaac Sim Tutorials: https://docs.omniverse.nvidia.com/isaacsim/latest/tutorials.html

**Research Conferences**:
- ICRA (IEEE International Conference on Robotics and Automation)
- IROS (IEEE/RSJ International Conference on Intelligent Robots and Systems)
- RSS (Robotics: Science and Systems)
- CoRL (Conference on Robot Learning)

### Hardware Vendors

**Development Kits**:
- **NVIDIA Jetson Orin**: https://developer.nvidia.com/embedded/jetson-orin
  - Nano (8GB), NX (16GB), AGX (32GB/64GB)
- **Intel RealSense Cameras**: https://www.intelrealsense.com/
  - D435 (Depth), D455 (Long-range), L515 (LiDAR)

**Humanoid Robots** (for advanced learners):
- Research platforms: Boston Dynamics Spot, Unitree Go1/H1
- Educational: Trossen Robotics PincherX, Interbotix LoCoBot

---

**Last Updated**: 2025-12-04
**Version**: 1.0
**Status**: Ready for use
