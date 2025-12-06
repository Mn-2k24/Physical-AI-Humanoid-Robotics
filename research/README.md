# Research Documentation

This directory contains research materials, literature reviews, and reference papers for the Physical AI & Humanoid Robotics book project.

## Directory Structure

```
research/
├── papers/              # Downloaded PDF papers (organized by topic)
├── notes/               # Reading notes and summaries
├── literature-review/   # Comprehensive literature reviews by module
└── datasets/            # Dataset documentation and links
```

## Papers Organization

Organize papers by topic subdirectories:
- `papers/ros2/` - ROS 2 middleware and communication
- `papers/simulation/` - Gazebo, Unity, Isaac Sim research
- `papers/perception/` - Computer vision, SLAM, sensor fusion
- `papers/vla/` - Vision-Language-Action models (RT-1, RT-2, PaLM-E)
- `papers/humanoid/` - Bipedal locomotion, whole-body control
- `papers/rl/` - Reinforcement learning for robotics

## Notes Template

For each paper, create a note file `notes/<topic>/<paper-slug>.md`:

```markdown
# [Paper Title]

**Authors**: [First Author et al.]
**Year**: [2023]
**Venue**: [ICRA / IROS / arXiv]
**Link**: [https://arxiv.org/abs/...]

## Key Contributions
- Point 1
- Point 2

## Methodology
Brief description

## Results
Key findings

## Relevance to Book
How this paper informs Module X, Chapter Y

## BibTeX
\`\`\`bibtex
@article{...}
\`\`\`
```

## Literature Review Structure

Create comprehensive reviews for each module:
- `literature-review/module-1-ros2.md`
- `literature-review/module-2-simulation.md`
- `literature-review/module-3-isaac.md`
- `literature-review/module-4-vla.md`

Each review should:
1. Identify seminal papers (foundational work)
2. Track recent advances (2020-2025)
3. Synthesize research trends
4. Identify gaps/opportunities
5. Connect to book content

## Citation Management

All papers should be added to Zotero collection "Physical AI Book" and exported to `docs/references.bib` for citation in Docusaurus.

## Research Tasks (Phase 2)

See `specs/001-physical-ai-book/tasks.md` for specific research tasks:
- RT-001: ROS 2 literature review
- RT-002: Simulation best practices
- RT-003: VLA model landscape
- RT-004: Dataset identification
- RT-005: Citation management setup

---

**Last Updated**: 2025-12-04
