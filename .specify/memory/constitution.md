# Physical AI & Humanoid Robotics Book Constitution

<!--
SYNC IMPACT REPORT
==================
Version Change: 1.0.0 → 2.0.0
Modified Principles: None (existing principles preserved)
Added Sections:
  - New Principle VII: RAG-Powered Interactive Learning
  - New Focus Area: Interactive AI chatbot integration
  - New Technical Constraint: RAG infrastructure requirements
  - New Success Criterion: Chatbot functionality validation
Removed Sections: None
Templates Updated:
  ✅ plan-template.md - Constitution Check aligns with all principles including new RAG requirements
  ✅ spec-template.md - User scenarios support interactive chatbot features
  ✅ tasks-template.md - Task structure accommodates RAG backend and frontend integration tasks
Follow-up TODOs:
  - Implement RAG chatbot backend (FastAPI + OpenAI + Qdrant)
  - Integrate chatbot UI into Docusaurus
  - Set up Neon Postgres for analytics
  - Create embedding pipeline for book content
==================
-->

## Project Overview

**Book Title**: Physical AI & Humanoid Robotics
**Platform**: Docusaurus
**Management**: Spec-Kit Plus
**Authoring Tools**: Claude CLI / Claude Code
**Publishing**: Vercel (primary), GitHub Pages
**Interactive Features**: RAG-powered AI chatbot for content exploration

## Core Purpose

To produce a publicly accessible, academically grounded, and technically accurate interactive book that explains how AI systems can control humanoid robots in both simulated and real-world physical environments, enhanced with an intelligent chatbot for personalized learning assistance.

**Target Audience**: Students, researchers, and developers transitioning from AI theory to embodied intelligence practice.

## Focus Areas

The book covers the following domains:

- Physical AI fundamentals
- Digital brain → physical body integration
- Humanoid robotic architectures
- Sensor fusion and control systems
- Simulation environments (Gazebo, Mujoco, Isaac Sim, Webots, etc.)
- Real-world deployment challenges
- Safety, ethics, and engineering constraints
- Open-source robotics ecosystems
- Interactive learning through RAG-powered AI assistance

## Core Principles

### I. Accuracy Through Technical Verification

**Rule**: All robotics and AI claims MUST have traceable technical foundations.

- Mathematical or algorithmic explanations MUST be reproducible
- Claims require citation to authoritative sources (peer-reviewed papers, technical documentation)
- No speculative or unverified technical statements
- Code examples MUST be tested and functional

**Rationale**: Technical accuracy is non-negotiable for educational material. Readers rely on this book to build real systems; misinformation could lead to failed implementations or safety issues.

### II. Clarity for Technical Learner Audience

**Rule**: Content MUST assume computer science/engineering background while remaining accessible.

- Use plain technical English without unnecessary jargon
- Define domain-specific terms on first use
- Prefer clear explanations over dense academic prose
- Target Flesch-Kincaid readability grade 11–14

**Rationale**: The audience has technical competency but may be new to robotics. Balance rigor with readability to maximize learning effectiveness.

### III. Hands-On Reproducibility

**Rule**: All examples MUST run using commonly available open-source tools.

- Provide complete simulation examples with parameters and environment settings
- Include setup instructions and dependency lists
- Test all code snippets before publication
- Document version requirements for tools and libraries

**Rationale**: Reproducible examples enable active learning. Readers should be able to run every example on their own systems to build practical skills.

### IV. AI-Assisted Writing Transparency

**Rule**: Sections written using Claude CLI MUST be marked in commit history.

- Commit messages MUST indicate "AI-assisted: [Claude CLI]" for generated content
- Spec-Kit Plus files MUST define structure before content writing begins
- Human review and validation required before merging AI-generated content
- Track authorship distinction: human vs. AI-generated vs. human-edited-AI

**Rationale**: Transparency about AI authorship maintains intellectual honesty and helps identify sections that may need additional human validation.

### V. Citation and Source Quality Standards

**Rule**: All factual claims MUST be traceable to authoritative sources.

- **Minimum 50%** of citations MUST be peer-reviewed robotics research (conferences/journals)
- Remaining citations may include reputable technical documentation (NVIDIA, OpenAI, ROS, Mujoco, etc.)
- Use IEEE or ACM citation style consistently
- **Zero plagiarism tolerance** - all content must be original or properly attributed
- Minimum 30 academic or technical references across the book

**Rationale**: Academic rigor requires proper attribution. High-quality sources ensure technical accuracy and credibility.

### VI. Spec-First Development Workflow

**Rule**: Every major section MUST be specified before content creation.

- Use `/sp.specify` to define chapter structure and requirements
- Use `/sp.plan` for architectural and content design decisions
- Use `/sp.tasks` to break down writing and validation work
- No content writing without a completed specification file

**Rationale**: Spec-first approach ensures systematic coverage, prevents scope creep, and enables progress tracking.

### VII. RAG-Powered Interactive Learning

**Rule**: The book MUST provide an intelligent, retrieval-augmented chatbot for content exploration.

**Core Requirements**:
- **Grounded Responses**: All chatbot answers MUST be sourced strictly from book content (no hallucinations)
- **Dual Query Modes**: Support both full-book queries and user-selected-text queries
- **Retrieval Pipeline**: OpenAI embeddings → Qdrant similarity search → rerank → answer generation
- **Seamless Integration**: Chatbot MUST function directly within the Docusaurus UI using custom React components
- **Text Selection Feature**: UI MUST support text selection → "Ask AI About This" action for contextual queries

**Technical Standards**:
- **Backend**: FastAPI with three endpoints maximum:
  - `/ask` - full-book RAG queries
  - `/ask-local` - selected-text only queries
  - `/track` - conversation analytics to Neon Postgres
- **Vector Database**: Qdrant Cloud Free Tier
  - Collection name: `physical_ai_book`
  - Minimum 1,000 text chunks embedded for full coverage
  - Embedding dimension: 1536 (OpenAI standard)
- **Storage**: Neon Serverless Postgres for logs, conversations, and analytics
- **Performance**: Query latency < 3 seconds
- **Security**: All API keys and environment variables stored server-side only (zero leakage to frontend)
- **Reliability**: Backend MUST be stable under 500 requests/day (free-tier limits)

**Data Management**:
- Logging, embeddings, and metadata MUST follow strict schema in Neon Postgres
- All responses MUST be traceable to source book sections used
- Selected-text mode MUST ignore all unrelated content (strict context isolation)
- Conversation analytics MUST be visible in Neon dashboard

**Success Validation**:
- Chatbot functions fully inside the Docusaurus book UI
- Responses are accurate and verifiable against book content
- End-to-end pipeline deployed and publicly accessible
- Zero security vulnerabilities in API key management

**Rationale**: Modern educational resources require interactive assistance. A RAG-powered chatbot enhances learning by providing instant, contextually relevant answers while ensuring accuracy through retrieval from verified book content. This transforms passive reading into active exploration without compromising technical accuracy.

## Writing Standards

### Required Quality Metrics

- **Word Count**: 25,000–40,000 words total
- **Minimum Sources**: 30 academic or technical references
- **Visual Aids**: At least 20 figures/diagrams
- **Tutorial Sections**: Minimum 5 step-by-step tutorials
- **Readability**: Flesch-Kincaid grade 11–14
- **Format**: Markdown in Docusaurus
- **Interactive Features**: RAG chatbot integrated and functional

### Citation Requirements

- **Primary Sources (≥50%)**: Peer-reviewed conference/journal papers in robotics, AI, control theory
- **Secondary Sources (≤50%)**: Official technical documentation from reputable organizations (NVIDIA, OpenAI, Boston Dynamics, ROS community, etc.)
- **Style**: IEEE or ACM citation format
- **Inline Citations**: All technical claims require citation
- **Bibliography**: Complete references in appendix

### Plagiarism Policy

- **Zero tolerance** for uncited copied content
- All paraphrased content must be cited
- Direct quotes must use quotation marks and page numbers
- Plagiarism check required before publication

## Structural Requirements

### Book Structure (Mandatory Chapters)

1. **Introduction to Physical AI**
2. **Foundations of Humanoid Robotics**
3. **Sensors & Perception**
4. **Locomotion & Control Systems**
5. **Reinforcement Learning in Embodied Agents**
6. **Simulation Environments**
7. **Real-World Robotics Deployment**
8. **Safety, Ethics & Engineering Constraints**
9. **Case Studies & Student Projects**
10. **Glossary + Reference Appendix**

**Note**: Structure MUST be declared in specification files before writing content.

### Chapter Requirements

Each chapter MUST include:

- Learning objectives at the beginning
- Technical explanations with mathematical foundations where applicable
- Code examples or simulation demonstrations
- References to relevant literature
- Summary and key takeaways
- Exercises or projects (where appropriate)
- Chatbot-friendly content structure for effective RAG retrieval

## Technical Workflow

### Tools and Platform

- **Docusaurus**: Documentation site and final book UI
- **Spec-Kit Plus**: Project specification, section control, content governance
- **Claude CLI / Claude Code**: AI-assisted writing, code generation, technical explanations
- **Vercel**: Primary hosting with continuous deployment
- **GitHub Pages**: Secondary hosting option
- **Git**: Version control with clear commit messages
- **RAG Infrastructure**:
  - FastAPI backend for chatbot endpoints
  - OpenAI API for embeddings and chat completion
  - Qdrant Cloud for vector storage and similarity search
  - Neon Postgres for analytics and conversation logs

### Development Rules

1. Every major section MUST be generated through a Spec-Kit spec file
2. Every commit message MUST reflect chapter/section editing
3. Claude-generated sections require manual technical review before merging
4. Commit messages MUST indicate AI vs. human authorship
5. All code examples MUST be tested before committing
6. RAG chatbot changes MUST be tested for accuracy and security before deployment

### Workflow Steps

1. **Specification**: Use `/sp.specify` to define chapter structure
2. **Planning**: Use `/sp.plan` for content architecture
3. **Task Breakdown**: Use `/sp.tasks` for implementation steps
4. **Content Generation**: Write content (AI-assisted or manual)
5. **Technical Review**: Human validation of technical accuracy
6. **Citation Check**: Verify all claims are properly cited
7. **Testing**: Run all code examples and simulations
8. **RAG Integration**: Ensure new content is embedded and retrievable via chatbot
9. **Merge**: Commit to repository with appropriate messages

## Constraints

### Content Constraints

- **Total Word Count**: 25,000–40,000 words
- **Minimum References**: 30 academic/technical citations
- **Minimum Diagrams**: 20 visual explanations
- **Minimum Tutorials**: 5 step-by-step practical guides
- **Code Standards**: All code must be functional, commented, and use open-source tools

### Technical Constraints

- **Format**: Markdown only (Docusaurus-compatible)
- **Deployment**: Vercel (primary) and GitHub Pages (secondary)
- **Licensing**: Open-source friendly (figures must be original or openly licensed)
- **Accessibility**: Content must be readable without AI tools
- **RAG Infrastructure**:
  - Vector collection: `physical_ai_book` in Qdrant
  - Minimum 1,000 text chunks embedded
  - Embedding dimension: 1536
  - Query latency: < 3 seconds
  - Backend endpoints: Maximum 3 (`/ask`, `/ask-local`, `/track`)
  - Environment variables: Server-side only (no client-side exposure)

### Quality Constraints

- **Readability**: Flesch-Kincaid grade 11–14
- **Citation Style**: IEEE or ACM (consistent throughout)
- **Plagiarism**: 0% tolerance
- **Technical Accuracy**: All claims must be verifiable
- **Chatbot Accuracy**: All responses grounded in book content only
- **Security**: Zero API key leakage in frontend

## Success Criteria

A successful book project MUST demonstrate:

1. **Complete Structure**: Full book structure defined in Spec-Kit before writing starts
2. **Technical Accuracy**: Passes plagiarism and technical accuracy review
3. **Practical Examples**: Contains real robotic examples (simulation or hardware)
4. **Tutorial Content**: Provides at least 5 step-by-step tutorial sections
5. **Automated Deployment**: Deploys automatically via Vercel with GitHub integration
6. **Independent Readability**: Readable without Claude or AI tools
7. **Citation Compliance**: Minimum 30 references, 50% peer-reviewed
8. **Visual Content**: At least 20 diagrams/figures
9. **Reproducibility**: All examples run using open-source tools
10. **Interactive Chatbot**: RAG-powered chatbot fully functional within Docusaurus UI
11. **Chatbot Accuracy**: All chatbot responses traceable to book sections
12. **Dual Query Modes**: Both full-book and selected-text queries working
13. **Analytics Dashboard**: Conversation logs visible in Neon Postgres dashboard
14. **Security Compliance**: No API keys exposed in frontend
15. **Performance Target**: Chatbot responses delivered in < 3 seconds
16. **Free-Tier Stability**: Backend stable under 500 requests/day

## Versioning Rules

### Book Versions

- **v1.0 (MVP)**: All major chapters drafted and navigable online
- **v1.1**: Citations, case studies, and diagrams added
- **v1.2**: Simulation code and reproducible experiments added
- **v2.0**: Real-world hardware integration documented + RAG chatbot integrated
- **v2.1**: Enhanced chatbot features (conversation history, multi-turn context)
- **v2.2**: Analytics dashboard and user feedback integration

### Content Versioning

- Track major content changes in git history
- Tag releases corresponding to version milestones
- Maintain changelog documenting significant updates
- Version numbers follow semantic versioning for content maturity

## Review Process

### Five-Stage Review Workflow

1. **Spec-First Stage**
   - Create `/sp.*` specification files
   - Define chapter structure, requirements, and success criteria
   - Approval checkpoint before proceeding

2. **Claude Writing Stage**
   - Generate initial drafts using AI assistance
   - Follow specification requirements
   - Mark all AI-generated content in commits

3. **Human Review Stage**
   - Technical validation by domain expert
   - Citation verification
   - Code testing and validation
   - Readability assessment

4. **Publish Stage**
   - Push to GitHub (triggers Vercel deployment)
   - Automated deployment via CI/CD
   - Version tagging

5. **Feedback Stage**
   - Student and peer review
   - Issue tracking for improvements
   - Iterative enhancements

### Review Checkpoints

Each chapter MUST pass these checks before publication:

- [ ] Specification file complete and approved
- [ ] All technical claims cited
- [ ] All code examples tested and functional
- [ ] Readability meets grade 11–14 standard
- [ ] Figures are original or properly licensed
- [ ] No plagiarism detected
- [ ] Human technical review completed
- [ ] Commit history shows authorship transparency
- [ ] Content embedded in RAG vector database (for v2.0+)
- [ ] Chatbot can accurately answer questions about the chapter (for v2.0+)

## Governance

### Authority and Maintenance

- Single primary author maintains structural specifications
- All changes require specification file updates
- Commit history MUST reflect which tool authored which section
- No chapter is "final" until specification file is satisfied and validated

### Compliance Requirements

- All pull requests MUST verify constitution compliance
- Constitution supersedes conflicting practices
- Amendments require documentation and approval
- Template files must align with constitution principles

### Amendment Process

Constitution amendments MUST follow this process:

1. Propose change with rationale
2. Review impact on existing content and templates
3. Update affected specification and template files
4. Increment constitution version appropriately
5. Document in Sync Impact Report
6. Update commit history

### Complexity and Justification

Any violation of constitution principles MUST be:

- Explicitly documented with justification
- Approved by primary author
- Tracked in project documentation
- Revisited for potential remediation

### Figure and Media Governance

- All figures MUST be original or openly licensed
- Cite sources for adapted diagrams
- Maintain high visual quality standards
- Ensure accessibility (alt text, descriptions)

## Constitution Versioning

**Version**: 2.0.0
**Ratified**: 2025-12-04
**Last Amended**: 2025-12-09

### Version History

- **1.0.0** (2025-12-04): Initial constitution established with complete governance framework for Physical AI & Humanoid Robotics book project
- **2.0.0** (2025-12-09): MAJOR version bump - Added Principle VII (RAG-Powered Interactive Learning) with comprehensive chatbot requirements, infrastructure specifications, and security standards. This is a backward-incompatible change as it introduces mandatory interactive features and new technical infrastructure (FastAPI backend, Qdrant vector DB, Neon Postgres analytics) that fundamentally alter the project scope from static documentation to interactive learning platform.
