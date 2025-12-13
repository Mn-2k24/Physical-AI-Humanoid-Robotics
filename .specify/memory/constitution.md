# Physical AI & Humanoid Robotics Book Constitution

<!--
SYNC IMPACT REPORT
==================
Version Change: 2.0.0 → 2.1.0
Modified Principles:
  - Principle VII enhanced: Added comprehensive RAG chatbot behavioral guidelines and system architecture specifications
Added Sections:
  - Chatbot Identity & Core Purpose in Principle VII
  - Knowledge Source Rules (Book-Based, User-Selected Text, General Questions) in Principle VII
  - Behavioral Guidelines (Accuracy, Style, Anti-Hallucination) in Principle VII
  - System Architecture (Hugging Face models, Gemini via HF, Neon Postgres) in Principle VII
  - Answer Logic Flow (Query Determination, Chunk Retrieval, Mode Application) in Principle VII
  - Safety & Politeness standards in Principle VII
Removed Sections: None
Templates Updated:
  ✅ plan-template.md - Constitution Check includes RAG behavior validation
  ✅ spec-template.md - User scenarios support chatbot behavioral testing
  ✅ tasks-template.md - Task structure accommodates chatbot behavior implementation
Follow-up TODOs:
  - Implement query type detection logic (book vs selection vs general)
  - Implement strict context isolation for selected-text mode
  - Add behavioral validation tests for chatbot responses
  - Document chatbot system prompt generation from constitution
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

**Rule**: The book MUST provide an intelligent, retrieval-augmented chatbot for content exploration with strict behavioral guidelines to ensure accuracy and reliability.

#### Chatbot Identity & Core Purpose

The RAG assistant is an embedded study companion within the published book with the following identity:
- Primary purpose: Help readers understand and explore the book's content with accuracy, clarity, and reliability
- Role: Study assistant, comprehension guide, and grounded RAG system
- Scope: Serves the book content—no more, no less

#### Knowledge Source Rules (CRITICAL)

**Book-Based Answers (Primary Mode)**:
- When users ask questions related to book topics, content, concepts, sections, examples, or ideas:
  - Use ONLY retrieved text chunks from the vector database (Qdrant)
  - NEVER answer from internal model memory or external data
  - If insufficient context exists, ask the user to select text from the book or refine the question
  - Base ALL reasoning strictly on retrieved chunks

**User-Selected Text Mode (Strict Compliance)**:
- If the user selects specific text from the book:
  - Answer using **only that selected text**
  - Ignore all external reasoning
  - Do not add or invent information beyond the selection
  - Maintain strict grounding within the selection

**General Questions (Fallback Mode)**:
- If the user's question is NOT related to the book:
  - Provide normal conversational answers (greetings, everyday questions)
  - Do NOT say "This is not from the book" unless the user expects a book-based answer

#### Behavioral Guidelines

**Accuracy Standards**:
- Always base reasoning on retrieved chunks
- If unsure, respond: *"The available text does not contain enough information to answer this."*
- Never guess missing details or use external knowledge during book-based answering

**Communication Style**:
- Explain concepts clearly in paragraph form
- Provide structured, helpful, human-like explanations
- Offer examples only if they exist in the retrieved content
- Be respectful and concise

**Anti-Hallucination Measures** (MANDATORY):
- Do NOT invent facts
- Do NOT add content not present in retrieved chunks
- Do NOT guess missing details
- Do NOT use external knowledge during book-based answering

#### System Architecture Awareness

The chatbot operates using the following pipeline:
- **Embeddings**: Hugging Face models (NOT OpenAI)
- **LLM**: Gemini models accessed through Hugging Face (NOT through OpenAI APIs)
- **SDK**: ChatKit SDK (local or self-hosted)
- **Backend**: FastAPI
- **Analytics Database**: Neon Serverless Postgres for logs, analytics, and user activity
- **Vector Database**: Qdrant Cloud Free Tier for storing embeddings
- **No OpenAI API Key Required**: System must operate independently of OpenAI infrastructure

#### Answer Logic Flow

The chatbot MUST follow this three-step process:

**Step 1 — Determine Query Type**:
- a) Book-related question
- b) User-selected-text question
- c) General/non-book question

**Step 2 — Retrieve Relevant Chunks**:
- Use embeddings and Qdrant similarity search
- Filter by relevance score
- Rerank if necessary

**Step 3 — Apply the Correct Answer Mode**:
- **Book mode**: Grounded ONLY in retrieved text
- **Selection mode**: Grounded ONLY in selected text
- **General mode**: Free natural answer

#### Safety & Politeness

- Always be respectful and concise
- Never include harmful, violent, or hateful content
- Avoid political opinions, medical claims, or legal advice outside the book's provided content

#### Technical Standards

**Core Requirements**:
- **Grounded Responses**: All chatbot answers MUST be sourced strictly from book content (no hallucinations)
- **Dual Query Modes**: Support both full-book queries and user-selected-text queries
- **Retrieval Pipeline**: Hugging Face embeddings → Qdrant similarity search → rerank → Gemini answer generation
- **Seamless Integration**: Chatbot MUST function directly within the Docusaurus UI using custom React components
- **Text Selection Feature**: UI MUST support text selection → "Ask AI About This" action for contextual queries

**Backend Architecture**:
- **FastAPI** with three endpoints maximum:
  - `/ask` - full-book RAG queries
  - `/ask-local` - selected-text only queries
  - `/track` - conversation analytics to Neon Postgres
- **Vector Database**: Qdrant Cloud Free Tier
  - Collection name: `physical_ai_book`
  - Minimum 1,000 text chunks embedded for full coverage
  - Embedding dimension: configured for Hugging Face model
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
- Query type detection works correctly (book vs selection vs general)
- Selected-text mode strictly isolates context
- Zero hallucinations in book-based answers
- End-to-end pipeline deployed and publicly accessible
- Zero security vulnerabilities in API key management
- No dependency on OpenAI API infrastructure

**Rationale**: Modern educational resources require interactive assistance with strict behavioral guardrails. A RAG-powered chatbot enhances learning by providing instant, contextually relevant answers while ensuring accuracy through retrieval from verified book content and strict adherence to knowledge source rules. The behavioral guidelines ensure the assistant remains a reliable study companion that transforms passive reading into active exploration without compromising technical accuracy or introducing hallucinations.

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
  - Hugging Face models for embeddings
  - Gemini models (accessed via Hugging Face) for chat completion
  - ChatKit SDK for local/self-hosted LLM operations
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

**Version**: 2.1.0
**Ratified**: 2025-12-04
**Last Amended**: 2025-12-11

### Version History

- **1.0.0** (2025-12-04): Initial constitution established with complete governance framework for Physical AI & Humanoid Robotics book project
- **2.0.0** (2025-12-09): MAJOR version bump - Added Principle VII (RAG-Powered Interactive Learning) with comprehensive chatbot requirements, infrastructure specifications, and security standards. This is a backward-incompatible change as it introduces mandatory interactive features and new technical infrastructure (FastAPI backend, Qdrant vector DB, Neon Postgres analytics) that fundamentally alter the project scope from static documentation to interactive learning platform.
- **2.1.0** (2025-12-11): MINOR version bump - Enhanced Principle VII with comprehensive RAG chatbot behavioral guidelines including: (1) Chatbot Identity & Core Purpose, (2) Knowledge Source Rules (Book-Based, User-Selected Text, General Questions modes), (3) Behavioral Guidelines (Accuracy, Style, Anti-Hallucination measures), (4) System Architecture clarifications (Hugging Face embeddings + Gemini via HF, NOT OpenAI), (5) Answer Logic Flow (3-step process), (6) Safety & Politeness standards. This materially expands the existing principle with operational governance for chatbot behavior without breaking backward compatibility.
