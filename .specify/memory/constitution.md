# Physical AI & Humanoid Robotics Book Constitution

<!--
SYNC IMPACT REPORT
==================
Version Change: 3.0.0 → 4.0.0
Modified Principles:
  - Principle VII: RAG infrastructure changed from OpenAI to Gemini API (BREAKING CHANGE)
  - Principle VII: Backend endpoints changed (BREAKING CHANGE)
  - Principle VII: Removed text-selection UI requirement (focus on backend only)
  - Principle VIII: Expanded with detailed Better Auth integration requirements
Added Sections:
  - Authentication frontend UI requirements (Signup/Signin components)
  - User background questionnaire schema details
  - Neon Postgres data persistence requirements
  - Security and privacy requirements
  - Non-negotiable constraints section
Removed Sections:
  - OpenAI API dependencies (replaced with Gemini)
  - Chatbot frontend implementation requirements (explicitly excluded)
Templates Requiring Updates:
  ✅ plan-template.md - Constitution Check will validate Gemini usage and Better Auth integration
  ✅ spec-template.md - User scenarios support authenticated interactions
  ✅ tasks-template.md - Task structure accommodates auth + RAG backend tasks
Follow-up TODOs:
  - Migrate existing RAG embeddings from OpenAI to Gemini
  - Implement Better Auth FastAPI integration
  - Create Neon Postgres schema for user accounts and background data
  - Build Signup/Signin UI components
  - Update environment variables for Gemini API keys
  - Remove OpenAI dependencies from backend
  - Test authentication flow end-to-end
  - Validate RAG pipeline with Gemini embeddings
==================
-->

## Project Overview

**Book Title**: Physical AI & Humanoid Robotics
**Platform**: Docusaurus
**Management**: Spec-Kit Plus
**Authoring Tools**: Claude CLI / Claude Code
**Publishing**: Vercel (primary), GitHub Pages
**Interactive Features**: RAG-powered AI chatbot with user authentication for personalized learning assistance

## Core Purpose

To produce a publicly accessible, academically grounded, and technically accurate interactive book that explains how AI systems can control humanoid robots in both simulated and real-world physical environments, enhanced with an intelligent chatbot for personalized learning assistance secured by user authentication.

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
- Personalized learning experiences through user authentication

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

**Rule**: The book MUST provide an intelligent, retrieval-augmented chatbot for content exploration with strict grounding in book content.

**Core Requirements**:
- **Grounded Responses Only**: All chatbot answers MUST be sourced strictly from book content - zero hallucinations, zero external knowledge usage
- **Dual Query Modes**:
  - Global mode: Full-book queries across all chapters
  - Local mode: Selected-text queries (users select text, chatbot answers ONLY from that selection)
- **Strict Context Isolation**: Selected-text mode MUST ignore all unrelated content (absolute rule)
- **Backend-Only Scope**: Frontend chatbot UI implementation is explicitly excluded

**Technical Standards**:
- **Backend Framework**: FastAPI
- **LLM Provider**: Gemini API (NOT OpenAI, NOT Hugging Face)
  - Embeddings: Gemini API embeddings
  - Answer Generation: Gemini API LLM
- **Vector Database**: Qdrant Cloud (Free Tier)
  - Collection name: `physical_ai_book`
  - Minimum 1,000 semantically meaningful text chunks
  - Chunks MUST preserve paragraph boundaries and section headings
  - Metadata required: `file_path`, `section_heading`, `chunk_index`
- **Persistence**: Neon Serverless Postgres
  - Chat interaction metadata (query, timestamps, sources)
  - User authentication data
  - Conversation analytics
- **API Endpoints**: Maximum 3 endpoints
  - `/ask` - Full-book RAG queries
  - `/ask-local` - Selected-text only queries
  - `/track` - Conversation analytics logging
- **Performance**: Query latency < 3 seconds
- **Security**: All API keys server-side only (zero client exposure)

**Retrieval Pipeline**:
1. User query → Gemini API embedding
2. Qdrant vector similarity search (top-k)
3. Context filtering (global or selected-text only)
4. Answer generation using Gemini API with retrieved chunks
5. Similarity score threshold validation

**Answer Generation Rules**:
- Retrieved chunks are the **only source of truth**
- Concise, factual answers only
- Implicit grounding to source text
- Never fabricate missing information
- If similarity below threshold: "No relevant information found in this book."

**Data Management**:
- Embeddings stored in Qdrant only (NOT in Postgres)
- Conversation logs and metadata stored in Neon Postgres
- All responses MUST be traceable to source book sections
- Strict schema enforcement in Neon Postgres

**Success Validation**:
- Backend endpoints deployed and accessible
- Responses accurate and verifiable against book content
- Selected-text mode strictly isolates context
- Zero API key leakage in any layer
- Analytics visible in Neon dashboard

**Rationale**: Modern educational resources require interactive assistance. A RAG-powered chatbot enhances learning by providing instant, contextually relevant answers while ensuring accuracy through retrieval from verified book content. Gemini API provides cost-effective embeddings and generation without external dependencies. Strict grounding prevents misinformation and maintains educational integrity.

### VIII. User Authentication & Personalization

**Rule**: The system MUST provide secure user authentication using Better Auth to enable personalized learning experiences and future content customization.

**Core Requirements**:
- **Authentication Provider**: ALL authentication MUST be implemented using Better Auth (https://www.better-auth.com) - custom authentication logic is prohibited
- **Mandatory Frontend UI**: Full Signup and Signin UI components MUST be provided and integrated into Docusaurus
- **User Background Collection**: During signup, the system MUST collect:
  - **Software Background**: Programming languages, frameworks, experience level
  - **Hardware Background**: CPU/GPU availability, robotics/AI hardware access
- **Session Management**: Secure session-based or token-based authentication
- **Backend Validation**: All authenticated endpoints MUST validate auth state server-side

**Signup Requirements**:
- Full name, email, password (minimum 8 characters)
- Software background questionnaire:
  - Known programming languages (multi-select: Python, C++, JavaScript, Rust, Go, Java, Other)
  - Frameworks experience (multi-select: ROS, ROS2, PyTorch, TensorFlow, Unity, Unreal, Other)
  - Experience level (radio: Beginner, Intermediate, Advanced, Expert)
- Hardware background questionnaire:
  - Available hardware (multi-select: CPU only, NVIDIA GPU, AMD GPU, Apple Silicon, Jetson, Other)
  - Robotics hardware access (multi-select: None, Simulation only, Educational robot kit, Research platform, Other)
- All data stored in Neon Serverless Postgres

**Signin Requirements**:
- Email and password authentication
- "Remember me" functionality
- Password reset capability
- Secure token generation and validation

**Frontend UI Requirements (Mandatory)**:
- **Signup Page**:
  - Clean, accessible form with validation
  - Multi-step wizard for background questionnaires
  - Real-time validation feedback
  - Password strength indicator
  - Terms of service acceptance
- **Signin Page**:
  - Email/password form
  - "Remember me" checkbox
  - "Forgot password" link
  - Clear error messaging
- **Integration**: Auth UI MUST be integrated into Docusaurus navigation
- **Responsive Design**: All forms MUST work on mobile, tablet, desktop
- **Explicitly Excluded**: Chatbot UI, message rendering, chat streaming interface

**Backend Architecture**:
- **Better Auth Integration**: Use Better Auth SDK for FastAPI backend
- **Database Schema** (Neon Postgres):
  - `users` (id, email, hashed_password, full_name, created_at, updated_at)
  - `user_profiles` (user_id, experience_level, created_at, updated_at)
  - `user_software_background` (user_id, programming_languages, frameworks)
  - `user_hardware_background` (user_id, available_hardware, robotics_hardware)
  - `auth_sessions` (session_id, user_id, token, expires_at, created_at)
- **Protected Endpoints**: Authentication required for `/ask`, `/ask-local`, `/track`

**Security Requirements**:
- Password hashing using industry-standard algorithms (bcrypt, Argon2)
- HTTPS-only for all authentication endpoints
- CSRF protection on all forms
- Rate limiting on signup/signin (max 5 attempts per minute)
- Secure session tokens with expiration
- No plaintext passwords in logs or database
- Environment variables for all secrets

**Privacy Requirements**:
- User background data MUST NOT be exposed to chatbot responses directly
- User data MUST NOT be used for training or shared with third parties
- Clear privacy policy during signup
- GDPR-compliant data handling (right to deletion, data export)
- Audit logs for authentication events in Neon Postgres

**Future Personalization (Post-MVP)**:
- Tailor responses based on user's programming language preference
- Adjust code examples to match framework experience
- Recommend hardware-appropriate simulation environments
- Track learning progress and suggest next chapters

**Non-Negotiable Constraints**:
- Better Auth MUST be used (no custom auth)
- Signup MUST collect software and hardware background
- Frontend UI for auth MUST be provided
- All user data MUST be in Neon Postgres
- No authentication secrets exposed to frontend
- No unprotected personal data in API responses

**Success Validation**:
- Users can sign up and sign in successfully
- Background questionnaire data captured and stored
- Protected endpoints require valid authentication
- Session management works across browser sessions
- All forms accessible and responsive
- Zero security vulnerabilities in authentication flow

**Rationale**: Personalized learning experiences significantly improve educational outcomes. By collecting user background during signup, the system can future-proof for adaptive content delivery while maintaining strict security and privacy standards. Better Auth provides production-ready authentication without reinventing security-critical infrastructure, allowing focus on educational features. The mandatory frontend UI ensures users can immediately interact with authentication.

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
  - Gemini API for embeddings and chat completion (NOT OpenAI)
  - Qdrant Cloud for vector storage and similarity search
  - Neon Postgres for analytics, conversation logs, and user data
- **Authentication**:
  - Better Auth for user authentication
  - Neon Postgres for user accounts and background data

### Development Rules

1. Every major section MUST be generated through a Spec-Kit spec file
2. Every commit message MUST reflect chapter/section editing
3. Claude-generated sections require manual technical review before merging
4. Commit messages MUST indicate AI vs. human authorship
5. All code examples MUST be tested before committing
6. RAG chatbot changes MUST be tested for accuracy and security before deployment
7. Authentication changes MUST pass security review before deployment

### Workflow Steps

1. **Specification**: Use `/sp.specify` to define chapter structure
2. **Planning**: Use `/sp.plan` for content architecture
3. **Task Breakdown**: Use `/sp.tasks` for implementation steps
4. **Content Generation**: Write content (AI-assisted or manual)
5. **Technical Review**: Human validation of technical accuracy
6. **Citation Check**: Verify all claims are properly cited
7. **Testing**: Run all code examples and simulations
8. **RAG Integration**: Ensure new content is embedded and retrievable via chatbot
9. **Authentication Testing**: Verify auth flows work end-to-end
10. **Merge**: Commit to repository with appropriate messages

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
  - LLM & Embeddings: Gemini API only (NOT OpenAI, NOT Hugging Face)
  - Vector collection: `physical_ai_book` in Qdrant Cloud (Free Tier)
  - Minimum 1,000 semantically meaningful text chunks
  - Chunks preserve paragraph boundaries and section headings
  - Metadata: `file_path`, `section_heading`, `chunk_index`
  - Query latency: < 3 seconds
  - Backend endpoints: Maximum 3 (`/ask`, `/ask-local`, `/track`)
  - Environment variables: Server-side only (no client-side exposure)
- **Authentication**:
  - Provider: Better Auth only (custom auth prohibited)
  - Database: Neon Serverless Postgres
  - Frontend UI: Mandatory for Signup/Signin
  - Security: HTTPS, rate limiting, password hashing, CSRF protection

### Quality Constraints

- **Readability**: Flesch-Kincaid grade 11–14
- **Citation Style**: IEEE or ACM (consistent throughout)
- **Plagiarism**: 0% tolerance
- **Technical Accuracy**: All claims must be verifiable
- **Chatbot Accuracy**: All responses grounded in book content only (no hallucinations)
- **Context Isolation**: Selected-text mode strictly excludes unrelated content
- **Security**: Zero API key leakage, zero authentication vulnerabilities

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
10. **Interactive Chatbot**: RAG-powered chatbot fully functional backend
11. **Chatbot Accuracy**: All responses traceable to book sections, zero hallucinations
12. **Dual Query Modes**: Both full-book and selected-text queries working
13. **Context Isolation**: Selected-text mode strictly isolated from other content
14. **Analytics Dashboard**: Conversation logs visible in Neon Postgres dashboard
15. **Security Compliance**: No API keys exposed, no auth vulnerabilities
16. **Performance Target**: Chatbot responses delivered in < 3 seconds
17. **Authentication Works**: Users can sign up, sign in, and access protected endpoints
18. **User Background Collected**: Signup captures software and hardware background data
19. **Free-Tier Stability**: Backend stable under expected load on free tiers

## Non-Negotiable Constraints

The following constraints are absolute and MUST NOT be violated:

1. **No OpenAI APIs**: Use Gemini API only for embeddings and LLM
2. **No Hugging Face APIs**: Use Gemini API only
3. **No Chatbot Frontend Implementation**: Backend endpoints only
4. **No Ungrounded Answers**: All chatbot responses MUST be from book content
5. **No External Knowledge Usage**: Chatbot MUST NOT use general world knowledge
6. **No Custom Authentication**: Better Auth MUST be used
7. **No Skipped Signup Background**: MUST collect software and hardware background
8. **No Frontend Exclusion for Auth**: Signup/Signin UI MUST be provided
9. **No Embeddings in Postgres**: Embeddings stored in Qdrant only
10. **No Secrets in Frontend**: All API keys and secrets server-side only

Violation of any constraint invalidates the implementation.

## Versioning Rules

### Book Versions

- **v1.0 (MVP)**: All major chapters drafted and navigable online
- **v1.1**: Citations, case studies, and diagrams added
- **v1.2**: Simulation code and reproducible experiments added
- **v2.0**: Real-world hardware integration documented
- **v3.0**: RAG chatbot integrated with OpenAI (deprecated)
- **v4.0**: RAG chatbot migrated to Gemini API + Better Auth integration

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
- [ ] Content embedded in RAG vector database (v4.0+)
- [ ] Chatbot can accurately answer questions about the chapter (v4.0+)
- [ ] Authentication protects chatbot endpoints (v4.0+)

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
4. Increment constitution version appropriately (MAJOR, MINOR, or PATCH)
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

**Version**: 4.0.0
**Ratified**: 2025-12-04
**Last Amended**: 2025-12-13

### Version History

- **1.0.0** (2025-12-04): Initial constitution established with complete governance framework for Physical AI & Humanoid Robotics book project
- **2.0.0** (2025-12-09): MAJOR - Added Principle VII (RAG-Powered Interactive Learning) with OpenAI-based chatbot requirements, infrastructure specifications, and security standards
- **3.0.0** (2025-12-09): MAJOR - Added Principle VIII (User Authentication & Personalization) with Better Auth requirements, expanding interactive platform capabilities
- **4.0.0** (2025-12-13): MAJOR - Backward-incompatible infrastructure change:
  - RAG backend migrated from OpenAI to Gemini API (embeddings + LLM)
  - Authentication made mandatory with Better Auth integration
  - Frontend chatbot UI explicitly excluded (backend endpoints only)
  - Added strict grounding requirements (no hallucinations, no external knowledge)
  - Added selected-text mode with absolute context isolation
  - Updated non-negotiable constraints section
  - Expanded Principle VIII with detailed user background collection and Neon Postgres schema
  - This version fundamentally changes the technical stack and requires full migration
