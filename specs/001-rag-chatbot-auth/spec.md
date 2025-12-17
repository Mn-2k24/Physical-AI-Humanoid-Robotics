# Feature Specification: Integrated RAG Chatbot & Authentication System

**Feature Branch**: `001-rag-chatbot-auth`
**Created**: 2025-12-13
**Status**: Draft
**Input**: User description: "Integrated RAG Chatbot & Authentication System"

## Clarifications

### Session 2025-12-13

- Q: Should authentication be required to access the book and chatbot? → A: Yes, authentication is mandatory - users cannot read the book or ask questions without signing in initially
- Q: Should conversation history or multi-turn context be implemented in the chatbot? → A: Yes, implementing conversation history or multi-turn context in the chatbot is now in scope
- Q: Should mobile responsive UI be built? → A: Yes, building mobile responsive UI is now in scope
- Q: Should recommendation systems for suggesting next chapters be built? → A: Yes, building recommendation systems for suggesting next chapters is now in scope

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Create Account and Sign In (Priority: P1)

As a new reader, I want to create an account by providing my background information (programming languages, frameworks, hardware availability) so that I can access the book and chatbot features.

**Why this priority**: Authentication is now mandatory for all book access and chatbot features. This is the entry point to the system and must be implemented first before any other functionality can be used.

**Independent Test**: Can be tested by attempting to access the book without authentication (should be blocked), then completing signup, signing in, and verifying access is granted.

**Acceptance Scenarios**:

1. **Given** a new reader visits the book, **When** they try to access any chapter or chatbot, **Then** they are redirected to the signin page
2. **Given** an unauthenticated visitor clicks "Signup", **When** they complete the signup form with valid information (including software/hardware background), **Then** their account is created and they are automatically signed in with full access
3. **Given** an existing user visits the book, **When** they click "Signin" and provide valid credentials, **Then** they are authenticated and can access all book chapters and chatbot features
4. **Given** a user is signed in, **When** they refresh the page or navigate between chapters, **Then** their session persists and they remain authenticated
5. **Given** a user is signed in, **When** they view the sidebar, **Then** their name appears with logout option

---

### User Story 2 - Ask Questions About Book Content with Conversation History (Priority: P2)

As an authenticated reader of the Physical AI & Humanoid Robotics book, I want to ask questions about any topic covered in the book and receive accurate, grounded answers with conversation context preserved so that I can have meaningful multi-turn discussions about the content.

**Why this priority**: This is the core value proposition of the RAG chatbot - enabling interactive learning with conversation memory. Requires authentication to be implemented first.

**Independent Test**: Can be fully tested by asking a series of related questions and verifying the chatbot maintains context across multiple turns, with all answers sourced from the correct chapters.

**Acceptance Scenarios**:

1. **Given** an authenticated reader is viewing the book, **When** they ask "What is Physical AI?", **Then** the chatbot returns an answer sourced only from book chapters with citations to specific sections
2. **Given** a reader has asked about Physical AI, **When** they follow up with "What are its main applications?", **Then** the chatbot understands the context and answers about Physical AI applications
3. **Given** a reader asks about a topic not covered in the book, **When** the query is submitted, **Then** the chatbot responds with "This information is not available in the book."
4. **Given** multiple relevant sections exist for a query, **When** the chatbot generates an answer, **Then** all source references are provided with links back to the original book sections
5. **Given** a user starts a new conversation, **When** they return to the chatbot, **Then** their previous conversation history is preserved and accessible

---

### User Story 3 - Ask Questions About Selected Text (Priority: P3)

As an authenticated reader studying a specific section of the book, I want to select text and ask questions about only that selection so that I can get focused, context-specific answers without interference from other chapters.

**Why this priority**: Enables deep, focused learning on specific topics. Requires global query mode and authentication to exist first.

**Independent Test**: Can be tested by selecting a paragraph about ROS 2, asking "What are the key benefits mentioned here?", and verifying the answer only uses the selected text as context.

**Acceptance Scenarios**:

1. **Given** an authenticated reader selects a paragraph about NVIDIA Isaac Sim, **When** they ask "What simulation capabilities are mentioned?", **Then** the answer only references information from the selected text
2. **Given** a reader selects text from Chapter 3, **When** they ask a question, **Then** no information from other chapters is included in the answer
3. **Given** a reader selects multiple paragraphs spanning two sections, **When** they submit a query, **Then** the answer context is strictly limited to those selected paragraphs
4. **Given** a reader asks a question without selecting text, **When** the query is submitted, **Then** the system defaults to global book search mode with conversation history

---

### User Story 4 - View Authentication State in Header and Sidebar (Priority: P4)

As a user, I want to see consistent authentication status in both the header and sidebar so that I always know whether I'm logged in and can access authentication actions easily.

**Why this priority**: Provides clear UX feedback and navigation consistency. This is a UI refinement that depends on Story 1 being implemented first.

**Independent Test**: Can be tested by signing in and verifying both header and sidebar update simultaneously to show user name and hide signin/signup buttons.

**Acceptance Scenarios**:

1. **Given** a user is not signed in, **When** they view the landing page, **Then** only signin/signup options are visible
2. **Given** a user signs in, **When** authentication completes, **Then** header displays user name and sidebar shows user name at the bottom with logout option
3. **Given** a signed-in user logs out, **When** logout completes, **Then** they are redirected to signin page and all book/chatbot access is revoked
4. **Given** a user is signed in on desktop, **When** they switch to mobile view, **Then** authentication state remains consistent in the mobile header and sidebar with responsive UI

---

### User Story 5 - Receive Personalized Chapter Recommendations (Priority: P5)

As an authenticated reader with recorded software and hardware background, I want to receive personalized recommendations for next chapters based on my profile and reading progress so that I can navigate the most relevant content efficiently.

**Why this priority**: Enhances personalized learning experience using the background data collected during signup. Requires authentication, user profiles, and reading progress tracking.

**Independent Test**: Can be tested by completing signup with specific background (e.g., Python + NVIDIA GPU), reading Chapter 1, and verifying recommendations prioritize GPU-accelerated content and Python examples.

**Acceptance Scenarios**:

1. **Given** a user has Python and NVIDIA GPU in their profile, **When** they finish reading the ROS 2 chapter, **Then** the system recommends the NVIDIA Isaac Sim chapter next
2. **Given** a beginner user completes the introduction, **When** they view recommendations, **Then** foundational chapters are suggested before advanced topics
3. **Given** a user with no robotics hardware, **When** recommendations are generated, **Then** simulation-focused chapters are prioritized over hardware deployment chapters
4. **Given** a user is reading Chapter 3, **When** they finish, **Then** the recommendation sidebar shows 3-5 relevant next chapters with personalized ranking

---

### Edge Cases

- What happens when a user asks a question in a language other than English?
  - System should respond in English (book content language) with a message indicating the book is in English only
- What happens when Qdrant vector database is temporarily unavailable?
  - System should display: "The chatbot is temporarily unavailable. Please try again in a few moments."
- What happens when a user selects text and then navigates to a different chapter before asking a question?
  - System should clear the text selection and default to global search mode with conversation history preserved
- What happens when a user's session expires while they are reading?
  - User should be immediately redirected to signin page and must re-authenticate to continue
- What happens when a user provides invalid email during signup?
  - Form should display real-time validation error: "Please enter a valid email address"
- What happens when a user tries to create an account with an email that already exists?
  - System should display: "An account with this email already exists. Please sign in instead."
- What happens when no chunks match the user's query (similarity score below threshold)?
  - System should respond: "This information is not available in the book."
- What happens when selected text is too short (less than 50 characters)?
  - System should proceed with the query but inform user: "Selected text is very brief - consider selecting more context for better answers."
- What happens when an unauthenticated user tries to access a direct chapter URL?
  - System should redirect to signin page and preserve the intended destination to redirect after successful authentication
- What happens when conversation history exceeds storage limits?
  - System should archive older conversations (>30 days) and display a message: "Viewing archived conversation. Recent context may be limited."

## Requirements *(mandatory)*

### Functional Requirements

**Authentication & Access Control Requirements:**

- **FR-001**: System MUST require authentication for all book and chatbot access - no anonymous access allowed
- **FR-002**: System MUST redirect unauthenticated users to signin page when accessing any book chapter or chatbot feature
- **FR-003**: System MUST preserve intended destination URL and redirect authenticated users to it after successful signin
- **FR-004**: System MUST implement authentication using Better Auth (https://www.better-auth.com) - no custom authentication logic allowed
- **FR-005**: System MUST provide a signup form collecting: name, email, password (min 8 characters)
- **FR-006**: System MUST collect software background during signup: programming languages (multi-select), frameworks (multi-select), experience level (radio)
- **FR-007**: System MUST collect hardware background during signup: available hardware (multi-select), robotics hardware access (multi-select)
- **FR-008**: System MUST store all user account data and background questionnaires in Neon Serverless Postgres
- **FR-009**: System MUST provide a signin form accepting email and password
- **FR-010**: System MUST support "Remember me" functionality for persistent sessions
- **FR-011**: System MUST provide password reset capability
- **FR-012**: System MUST validate authentication state on all protected endpoints (all book and chatbot endpoints)
- **FR-013**: System MUST hash passwords using industry-standard algorithms (bcrypt or Argon2)
- **FR-014**: System MUST terminate access immediately when user session expires and redirect to signin page

**RAG Chatbot Requirements:**

- **FR-015**: System MUST ingest all Markdown files under `/docs/` directory and embed them using Gemini API embeddings
- **FR-016**: System MUST chunk book content into semantically meaningful sections preserving paragraph boundaries and section headings
- **FR-017**: System MUST store embeddings in Qdrant Cloud with metadata: file_path, section_heading, chunk_index, raw_text
- **FR-018**: System MUST support global book queries that search across all embedded chapters
- **FR-019**: System MUST support local queries that only use user-selected text as context
- **FR-020**: System MUST generate answers using only retrieved content from the book (zero hallucinations, zero external knowledge)
- **FR-021**: System MUST respond with "This information is not available in the book." when similarity scores are below threshold
- **FR-022**: System MUST return source references with every answer, including file paths and section headings
- **FR-023**: System MUST provide metadata for highlighting source sections in the book UI
- **FR-024**: System MUST enforce strict context isolation for selected-text queries (no cross-chapter retrieval allowed)

**Conversation History & Multi-Turn Context Requirements:**

- **FR-025**: System MUST store conversation history for each authenticated user in Neon Postgres
- **FR-026**: System MUST maintain conversation context across multiple turns within a session
- **FR-027**: System MUST allow users to view their previous conversations
- **FR-028**: System MUST include conversation history in the chatbot context when generating answers for follow-up questions
- **FR-029**: System MUST support starting new conversations (clearing context) while preserving history
- **FR-030**: System MUST archive conversations older than 30 days to optimize performance
- **FR-031**: System MUST allow users to delete their conversation history

**UI Integration Requirements:**

- **FR-032**: Header MUST show only "Signin" and "Signup" buttons when user is not authenticated
- **FR-033**: Header MUST display user name and logout button when user is authenticated
- **FR-034**: Sidebar MUST show signin/signup options when user is not authenticated (on landing page only)
- **FR-035**: Sidebar MUST show user name and logout option when user is authenticated
- **FR-036**: Header and sidebar authentication state MUST remain synchronized at all times
- **FR-037**: System MUST update UI state immediately upon login/logout without requiring page refresh
- **FR-038**: Signup form MUST include multi-step wizard for background questionnaires
- **FR-039**: All authentication forms MUST be responsive (mobile, tablet, desktop)
- **FR-040**: Forms MUST provide real-time validation feedback
- **FR-041**: Signup form MUST include password strength indicator
- **FR-042**: All UI components MUST be fully responsive and mobile-optimized

**Recommendation System Requirements:**

- **FR-043**: System MUST generate personalized chapter recommendations based on user's software background
- **FR-044**: System MUST generate personalized chapter recommendations based on user's hardware availability
- **FR-045**: System MUST track user's reading progress (chapters read, time spent per chapter)
- **FR-046**: System MUST recommend 3-5 next chapters after user finishes reading a chapter
- **FR-047**: System MUST prioritize recommendations matching user's experience level (beginner/intermediate/advanced/expert)
- **FR-048**: System MUST display recommendations in a dedicated sidebar section
- **FR-049**: System MUST allow users to dismiss or mark recommendations as "not interested"
- **FR-050**: System MUST update recommendations dynamically as user progresses through the book

**Backend API Requirements:**

- **FR-051**: Backend MUST provide `/chat/global` endpoint for full-book RAG queries (authentication required)
- **FR-052**: Backend MUST provide `/chat/local` endpoint for selected-text-only RAG queries (authentication required)
- **FR-053**: Backend MUST provide `/chat/history` endpoint to retrieve user's conversation history (authentication required)
- **FR-054**: Backend MUST provide `/chat/new` endpoint to start a new conversation (authentication required)
- **FR-055**: Backend MUST provide `/auth/signup` endpoint for Better Auth signup
- **FR-056**: Backend MUST provide `/auth/signin` endpoint for Better Auth signin
- **FR-057**: Backend MUST provide `/auth/signout` endpoint for Better Auth signout
- **FR-058**: Backend MUST provide `/auth/me` endpoint to retrieve current logged-in user
- **FR-059**: Backend MUST provide `/recommendations` endpoint to get personalized chapter recommendations (authentication required)
- **FR-060**: Backend MUST provide `/progress` endpoint to track and retrieve reading progress (authentication required)
- **FR-061**: Backend MUST provide `/ingest` endpoint (internal use) to load `/docs` into Qdrant
- **FR-062**: All API responses MUST include appropriate CORS headers for Docusaurus frontend
- **FR-063**: Protected endpoints MUST return 401 Unauthorized when authentication is invalid

**Security Requirements:**

- **FR-064**: All API keys and secrets MUST be stored in environment variables (server-side only)
- **FR-065**: No API keys MUST be exposed to the frontend
- **FR-066**: Authentication endpoints MUST use HTTPS only
- **FR-067**: Forms MUST include CSRF protection
- **FR-068**: Signup and signin endpoints MUST be rate-limited (max 5 attempts per minute per IP)
- **FR-069**: No plaintext passwords MUST appear in logs or database
- **FR-070**: Session tokens MUST have expiration and be securely generated
- **FR-071**: System MUST implement proper session expiration handling with automatic signout

**Data Persistence Requirements:**

- **FR-072**: Qdrant MUST store embeddings, chunk text, and chunk metadata only (no user data)
- **FR-073**: Neon Postgres MUST store user accounts, background questionnaires, chat interaction logs, conversation history, reading progress, and recommendation data
- **FR-074**: Embeddings MUST NOT be stored in Postgres
- **FR-075**: User background data MUST be used for generating personalized recommendations
- **FR-076**: System MUST log all authentication events to Neon Postgres for audit purposes
- **FR-077**: Conversation history MUST be associated with user_id for retrieval and privacy compliance
- **FR-078**: Reading progress MUST track chapter_id, user_id, completion_percentage, time_spent, last_accessed

### Key Entities

- **User**: Represents a reader with an account. Attributes: id, email, hashed_password, full_name, created_at, updated_at
- **UserProfile**: Stores user background information. Attributes: user_id, experience_level (Beginner/Intermediate/Advanced/Expert), created_at, updated_at
- **UserSoftwareBackground**: Stores software skills. Attributes: user_id, programming_languages (array), frameworks (array)
- **UserHardwareBackground**: Stores hardware availability. Attributes: user_id, available_hardware (array), robotics_hardware (array)
- **AuthSession**: Manages authentication sessions. Attributes: session_id, user_id, token, expires_at, created_at
- **Conversation**: Represents a conversation thread. Attributes: id, user_id, title, created_at, updated_at, archived (boolean)
- **ChatInteraction**: Logs individual chatbot queries within a conversation. Attributes: id, conversation_id, user_id, query_text, answer_text, query_mode (global/local), source_chunks (array), created_at
- **BookChunk**: Represents an embedded section of the book. Attributes: chunk_id, file_path, section_heading, chunk_index, raw_text, embedding_vector (stored in Qdrant)
- **ReadingProgress**: Tracks user's progress through chapters. Attributes: id, user_id, chapter_id, completion_percentage, time_spent_seconds, last_accessed, completed (boolean)
- **Recommendation**: Stores generated recommendations. Attributes: id, user_id, recommended_chapter_id, score, reason, dismissed (boolean), created_at

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Authentication & Access Control:**

- **SC-001**: 100% of unauthenticated access attempts to book/chatbot are blocked and redirected to signin
- **SC-002**: Users can complete the signup process (including background questionnaire) in under 3 minutes
- **SC-003**: Users can sign in successfully within 10 seconds
- **SC-004**: User sessions persist across page refreshes and navigation for the duration of the session
- **SC-005**: Session expiration triggers immediate signout and redirect within 1 second

**Chatbot Functionality:**

- **SC-006**: Users can receive accurate answers to questions about book content in under 3 seconds
- **SC-007**: 95% of queries about topics covered in the book return relevant answers (not "information not available" messages)
- **SC-008**: Selected-text queries return answers that reference only the selected content in 100% of test cases
- **SC-009**: All chatbot answers include source references with links to specific book sections
- **SC-010**: System maintains zero hallucinations - all answers are verifiable against book content

**Conversation History:**

- **SC-011**: Conversation history is preserved across sessions with 100% accuracy
- **SC-012**: Multi-turn conversations maintain context for up to 10 consecutive turns
- **SC-013**: Users can retrieve their previous conversations within 2 seconds
- **SC-014**: Conversation archival (>30 days) reduces active storage by 70%

**Mobile Responsive UI:**

- **SC-015**: All UI elements (header, sidebar, forms, chatbot) render correctly on mobile devices (320px-480px width)
- **SC-016**: All UI elements render correctly on tablet devices (481px-1024px width)
- **SC-017**: All UI elements render correctly on desktop devices (>1024px width)
- **SC-018**: Touch interactions (tap, swipe, pinch) work correctly on mobile/tablet with 95% success rate

**Recommendation System:**

- **SC-019**: Personalized recommendations are generated within 1 second of chapter completion
- **SC-020**: Recommendations match user's software background in 80% of cases
- **SC-021**: Recommendations match user's hardware availability in 80% of cases
- **SC-022**: Beginner users receive foundational chapter recommendations before advanced topics in 100% of cases
- **SC-023**: Users can dismiss recommendations successfully 100% of the time

**User Experience:**

- **SC-024**: 90% of users successfully complete their first chatbot query on the first attempt
- **SC-025**: Authentication forms display validation errors within 500ms of input
- **SC-026**: Header and sidebar authentication state updates occur within 1 second of login/logout
- **SC-027**: 100% of signup submissions capture software and hardware background data successfully

**System Performance:**

- **SC-028**: Backend handles 500 requests per day on free-tier infrastructure without degradation
- **SC-029**: System ingests all 34 book chapters (15,000+ lines) and generates 1,000+ embeddings successfully
- **SC-030**: Vector similarity search returns top-k results in under 1 second
- **SC-031**: Conversation history retrieval supports up to 100 conversations per user without performance degradation

**Security and Privacy:**

- **SC-032**: Zero API keys or secrets are exposed in frontend code or network requests
- **SC-033**: Rate limiting successfully blocks more than 5 authentication attempts per minute from the same IP
- **SC-034**: All passwords are hashed using bcrypt or Argon2 (verifiable via database inspection)
- **SC-035**: User background data is only used for recommendations and never included in chatbot response text

## Assumptions

- The existing chatbot UI in `/src/components` can be extended to support conversation history and mobile responsiveness
- Book content in `/docs/` is in English and properly formatted Markdown
- Gemini API free tier provides sufficient quota for embedding 1,000+ chunks and answering 500+ queries per day with conversation context
- Qdrant Cloud free tier supports the vector collection size needed (~1,000 chunks with 1536-dimension embeddings)
- Neon Serverless Postgres free tier supports user accounts, conversation history, reading progress, and recommendation data
- Docusaurus supports the necessary React hooks and state management for authentication UI integration and mobile responsiveness
- Better Auth supports FastAPI backend integration (documentation available at https://www.better-auth.com)
- Users have modern browsers with JavaScript enabled (Chrome, Firefox, Safari, Edge) on desktop and mobile
- The book content does not change frequently (daily re-ingestion not required)
- Recommendation algorithm can leverage user background data effectively for personalization

## Out of Scope

- Redesigning or reimplementing the existing chatbot UI in `/src/components` (only extending with new features)
- Supporting languages other than English for book content or chatbot responses
- Building an analytics dashboard for viewing chat interactions
- Implementing user profile editing UI (beyond background questionnaire during signup)
- Creating admin interfaces for managing users or content
- Supporting OAuth/SSO authentication (Better Auth email/password only)
- Implementing payment or subscription features
- Building mobile native apps (responsive web only, no iOS/Android apps)
- Supporting offline mode or PWA features
- Implementing real-time collaborative features
- Creating automated content moderation for user queries
- Implementing gamification or progress tracking beyond basic reading progress
- Advanced recommendation algorithms using machine learning (rule-based recommendations only)
- Conversation export functionality
- Multi-language support for UI or content
