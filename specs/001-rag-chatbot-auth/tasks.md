# Tasks: Integrated RAG Chatbot & Authentication System

**Input**: Design documents from `/specs/001-rag-chatbot-auth/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/openapi.yaml

**Tests**: Tests are NOT included in this task list as they were not explicitly requested in the feature specification

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

- **Web app structure**: `backend/src/`, `frontend/src/` (per plan.md)
- Backend: FastAPI (Python 3.11)
- Frontend: React/Docusaurus with Better Auth integration

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend directory structure (backend/src/, backend/scripts/, backend/tests/)
- [X] T002 [P] Create frontend directory structure (src/components/auth/, src/pages/, src/hooks/)
- [X] T003 [P] Initialize Python project with requirements.txt (FastAPI, google-generativeai, qdrant-client, psycopg2-binary, pydantic, uvicorn, python-jose, passlib, slowapi)
- [ ] T004 [P] Initialize frontend dependencies (Better Auth React SDK, TailwindCSS, React Query)
- [X] T005 [P] Create .env.example with required environment variables (GEMINI_API_KEY, QDRANT_API_KEY, QDRANT_ENDPOINT, NEON_CONNECTION_STRING, BETTER_AUTH_SECRET)
- [X] T006 [P] Configure linting and formatting (black, isort for Python; prettier, eslint for TypeScript)
- [X] T007 [P] Create .gitignore for .env, __pycache__, node_modules, .venv

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database Infrastructure

- [X] T008 Create database migration script backend/scripts/migrations/001_create_users.sql (users table with id, email, hashed_password, full_name, created_at, updated_at)
- [X] T009 [P] Create migration script backend/scripts/migrations/002_create_profiles.sql (user_profiles, user_software_background, user_hardware_background tables)
- [X] T010 [P] Create migration script backend/scripts/migrations/003_create_auth_sessions.sql (auth_sessions table)
- [X] T011 [P] Create migration script backend/scripts/migrations/004_create_conversations.sql (conversations and chat_interactions tables)
- [X] T012 [P] Create migration script backend/scripts/migrations/005_create_progress.sql (reading_progress table)
- [X] T013 [P] Create migration script backend/scripts/migrations/006_create_recommendations.sql (recommendations table)
- [X] T014 [P] Create migration script backend/scripts/migrations/007_create_audit_logs.sql (audit_logs table)
- [X] T015 Create database initialization script backend/scripts/init_db.py (runs all migrations in order, creates indexes)

### Configuration & Core Services

- [X] T016 Create configuration management backend/src/core/config.py (load environment variables using pydantic-settings)
- [X] T017 [P] Create security utilities backend/src/core/security.py (password hashing with bcrypt, JWT token generation/validation, CSRF token utilities)
- [X] T018 [P] Create Neon Postgres connection pool backend/src/services/neon.py (async connection pool with psycopg_pool, min_size=2, max_size=10)
- [X] T019 [P] Create Qdrant client wrapper backend/src/services/qdrant.py (initialize QdrantClient with HNSW config, create collection with 768-dim vectors)
- [X] T020 Create Gemini embedding service backend/src/services/embedding.py (text-embedding-004 with task_type="retrieval_document", batch processing support)
- [X] T021 Create authentication middleware backend/src/core/middleware.py (JWT validation, extract user_id from token, dependency injection for get_current_user)
- [X] T022 Create rate limiting middleware backend/src/core/middleware.py (slowapi integration, 5 attempts/minute for auth endpoints)
- [X] T023 Create error handling utilities backend/src/utils/validation.py (Pydantic schemas for request/response validation)
- [X] T024 Create FastAPI app initialization backend/src/main.py (app setup, CORS middleware, error handlers, health check endpoint)

### Book Ingestion Pipeline

- [X] T025 Create text chunking utility backend/src/utils/chunking.py (semantic chunking, preserve paragraph boundaries, max 500 tokens per chunk)
- [X] T026 Create book ingestion script backend/scripts/ingest.py (read /docs markdown files, chunk content, generate embeddings, upload to Qdrant with metadata)

**Checkpoint**: ✅ Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Create Account and Sign In (Priority: P1) 🎯 MVP

**Goal**: Enable user signup with background questionnaire and signin with session persistence, blocking all unauthenticated access

**Independent Test**: Attempt to access any book chapter without authentication (should redirect to signin), complete signup with background info, signin, and verify access is granted with persistent session

### Backend Implementation (User Story 1)

- [ ] T027 [P] [US1] Create User model backend/src/models/user.py (User, UserProfile, UserSoftwareBackground, UserHardwareBackground Pydantic schemas)
- [ ] T028 [P] [US1] Create AuthSession model backend/src/models/auth.py (AuthSession schema with session_id, user_id, token_hash, expires_at)
- [ ] T029 [US1] Create authentication service backend/src/services/auth_service.py (signup, signin, signout, validate_session, refresh_token methods using Neon Postgres)
- [ ] T030 [US1] Implement /auth/signup endpoint backend/src/api/auth.py (POST, accepts email, password, full_name, experience_level, programming_languages, frameworks, available_hardware, robotics_hardware, validates input, hashes password, creates user + profiles, returns JWT in HTTP-only cookie)
- [ ] T031 [US1] Implement /auth/signin endpoint backend/src/api/auth.py (POST, accepts email and password, validates credentials, creates session, returns JWT in HTTP-only cookie with "remember me" support)
- [ ] T032 [US1] Implement /auth/signout endpoint backend/src/api/auth.py (POST, requires auth, deletes session, clears cookie)
- [ ] T033 [US1] Implement /auth/me endpoint backend/src/api/auth.py (GET, requires auth, returns current user profile with background data)
- [ ] T034 [US1] Implement /auth/refresh endpoint backend/src/api/auth.py (POST, refreshes JWT token, extends session expiration)
- [ ] T035 [US1] Add audit logging to authentication events backend/src/services/auth_service.py (log signup_success, signup_failed, signin_success, signin_failed, signout, session_expired to audit_logs table)
- [ ] T036 [US1] Add rate limiting to /auth/signup and /auth/signin (5 attempts/minute per IP using slowapi)

### Frontend Implementation (User Story 1)

- [X] T037 [P] [US1] Create Better Auth client frontend/src/lib/auth.ts (configure Better Auth with baseURL, credentials: "include")
- [X] T038 [P] [US1] Create auth context provider frontend/src/components/auth/AuthProvider.tsx (useSession hook, session state management)
- [X] T039 [US1] Create signup form component frontend/src/components/auth/SignupForm.tsx (multi-step wizard: Step 1: email/password/name, Step 2: experience level, Step 3: software background, Step 4: hardware background, real-time validation, password strength indicator, responsive mobile layout)
- [X] T040 [US1] Create signin form component frontend/src/components/auth/SigninForm.tsx (email/password inputs, "remember me" checkbox, show/hide password toggle, responsive mobile layout)
- [X] T041 [US1] Create signup page frontend/src/pages/signup.tsx (render SignupForm, redirect to book after successful signup)
- [X] T042 [US1] Create signin page frontend/src/pages/signin.tsx (render SigninForm, preserve intended destination URL, redirect after successful signin)
- [X] T043 [US1] Create authentication hook frontend/src/hooks/useAuth.ts (useSignup, useSignin, useSignout, useCurrentUser)
- [X] T044 [US1] Update header component frontend/src/components/layout/Header.tsx (show signin/signup buttons when unauthenticated, show user name and logout button when authenticated, mobile responsive)
- [X] T045 [US1] Update sidebar component frontend/src/components/layout/Sidebar.tsx (show signin/signup on landing page when unauthenticated, show user name and logout when authenticated, mobile responsive)
- [X] T046 [US1] Implement authentication guard (wrap all book chapter routes, redirect to signin if not authenticated, preserve destination URL)
- [X] T047 [US1] Add session expiration handling (detect 401 responses, auto-signout, redirect to signin with message)

**Checkpoint**: User Story 1 complete - users can create accounts, sign in, and access is properly gated with session persistence

---

## Phase 4: User Story 2 - Ask Questions About Book Content with Conversation History (Priority: P2)

**Goal**: Enable authenticated users to ask questions about book content with strict grounding, multi-turn conversation context, and conversation history persistence

**Independent Test**: Ask a series of related questions ("What is Physical AI?", then "What are its main applications?"), verify chatbot maintains context across turns, all answers are sourced from book chapters with citations, and conversation history is preserved across sessions

### Backend Implementation (User Story 2)

- [X] T048 [P] [US2] Create Conversation model backend/src/models/conversation.py (Conversation and ChatInteraction Pydantic schemas)
- [X] T049 [P] [US2] Create RAG service backend/src/services/rag.py (retrieve_chunks from Qdrant, generate_answer using Gemini gemini-2.5-flash with strict grounding system prompt, include conversation history context)
- [X] T050 [US2] Create conversation service backend/src/services/conversation.py (create_conversation, get_conversations, get_conversation_detail, archive_conversation, delete_conversation methods using Neon Postgres)
- [X] T051 [US2] Implement /chat/global endpoint backend/src/api/chat.py (POST, requires auth, accepts query and optional conversation_id, retrieves top 5 chunks from Qdrant, includes last 5 conversation turns in context, generates grounded answer with Gemini, stores interaction in chat_interactions, returns answer with source references)
- [X] T052 [US2] Implement /chat/history endpoint backend/src/api/conversations.py (GET, requires auth, returns paginated list of user's conversations, excludes archived by default, supports skip/limit parameters)
- [X] T053 [US2] Implement /chat/history/{conversation_id} endpoint backend/src/api/conversations.py (GET, requires auth, returns full conversation with all chat interactions ordered by created_at)
- [X] T054 [US2] Implement /chat/new endpoint backend/src/api/conversations.py (POST, requires auth, creates new conversation, returns conversation_id)
- [X] T055 [US2] Implement conversation archival cron job backend/scripts/archive_conversations.py (sets archived=TRUE for conversations where created_at < NOW() - 30 days)
- [X] T056 [US2] Add response caching for repeated queries backend/src/services/rag.py (use lru_cache with query_hash + context_hash, 30min TTL, maxsize=1000)
- [X] T057 [US2] Add Gemini API quota tracking backend/src/services/rag.py (track daily request count, return 429 when >1200 requests/day, reset counter daily)

**Checkpoint**: User Story 2 complete - users can ask questions with conversation context, view conversation history, and answers are strictly grounded in book content

---

## Phase 5: User Story 3 - Ask Questions About Selected Text (Priority: P3)

**Goal**: Enable authenticated users to select text and ask questions about only that selection with strict context isolation

**Independent Test**: Select a paragraph about ROS 2, ask "What are the key benefits mentioned here?", verify answer only uses selected text as context and does not reference other chapters

### Backend Implementation (User Story 3)

- [X] T058 [US3] Implement /chat/local endpoint backend/src/api/chat.py (POST, requires auth, accepts query, selected_text, file_path, chunk_indices, retrieves chunks filtered by file_path and chunk_index range from Qdrant, enforces strict context isolation, generates answer using only selected chunks, stores interaction with query_mode='local')
- [X] T059 [US3] Add Qdrant filtering logic backend/src/services/qdrant.py (search_selected_text method with Filter(must=[file_path match, chunk_index range]))
- [X] T060 [US3] Add validation for selected text queries backend/src/api/chat.py (minimum 50 characters for selected text, return warning if too short)

### Frontend Integration (User Story 3)

- [X] T061 [US3] Add text selection handler to book chapters (detect text selection events, capture file_path and chunk_indices)
- [X] T062 [US3] Update chatbot UI to support local query mode (show "Ask about selection" when text is selected, clear selection on chapter navigation)
- [X] T063 [US3] Add context indicator to chatbot UI (display "Global search" vs "Selected text only" mode)

**Checkpoint**: User Story 3 complete - users can ask questions about selected text with strict context isolation

---

## Phase 6: User Story 4 - View Authentication State in Header and Sidebar (Priority: P4)

**Goal**: Provide consistent authentication status in both header and sidebar with responsive mobile UI

**Independent Test**: Sign in and verify both header and sidebar update simultaneously to show user name, hide signin/signup buttons, and remain consistent in mobile view

### Frontend Implementation (User Story 4)

- [X] T064 [US4] Synchronize authentication state between header and sidebar (use shared auth context, update both components on signin/signout)
- [X] T065 [US4] Add mobile breakpoint styles to header frontend/src/components/layout/Header.tsx (mobile: <768px, tablet: 768-1023px, desktop: >1024px)
- [X] T066 [US4] Add mobile breakpoint styles to sidebar frontend/src/components/layout/Sidebar.tsx (mobile: fixed position with slide-out, tablet: 250px width, desktop: 300px width)
- [X] T067 [US4] Add touch interaction support to mobile UI (increase tap targets to 44px minimum, add touch feedback)
- [X] T068 [US4] Test authentication state updates (verify header and sidebar update within 1 second of login/logout without page refresh)

**Checkpoint**: User Story 4 complete - authentication state is consistently displayed and mobile-responsive

---

## Phase 7: User Story 5 - Receive Personalized Chapter Recommendations (Priority: P5)

**Goal**: Generate personalized chapter recommendations based on user profile, software/hardware background, and reading progress

**Independent Test**: Complete signup with specific background (e.g., Python + NVIDIA GPU), read Chapter 1, verify recommendations prioritize GPU-accelerated content and Python examples

### Backend Implementation (User Story 5)

- [ ] T069 [P] [US5] Create Recommendation model backend/src/models/recommendation.py (Recommendation schema with user_id, recommended_chapter_id, score, reason, dismissed)
- [ ] T070 [P] [US5] Create ReadingProgress model backend/src/models/progress.py (ReadingProgress schema with user_id, chapter_id, completion_percentage, time_spent_seconds, completed)
- [ ] T071 [US5] Create recommendation engine backend/src/services/recommendation_engine.py (generate_recommendations method with multi-factor scoring: experience_level 40%, software_background 30%, hardware_availability 20%, sequential_progression 10%)
- [ ] T072 [US5] Implement /progress endpoint backend/src/api/progress.py (POST, requires auth, accepts chapter_id, completion_percentage, time_spent_seconds, upserts reading_progress record, sets completed=TRUE if completion_percentage >= 90)
- [ ] T073 [US5] Implement /progress endpoint backend/src/api/progress.py (GET, requires auth, returns user's reading progress for all chapters)
- [ ] T074 [US5] Implement /recommendations endpoint backend/src/api/recommendations.py (GET, requires auth, generates personalized recommendations using recommendation engine, returns top 5 with scores and reasons, stores in recommendations table)
- [ ] T075 [US5] Implement /recommendations/{recommendation_id}/dismiss endpoint backend/src/api/recommendations.py (PUT, requires auth, sets dismissed=TRUE for recommendation)
- [ ] T076 [US5] Add chapter metadata backend/src/data/chapters.json (define all 34 chapters with difficulty, languages, frameworks, hardware_requirements, prerequisite_chapters)
- [ ] T077 [US5] Add recommendation cleanup cron job backend/scripts/cleanup_recommendations.py (keep last 10 recommendations per user, delete older ones)

### Frontend Implementation (User Story 5)

- [ ] T078 [US5] Create reading progress tracker (track scroll position, time spent on chapter, send updates to /progress endpoint every 30 seconds or on chapter exit)
- [ ] T079 [US5] Create recommendations sidebar component frontend/src/components/recommendations/RecommendationsSidebar.tsx (display 3-5 recommendations with titles, scores, reasons, dismiss button)
- [ ] T080 [US5] Add recommendation refresh on chapter completion (trigger /recommendations endpoint when user completes a chapter)
- [ ] T081 [US5] Add mobile-responsive styling to recommendations sidebar (mobile: bottom sheet, tablet: side panel, desktop: fixed sidebar)

**Checkpoint**: User Story 5 complete - users receive personalized chapter recommendations based on profile and progress

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### Documentation & Deployment

- [ ] T082 [P] Create backend README backend/README.md (setup instructions, API documentation link, environment variables)
- [ ] T083 [P] Create frontend README frontend/README.md (setup instructions, component documentation)
- [ ] T084 [P] Create Docker build for backend backend/Dockerfile (multi-stage build with Python 3.11-alpine, install dependencies, copy source, expose 8000, CMD uvicorn)
- [ ] T085 [P] Create Docker build for frontend frontend/Dockerfile (multi-stage build with Node.js, install dependencies, build production bundle, serve with nginx)
- [ ] T086 Create docker-compose.yml (define backend and frontend services, environment variables, health checks)
- [ ] T087 [P] Create deployment guide docs/DEPLOYMENT.md (instructions for Hugging Face Spaces, Railway, Render)

### Performance & Monitoring

- [ ] T088 Add health check endpoint backend/src/api/health.py (GET /health, checks Qdrant connection, Neon connection, Gemini API availability, returns status)
- [ ] T089 Add logging configuration backend/src/core/config.py (configure structured logging with log levels, log file rotation)
- [ ] T090 [P] Add error tracking integration backend/src/main.py (optional Sentry integration for production error monitoring)
- [ ] T091 Add Gemini API quota monitoring backend/src/services/rag.py (track and log daily request count, alert when approaching limit)

### Security Hardening

- [ ] T092 Add CSRF protection to mutation endpoints backend/src/core/middleware.py (verify CSRF token in POST/PUT/DELETE requests)
- [ ] T093 Add input sanitization backend/src/utils/validation.py (sanitize user inputs to prevent XSS, SQL injection)
- [ ] T094 Add secrets validation backend/src/core/config.py (verify all required environment variables are set on startup, fail fast if missing)
- [ ] T095 Add HTTPS enforcement backend/src/main.py (redirect HTTP to HTTPS in production, set Secure flag on cookies)

### Testing & Validation

- [ ] T096 Validate quickstart.md instructions (follow setup steps from scratch, verify all steps work, fix any issues)
- [ ] T097 Test all success criteria from spec.md (verify SC-001 through SC-035, document results)
- [ ] T098 Perform end-to-end user journey test (signup → signin → ask question with conversation history → get recommendations → verify all features work together)

---

## Phase 9: UI Enhancements & Visibility Fixes (Priority: P6)

**Purpose**: Improve user experience with text selection, dark mode, and authentication visibility

**Goal**: Enable "Ask AI about this" for selected text, ensure chatbot is always accessible, support dark/light mode, and fix authentication UI visibility issues

### User Story 6: Selected Text → Ask AI Flow

- [ ] T099 [P] [US6] Implement text selection detection hook src/hooks/useTextSelection.ts (detect when user selects text on book content pages, return selected text and position)
- [ ] T100 [US6] Create TextSelectionFloatingButton component src/components/chat/TextSelectionFloatingButton.tsx (floating button labeled "Ask AI about this", positioned near selected text, triggers chatbot with selected text as query)
- [ ] T101 [US6] Update ChatPanel to accept initialQuery prop src/components/chat/ChatPanel.tsx (auto-open chatbot, populate input with initial query, send as selected-text-only RAG query)
- [ ] T102 [US6] Integrate TextSelectionFloatingButton in Root.tsx src/theme/Root.tsx (render floating button when text is selected, pass selected text to ChatPanel)

### User Story 7: Chatbot Wrapper Button Visibility & Behavior

- [ ] T103 [P] [US7] Ensure chatbot wrapper button is always visible src/components/chat/ChatWrapper.module.css (fixed positioning, high z-index, visible on all pages)
- [ ] T104 [US7] Update ChatWrapper click behavior src/components/chat/ChatWrapper.tsx (instantly open chatbot UI on click, focus input field, maintain consistency across desktop and mobile)

### User Story 8: Dark Mode & Light Mode Support

- [ ] T105 [P] [US8] Configure Docusaurus dark mode docusaurus.config.js (enable colorMode with light/dark themes, set default and respect system preference)
- [ ] T106 [P] [US8] Add dark mode styles to ChatPanel src/components/chat/ChatPanel.module.css (dark mode colors for messages, input, buttons using CSS variables)
- [ ] T107 [P] [US8] Add dark mode styles to authentication pages src/components/auth/SignupForm.module.css, src/components/auth/SigninForm.module.css (dark mode colors for forms, inputs, buttons)
- [ ] T108 [US8] Add theme persistence src/theme/Root.tsx (persist user theme preference to localStorage, sync with Docusaurus theme switcher)
- [ ] T109 [US8] Test theme transitions (verify smooth transitions between light and dark mode without page reload across book content, chatbot, and auth pages)

### User Story 9: Authentication UI Visibility Fix

- [ ] T110 [P] [US9] Create authentication routes src/pages/auth/signin.tsx and src/pages/auth/signup.tsx (ensure Signin and Signup pages are properly rendered and accessible)
- [ ] T111 [US9] Fix NavbarAuth routing src/components/auth/NavbarAuth.tsx (verify links to /auth/signin and /auth/signup work correctly, show "Signin / Signup" when not logged in, show user info when logged in)
- [ ] T112 [US9] Fix SidebarAuth visibility src/components/auth/SidebarAuth.tsx (ensure "Signin / Signup" options appear in sidebar, especially at bottom, replace with user info after login)
- [ ] T113 [US9] Verify auth state consistency src/hooks/useSession.ts (ensure authentication state reflects correctly in both header and sidebar, test state updates on login/logout)

**Checkpoint**: User Story 6-9 complete - users can select text to ask AI, chatbot is always accessible, dark/light mode works everywhere, and authentication UI is fully visible and functional

---

## Phase 10: Frontend Runtime Verification (Priority: P7) 🧪

**Purpose**: Verify all features work correctly by running the frontend locally and testing in real browser

**Goal**: Ensure no runtime errors, verify all user interactions work as expected, identify and fix any integration issues

### Frontend Runtime Verification

- [ ] T114 Start development server (run npm start or yarn start, wait for server to start successfully, verify no build errors)
- [ ] T115 Verify authentication pages render correctly (navigate to /auth/signin and /auth/signup, verify forms render without "Loading..." stuck state, check browser console for errors)
- [ ] T116 Test authentication flow (perform signup with valid data, verify API call succeeds and redirects, perform signin with credentials, verify session persistence, check Network tab for failed requests)
- [ ] T117 Test selected text interaction (navigate to book content page, select text (>50 characters), verify "Ask AI about this" button appears, click button and verify chat UI opens with query, check if query is sent successfully)
- [ ] T118 Test chatbot wrapper button behavior (click chatbot button from different pages, verify chat UI opens instantly every time, verify input field receives focus, test on both desktop viewport and mobile viewport)
- [ ] T119 Test dark mode functionality (toggle dark/light mode from navbar, verify smooth transition without page reload, check all pages: book content, chat UI, auth pages, verify styles apply correctly in both modes, check for visual glitches or contrast issues)
- [ ] T120 Inspect browser console and network tab (open browser DevTools console, navigate through all features, document any console errors or warnings, check Network tab for failed API calls or stuck requests, verify all API responses return successfully)
- [ ] T121 Document verification results (create FRONTEND_VERIFICATION_REPORT.md, list what works correctly, list what does not work or has issues, document all runtime/network errors with stack traces, specify files/components that require fixes)

**Checkpoint**: All frontend features verified in runtime environment - authentication, text selection, chatbot, dark mode all working without errors

---

## Phase 11: Backend Deployment to Hugging Face Spaces (Priority: P8) 🚀

**Purpose**: Deploy the backend to Hugging Face Spaces and connect it with the already deployed frontend on Vercel

**Goal**: Backend is fully functional on Hugging Face Spaces and integrated with the Vercel frontend

**Information**:
- **Project Name (Hugging Face)**: `physical-ai-humanoid-robotics-backend`
- **Hugging Face Token**: `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`
- **Hugging Face Git Clone Link**: `git clone https://huggingface.co/spaces/Mn-2k24/physical-ai-humanoid-robotics-backend`
- **Frontend URL (Vercel)**: `https://physical-ai-humanoid-robotics-zeta.vercel.app`
- **Environment**: HF CLI already installed

### Pre-Deployment Preparation

- [ ] T122 [P] Create requirements.txt for Hugging Face backend/requirements.txt (include all Python dependencies: fastapi, uvicorn, google-generativeai, qdrant-client, psycopg[pool], pydantic, pydantic-settings, python-jose, passlib[bcrypt], slowapi, tenacity, python-multipart, python-dotenv)
- [ ] T123 [P] Create app.py for Hugging Face backend/app.py (import main app from src.main and expose it for Hugging Face Spaces: `from src.main import app`)
- [ ] T124 Create README.md for Hugging Face Space backend/HF_README.md (setup instructions, environment variables documentation, API endpoints documentation)
- [ ] T125 Update CORS configuration backend/src/main.py (add Vercel frontend URL to allowed origins: `https://physical-ai-humanoid-robotics-zeta.vercel.app`, set allow_credentials=True, allow_methods=["*"], allow_headers=["*"])

### Hugging Face Space Configuration

- [ ] T126 Create Hugging Face Space configuration .space.yaml or README.md (specify Python 3.11, SDK: gradio or docker, add environment variables: GEMINI_API_KEY, QDRANT_API_KEY, QDRANT_ENDPOINT, NEON_CONNECTION_STRING, SESSION_SECRET, FRONTEND_URL)
- [ ] T127 [P] Configure Hugging Face Secrets (add all required environment variables as Hugging Face Space secrets: GEMINI_API_KEY, QDRANT_API_KEY, QDRANT_ENDPOINT, NEON_CONNECTION_STRING, SESSION_SECRET)
- [ ] T128 Create Dockerfile for Hugging Face backend/Dockerfile (if needed: multi-stage build with Python 3.11-slim, copy requirements.txt and install, copy backend source code, expose port 7860, CMD: `uvicorn src.main:app --host 0.0.0.0 --port 7860`)

### Deployment Execution

- [ ] T129 Initialize Hugging Face git repository (login with HF CLI: `huggingface-cli login --token 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`, clone the space: `git clone https://huggingface.co/spaces/Mn-2k24/physical-ai-humanoid-robotics-backend`, navigate to cloned directory)
- [ ] T130 Copy backend files to Hugging Face repo (copy backend/src/, backend/requirements.txt, backend/app.py, backend/Dockerfile, backend/HF_README.md to the Hugging Face Space directory)
- [ ] T131 Commit and push to Hugging Face Space (git add all files, commit with message "Deploy backend to Hugging Face Spaces", push to Hugging Face: `git push origin main`)
- [ ] T132 Monitor deployment logs (check Hugging Face Space build logs, verify application starts successfully, check health endpoint: `https://mn-2k24-physical-ai-humanoid-robotics-backend.hf.space/health`)

### Frontend Integration

- [ ] T133 Update frontend API base URL (update environment variable or config in frontend to point to Hugging Face backend: `NEXT_PUBLIC_API_URL=https://mn-2k24-physical-ai-humanoid-robotics-backend.hf.space` or similar, redeploy frontend to Vercel if needed)
- [ ] T134 Test frontend-backend connection (navigate to `https://physical-ai-humanoid-robotics-zeta.vercel.app`, test authentication flow: signup and signin, test chatbot functionality: ask a question, verify responses are working, check browser console and network tab for errors)

### Post-Deployment Validation

- [ ] T135 Verify all API endpoints are accessible (test /health, /auth/signup, /auth/signin, /auth/me, /chat/global, /chat/history, verify all return successful responses from Hugging Face backend URL)
- [ ] T136 Test end-to-end user flow (complete signup on Vercel frontend → verify backend creates user in Neon Postgres, signin → verify JWT token from Hugging Face backend, ask question in chatbot → verify Qdrant retrieval and Gemini answer generation work, check conversation history → verify database persistence)
- [ ] T137 Monitor production metrics (check Hugging Face Space logs for errors, monitor Gemini API quota usage, verify Neon Postgres connection pool is healthy, check for any rate limiting issues)
- [ ] T138 Document deployment process (create DEPLOYMENT.md with step-by-step instructions, document environment variables and their purposes, document troubleshooting steps for common issues, include rollback procedure)

**Checkpoint**: Backend successfully deployed to Hugging Face Spaces, frontend on Vercel connects correctly, all features working in production

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4 → P5)
- **Polish (Phase 8)**: Depends on all desired user stories being complete
- **UI Enhancements (Phase 9)**: Depends on Phase 3 (Authentication) and chatbot implementation
  - Can proceed in parallel with other phases once core features are complete
- **Frontend Runtime Verification (Phase 10)**: Depends on Phase 9 completion
  - Must run frontend locally to verify all features work in real browser environment
- **Backend Deployment (Phase 11)**: Depends on Phase 2 (Foundational) completion
  - Can proceed once backend is functional locally
  - Frontend deployment integration depends on backend deployment completion

### User Story Dependencies

- **User Story 1 (P1) - Authentication**: BLOCKS all other stories (authentication required for all features)
- **User Story 2 (P2) - RAG with conversation history**: Depends on US1 completion
- **User Story 3 (P3) - Selected text queries**: Depends on US1 and US2 completion
- **User Story 4 (P4) - Auth UI in header/sidebar**: Depends on US1 completion
- **User Story 5 (P5) - Personalized recommendations**: Depends on US1 completion (requires user profiles and authentication)

### Within Each User Story

- Backend models before services
- Services before API endpoints
- API endpoints before frontend integration
- Core implementation before integration testing

### Parallel Opportunities

- **Setup (Phase 1)**: T002, T003, T004, T005, T006, T007 can run in parallel
- **Foundational (Phase 2)**:
  - T009, T010, T011, T012, T013, T014 (migration scripts) can run in parallel
  - T017, T018, T019 (core services) can run in parallel after T016 (config)
- **User Story 1**:
  - T027, T028 (models) can run in parallel
  - T037, T038 (frontend setup) can run in parallel
  - T030, T031, T032, T033, T034 (API endpoints) can run in parallel after T029 (auth service)
- **User Story 2**:
  - T048, T049 (models and RAG service) can run in parallel
- **User Story 5**:
  - T069, T070 (models) can run in parallel
- **Polish (Phase 8)**:
  - T082, T083, T084, T085, T087, T090 can run in parallel

---

## Parallel Example: User Story 1 (Authentication)

```bash
# After completing T029 (auth service), launch all API endpoints in parallel:
Task T030: "Implement /auth/signup endpoint"
Task T031: "Implement /auth/signin endpoint"
Task T032: "Implement /auth/signout endpoint"
Task T033: "Implement /auth/me endpoint"
Task T034: "Implement /auth/refresh endpoint"

# Frontend components can be developed in parallel:
Task T037: "Create Better Auth client"
Task T038: "Create auth context provider"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T007)
2. Complete Phase 2: Foundational (T008-T026) - CRITICAL
3. Complete Phase 3: User Story 1 (T027-T047)
4. **STOP and VALIDATE**: Test authentication flow end-to-end
5. Deploy/demo if ready

**MVP Deliverable**: Users can create accounts with background questionnaire, sign in, and all book/chatbot access is properly authenticated.

### Incremental Delivery

1. **Foundation**: Setup + Foundational → Authentication framework ready
2. **MVP**: Add User Story 1 → Test independently → Deploy/Demo
3. **Core Feature**: Add User Story 2 (RAG + conversation history) → Test independently → Deploy/Demo
4. **Enhanced Query**: Add User Story 3 (selected text) → Test independently → Deploy/Demo
5. **UI Polish**: Add User Story 4 (auth state in header/sidebar) → Test independently → Deploy/Demo
6. **Personalization**: Add User Story 5 (recommendations) → Test independently → Deploy/Demo
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (critical path)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (Authentication) - MUST complete first
3. After User Story 1 completes:
   - **Developer A**: User Story 2 (RAG + conversation history)
   - **Developer B**: User Story 4 (Auth UI polish) + User Story 5 (Recommendations)
   - **Developer C**: User Story 3 (Selected text queries)
4. Stories integrate and test independently

---

## Summary

**Total Tasks**: 138 tasks
**Task Count by User Story**:
- Setup (Phase 1): 7 tasks
- Foundational (Phase 2): 19 tasks (BLOCKING)
- User Story 1 (P1 - Authentication): 21 tasks
- User Story 2 (P2 - RAG + conversation history): 10 tasks
- User Story 3 (P3 - Selected text): 6 tasks
- User Story 4 (P4 - Auth UI): 5 tasks
- User Story 5 (P5 - Recommendations): 13 tasks
- Polish (Phase 8): 17 tasks
- UI Enhancements (Phase 9): 15 tasks
  - User Story 6 (P6 - Text selection AI): 4 tasks
  - User Story 7 (P6 - Chatbot visibility): 2 tasks
  - User Story 8 (P6 - Dark mode): 5 tasks
  - User Story 9 (P6 - Auth UI fixes): 4 tasks
- Frontend Runtime Verification (Phase 10): 8 tasks 🧪
- Backend Deployment to Hugging Face (Phase 11): 17 tasks 🚀

**Parallel Opportunities**: 38 tasks marked [P] can run in parallel within their phases

**Independent Test Criteria**:
- **US1**: Complete signup, signin, verify session persistence and access control
- **US2**: Ask related questions, verify conversation context and grounding
- **US3**: Select text, ask question, verify context isolation
- **US4**: Sign in, verify header/sidebar sync on mobile/desktop
- **US5**: Complete chapter, verify personalized recommendations match profile
- **US6**: Select text on book page, click "Ask AI about this", verify chatbot opens with query
- **US7**: Click chatbot button on any page, verify instant open and input focus
- **US8**: Toggle dark/light mode, verify consistent theming across all pages
- **US9**: Navigate to auth pages, verify signin/signup visibility and state consistency

**Suggested MVP Scope**: Phase 1 (Setup) + Phase 2 (Foundational) + Phase 3 (User Story 1 - Authentication)

**Constitution Compliance Verified**:
- ✅ Gemini API only (text-embedding-004, gemini-2.5-flash)
- ✅ Better Auth only (no custom authentication)
- ✅ Strict grounding (retrieval-only responses)
- ✅ Backend endpoints only (chatbot frontend UI out of scope)
- ✅ Qdrant for embeddings, Neon Postgres for user data
- ✅ Security hardening (password hashing, rate limiting, CSRF protection)

---

## Notes

- [P] tasks = different files, no dependencies within phase
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- Tests were NOT included as they were not requested in the feature specification
