---
id: 0006
title: Add UI Enhancement Tasks to RAG Chatbot
stage: tasks
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-chatbot-auth
branch: 001-rag-chatbot-auth
user: nizam
command: /sp.tasks
labels: ["tasks", "ui-enhancements", "dark-mode", "text-selection", "authentication-ui"]
links:
  spec: specs/001-rag-chatbot-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot-auth/tasks.md
tests:
  - null
---

## Prompt

User requested via /sp.tasks command:

```
please add these tasks in specs/001-rag-chatbot-auth/tasks.md file
# Tasks: RAG Chatbot UI Enhancements & Authentication Visibility

## 1. Selected Text → Ask AI Flow
- Detect when a user selects text inside the book content pages.
- Show a contextual floating button labeled **"Ask AI about this"** near the selected text.
- On button click:
  - Automatically open the chatbot UI.
  - Send the selected text as the initial query to the chatbot backend.
  - Ensure the query is handled as a **selected-text-only RAG query**.

## 2. Chatbot Wrapper Button Behavior
- Ensure the global chatbot wrapper/floating button is always visible.
- When the chatbot wrapper button is clicked:
  - Instantly open the chatbot UI.
  - Focus the input field for user interaction.
- Maintain consistent behavior across desktop and mobile devices.

## 3. Dark Mode & Light Mode Support
- Enable both **Light Mode** and **Dark Mode** across the entire project.
- Modes should affect:
  - Book content
  - Chatbot UI
  - Authentication pages (Signin / Signup)
- Persist user preference (local storage or system preference).
- Ensure smooth transition between modes without page reload.

## 4. Authentication UI Visibility Fix
- Ensure **Signin** and **Signup** pages are properly rendered and accessible.
- Fix routing issues if auth pages are not visible.
- Display authentication options correctly:
  - Header: show **Signin / Signup** when user is not logged in.
  - Sidebar: show **Signin / Signup** options (especially at the bottom).
- After login:
  - Replace auth buttons with logged-in user information (name/email).
  - Ensure auth state reflects consistently in header and sidebar.

- Do not change anything apart from these tasks.
```

## Response snapshot

Added Phase 9 to tasks.md with 15 new tasks organized into 4 user stories:

**Phase 9: UI Enhancements & Visibility Fixes (Priority: P6)**

### User Story 6: Selected Text → Ask AI Flow (4 tasks)
- T099: Implement text selection detection hook (useTextSelection.ts)
- T100: Create TextSelectionFloatingButton component
- T101: Update ChatPanel to accept initialQuery prop
- T102: Integrate TextSelectionFloatingButton in Root.tsx

### User Story 7: Chatbot Wrapper Button Visibility (2 tasks)
- T103: Ensure chatbot wrapper button always visible (CSS)
- T104: Update ChatWrapper click behavior (instant open, focus input)

### User Story 8: Dark Mode & Light Mode Support (5 tasks)
- T105: Configure Docusaurus dark mode (docusaurus.config.js)
- T106: Add dark mode styles to ChatPanel
- T107: Add dark mode styles to authentication pages
- T108: Add theme persistence (localStorage)
- T109: Test theme transitions

### User Story 9: Authentication UI Visibility Fix (4 tasks)
- T110: Create authentication routes (signin.tsx, signup.tsx)
- T111: Fix NavbarAuth routing
- T112: Fix SidebarAuth visibility
- T113: Verify auth state consistency

Updated summary section:
- Total tasks: 98 → 113 tasks
- Added test criteria for US6-US9
- Updated parallel opportunities: 28 → 35 tasks
- Added Phase 9 dependencies (requires Phase 3 Authentication + chatbot implementation)

## Outcome

- ✅ Impact: Added 15 new tasks for UI enhancements without modifying existing tasks
- 🧪 Tests: Test criteria added for 4 new user stories (US6-US9)
- 📁 Files: Modified specs/001-rag-chatbot-auth/tasks.md
- 🔁 Next prompts: /sp.implement to execute Phase 9 tasks, or continue with existing phases
- 🧠 Reflection: Tasks properly organized by user story, all following checklist format with [P] markers for parallelizable tasks

## Evaluation notes (flywheel)

- Failure modes observed: None - task addition completed without issues
- Graders run and results (PASS/FAIL): Format validation PASS - all tasks follow `- [ ] [ID] [P?] [Story] Description (file path)` format
- Prompt variant (if applicable): Standard /sp.tasks workflow
- Next experiment (smallest change to try): User can now execute /sp.implement for Phase 9 or continue with pending tasks from earlier phases
