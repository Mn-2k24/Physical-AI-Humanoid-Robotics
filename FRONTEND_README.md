# Frontend Technical Documentation

**Framework**: Docusaurus 3.9.2 + React 19.0.0 + TypeScript 5.6.2

This document provides technical details for frontend developers working on the Physical AI & Humanoid Robotics interactive book.

---

## 🏗️ Architecture Overview

### Tech Stack

- **Static Site Generator**: Docusaurus 3.9.2
- **UI Library**: React 19.0.0
- **Language**: TypeScript 5.6.2
- **Styling**: CSS Modules + Docusaurus theme system
- **State Management**: React Context API
- **Routing**: Docusaurus built-in (React Router under the hood)
- **Build Tool**: Webpack 5 (via Docusaurus)

### Key Design Patterns

1. **Component Composition**: Small, reusable components
2. **Custom Hooks**: Encapsulate business logic
3. **Context Providers**: Share state across component tree
4. **CSS Modules**: Scoped styling to avoid conflicts
5. **Theme Swizzling**: Override Docusaurus default components

---

## 📁 Detailed Project Structure

```
src/
├── components/                 # Reusable React components
│   ├── auth/                  # Authentication UI
│   │   ├── AuthProvider.tsx   # Context provider for auth state
│   │   ├── SignupForm.tsx     # Multi-step signup form
│   │   ├── SigninForm.tsx     # Signin form
│   │   ├── NavbarAuth.tsx     # Navbar auth buttons/user info
│   │   ├── SidebarAuth.tsx    # Sidebar auth info
│   │   └── AuthForms.module.css  # Shared form styles
│   │
│   ├── ChatWidget/            # Chatbot UI components
│   │   ├── ChatButton.tsx     # Floating chat toggle button
│   │   ├── ChatPanel.tsx      # Main chat panel container
│   │   ├── MessageList.tsx    # Chat message list with scroll
│   │   ├── InputBox.tsx       # Message input field
│   │   └── styles.module.css  # Chat UI styles (dark mode included)
│   │
│   ├── TextSelectionMenu/     # Selected text interaction
│   │   ├── TextSelectionMenu.tsx  # Floating "Ask AI" button
│   │   └── TextSelectionMenu.module.css
│   │
│   ├── recommendations/       # Personalized recommendations
│   │   ├── RecommendationsSidebar.tsx
│   │   └── RecommendationsSidebar.module.css
│   │
│   ├── layout/                # Layout components
│   │   └── (custom layouts if any)
│   │
│   └── HomepageFeatures/      # Landing page feature cards
│       └── index.tsx
│
├── hooks/                     # Custom React hooks
│   ├── useAuth.ts            # Authentication logic
│   ├── useChat.ts            # Chat functionality
│   ├── useSession.ts         # Session management
│   ├── useTextSelection.ts   # Text selection detection
│   └── useReadingProgress.ts # Reading progress tracking
│
├── lib/                       # Utility libraries
│   └── auth.ts               # Auth API client wrapper
│
├── pages/                     # Docusaurus pages (routes)
│   ├── index.tsx             # Homepage (/)
│   └── auth/                 # Auth pages
│       ├── signin.tsx        # /auth/signin
│       └── signup.tsx        # /auth/signup
│
├── theme/                     # Docusaurus theme overrides (swizzled)
│   ├── Root.tsx              # Global wrapper (ChatWidget injection)
│   ├── Navbar/               # Custom navbar
│   │   ├── index.tsx
│   │   └── Content/
│   │       └── index.tsx
│   └── DocSidebar/           # Custom sidebar
│       └── Desktop/
│           └── Content/
│               └── index.tsx
│
├── utils/                     # Utility functions
│   └── config.ts             # Safe config access (no process.env in browser)
│
└── css/                       # Global styles
    └── custom.css            # Custom CSS variables and overrides
```

---

## 🎨 Component API Documentation

### AuthProvider

**Purpose**: Provides authentication context to entire app

**Usage**:
```typescript
import { AuthProvider } from '@site/src/components/auth/AuthProvider';

// In Root.tsx or app entry point
<AuthProvider>
  {children}
</AuthProvider>
```

**Exports**:
```typescript
interface AuthContextValue {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  signup: (data: SignupData) => Promise<void>;
  signin: (data: SigninData) => Promise<void>;
  signout: () => Promise<void>;
  refreshSession: () => Promise<void>;
}
```

**Hook**:
```typescript
import { useAuth } from '@site/src/hooks/useAuth';

const { user, isAuthenticated, signin, signup, signout } = useAuth();
```

---

### SignupForm

**Purpose**: Multi-step user registration form

**Props**: None (standalone component)

**Features**:
- Step 1: Email, password, full name
- Step 2: Experience level selection
- Step 3: Software background (languages, frameworks)
- Step 4: Hardware background (available devices)
- Real-time validation
- Password strength indicator
- Mobile-responsive layout

**State Management**:
```typescript
const [step, setStep] = useState(1); // Current step (1-4)
const [formData, setFormData] = useState<SignupFormData>({
  email: '',
  password: '',
  full_name: '',
  experience_level: '',
  programming_languages: [],
  frameworks: [],
  available_hardware: [],
  robotics_hardware: [],
});
```

**Validation**:
- Email: RFC 5322 format
- Password: ≥8 characters, includes uppercase, lowercase, number
- Required fields per step

---

### ChatPanel

**Purpose**: Main chat interface with message history

**Props**:
```typescript
interface ChatPanelProps {
  isOpen: boolean;
  onClose: () => void;
  initialQuery?: string; // Pre-fill query and auto-send
}
```

**Features**:
- Message list with auto-scroll to bottom
- Global RAG mode (entire book search)
- Local RAG mode (selected text only)
- Conversation history (last 10 messages)
- Loading states
- Error handling
- Dark mode support

**Usage**:
```typescript
<ChatPanel
  isOpen={isChatOpen}
  onClose={() => setIsChatOpen(false)}
  initialQuery="What is ROS 2?"
/>
```

---

### TextSelectionMenu

**Purpose**: Floating button for selected text interaction

**Props**: None (auto-detects selection)

**Behavior**:
1. User selects text on page (≥50 characters)
2. Button appears near selection
3. User clicks "✨ Ask AI about this"
4. Chat panel opens with selected text
5. Local RAG query sent automatically

**Implementation**:
```typescript
const { text, position } = useTextSelection();

if (text && position) {
  return (
    <div style={{ left: position.x, top: position.y }}>
      <button onClick={() => openChatWithQuery(text)}>
        ✨ Ask AI about this
      </button>
    </div>
  );
}
```

**Positioning**:
- Uses `window.getSelection()` and `getBoundingClientRect()`
- Positioned 10px above selection
- Centered horizontally
- z-index: 9999 (above chat button)

---

### RecommendationsSidebar

**Purpose**: Display personalized chapter recommendations

**Props**:
```typescript
interface RecommendationsSidebarProps {
  refreshTrigger?: number; // Increment to refresh
}
```

**Features**:
- Fetches top 3-5 recommendations
- Score-based color coding:
  - High (≥80%): Green
  - Medium (60-79%): Yellow
  - Low (<60%): Orange
- Dismiss button per recommendation
- Manual refresh button
- Loading and error states

**API Integration**:
```typescript
const fetchRecommendations = async () => {
  const response = await fetch(`${BACKEND_URL}/recommendations`, {
    credentials: 'include',
  });
  const data = await response.json();
  setRecommendations(data.recommendations);
};
```

---

## 🪝 Custom Hooks

### useAuth

**Purpose**: Access authentication state and actions

**Returns**:
```typescript
{
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  signin: (email: string, password: string) => Promise<void>;
  signup: (data: SignupData) => Promise<void>;
  signout: () => Promise<void>;
}
```

**Usage**:
```typescript
const { user, isAuthenticated, signout } = useAuth();

if (isAuthenticated) {
  return <div>Welcome, {user.full_name}!</div>;
}
```

---

### useChat

**Purpose**: Manage chat messages and send queries

**Returns**:
```typescript
{
  messages: ChatMessage[];
  sendMessage: (query: string, mode: 'global' | 'local', selectedText?: string) => Promise<void>;
  isLoading: boolean;
  error: string | null;
  clearMessages: () => void;
}
```

**Usage**:
```typescript
const { messages, sendMessage, isLoading } = useChat();

const handleSend = async (query: string) => {
  await sendMessage(query, 'global');
};
```

---

### useTextSelection

**Purpose**: Detect text selection and calculate position

**Returns**:
```typescript
{
  text: string;
  position: { x: number; y: number; width: number; height: number } | null;
}
```

**Implementation Details**:
- Listens to `selectionchange` and `mouseup` events
- Filters selections <50 characters
- Calculates position using `Range.getBoundingClientRect()`
- Cleans up event listeners on unmount

**Usage**:
```typescript
const { text, position } = useTextSelection();

if (text && position) {
  // Show floating button
}
```

---

### useReadingProgress

**Purpose**: Track scroll position and time on chapter

**Parameters**:
```typescript
useReadingProgress(chapterId: string, options?: {
  onComplete?: () => void;
})
```

**Returns**:
```typescript
{
  completionPercentage: number; // 0-100
  timeSpent: number; // seconds
}
```

**Behavior**:
- Calculates percentage based on scroll position
- Sends update to backend every 30 seconds
- Triggers `onComplete` callback at ≥90% completion
- Sends final update on unmount

**Usage**:
```typescript
const { completionPercentage, timeSpent } = useReadingProgress('chapter-1', {
  onComplete: () => console.log('Chapter completed!'),
});
```

---

## 🎨 Styling Guidelines

### CSS Modules

**Naming Convention**: `ComponentName.module.css`

**Usage**:
```typescript
import styles from './MyComponent.module.css';

<div className={styles.container}>
  <h1 className={styles.title}>Hello</h1>
</div>
```

**Benefits**:
- Scoped class names (no global conflicts)
- Type-safe with TypeScript
- Co-located with components

---

### Dark Mode Support

**Pattern**:
```css
/* Light mode (default) */
.container {
  background: #ffffff;
  color: #000000;
}

/* Dark mode */
[data-theme='dark'] .container {
  background: var(--ifm-background-surface-color);
  color: var(--ifm-font-color-base);
}
```

**Docusaurus CSS Variables**:
- `--ifm-background-surface-color`: Surface background
- `--ifm-background-color`: Page background
- `--ifm-font-color-base`: Text color
- `--ifm-color-primary`: Brand color
- `--ifm-color-emphasis-100` to `--ifm-color-emphasis-1000`: Emphasis colors

**Example** (ChatPanel dark mode):
```css
[data-theme='dark'] .chatPanel {
  background: var(--ifm-background-surface-color);
  box-shadow: -4px 0 12px rgba(0, 0, 0, 0.3);
}

[data-theme='dark'] .message.user .bubble {
  background: var(--ifm-color-primary-darker);
  color: white;
}
```

---

### Responsive Design

**Breakpoints**:
```css
/* Mobile: < 768px */
@media (max-width: 768px) {
  .chatButton {
    width: 56px;
    height: 56px;
    bottom: 20px;
    right: 20px;
  }
}

/* Tablet: 768px - 1023px */
@media (min-width: 768px) and (max-width: 1023px) {
  .sidebar {
    width: 280px;
  }
}

/* Desktop: ≥ 1024px */
@media (min-width: 1024px) {
  .container {
    max-width: 1280px;
  }
}
```

---

## ⚙️ Configuration

### docusaurus.config.ts

**Custom Fields** (Backend URL):
```typescript
customFields: {
  apiUrl: process.env.NODE_ENV === 'production'
    ? 'https://your-backend-url.com'
    : 'http://localhost:8000',
}
```

**Accessing in Components**:
```typescript
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const { siteConfig } = useDocusaurusContext();
const backendUrl = siteConfig.customFields.apiUrl as string;
```

**Safe Access Utility**:
```typescript
// src/utils/config.ts
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export function useBackendUrl(): string {
  const { siteConfig } = useDocusaurusContext();
  return (siteConfig.customFields?.apiUrl as string) || 'http://localhost:8000';
}

export const getBackendUrl = (): string => {
  if (typeof window !== 'undefined' && (window as any).__DOCUSAURUS_BACKEND_URL__) {
    return (window as any).__DOCUSAURUS_BACKEND_URL__;
  }
  return 'http://localhost:8000';
};
```

---

### sidebars.ts

**Structure**:
```typescript
const sidebars = {
  tutorialSidebar: [
    'intro',
    'preface',
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: ['module-1-ros2/overview', 'module-1-ros2/fundamentals'],
    },
  ],
};
```

---

## 🔒 Security Best Practices

### Authentication

**HTTP-Only Cookies**:
```typescript
// Backend sets cookie
res.cookie('session_token', token, {
  httpOnly: true,      // Prevents JavaScript access (XSS protection)
  secure: true,        // HTTPS only
  sameSite: 'lax',     // CSRF protection
  maxAge: 24 * 60 * 60 * 1000, // 24 hours
});
```

**Frontend Requests**:
```typescript
fetch(`${backendUrl}/auth/me`, {
  credentials: 'include', // Send cookies
});
```

**⚠️ Never**:
- Store JWT in localStorage (vulnerable to XSS)
- Store sensitive data in localStorage
- Use `sameSite: 'none'` unless necessary

---

### Input Sanitization

**Backend Responsibility**:
- All input validation done on backend
- Frontend only provides UX validation (real-time feedback)

**Frontend Validation**:
```typescript
// Example: Email validation
const isValidEmail = (email: string) => {
  const re = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return re.test(email);
};

// Example: Password strength
const getPasswordStrength = (password: string) => {
  if (password.length < 8) return 'weak';
  if (!/[a-z]/.test(password) || !/[A-Z]/.test(password)) return 'weak';
  if (!/[0-9]/.test(password)) return 'medium';
  return 'strong';
};
```

---

## 🧪 Development Workflow

### Adding New Feature

1. **Read Spec**: Check `specs/<feature>/spec.md`
2. **Check Tasks**: Review `specs/<feature>/tasks.md`
3. **Create Component**: Add to `src/components/`
4. **Add Hook** (if needed): Create in `src/hooks/`
5. **Style**: Add `ComponentName.module.css`
6. **Integrate**: Import in parent component or `Root.tsx`
7. **Test**: Manual testing in browser
8. **Document**: Update this README if needed

---

### Swizzling Docusaurus Components

**Eject Component**:
```bash
npm run swizzle @docusaurus/theme-classic ComponentName -- --eject
```

**Wrap Component**:
```bash
npm run swizzle @docusaurus/theme-classic ComponentName -- --wrap
```

**Example** (Navbar Content):
```bash
npm run swizzle @docusaurus/theme-classic Navbar/Content -- --wrap
```

This creates: `src/theme/Navbar/Content/index.tsx`

---

### Type Checking

```bash
npm run typecheck
```

**Common Errors**:
- Missing type definitions: Install `@types/package-name`
- Implicit any: Add type annotations
- Module not found: Check import paths

---

## 📦 Build & Deployment

### Production Build

```bash
npm run build
```

**Output**: `build/` directory with static files

**Build Process**:
1. TypeScript compilation
2. React component bundling
3. Static site generation (SSG)
4. Asset optimization
5. HTML generation for all routes

---

### Environment-Specific Config

**Development**:
- Backend: `http://localhost:8000`
- Hot module replacement enabled
- Source maps enabled

**Production**:
- Backend: `https://your-backend-url.com`
- Minified bundles
- Optimized assets
- No source maps

**Set Production Backend**:
```typescript
// docusaurus.config.ts
customFields: {
  apiUrl: process.env.NODE_ENV === 'production'
    ? 'https://your-backend-url.com'  // ← Update this
    : 'http://localhost:8000',
}
```

---

### Deployment Checklist

- [ ] Update backend URL in `docusaurus.config.ts`
- [ ] Run `npm run build` successfully
- [ ] Test production build locally (`npm run serve`)
- [ ] Verify all features work (auth, chat, dark mode)
- [ ] Check browser console for errors
- [ ] Test on mobile devices
- [ ] Verify backend CORS allows frontend domain

---

## 🐛 Common Issues & Solutions

### "process is not defined"

**Cause**: Using `process.env` in browser code

**Solution**: Use `src/utils/config.ts` utilities
```typescript
// ❌ Don't do this
const url = process.env.REACT_APP_BACKEND_URL;

// ✅ Do this
import { getBackendUrl } from '@site/src/utils/config';
const url = getBackendUrl();
```

---

### CORS Errors

**Symptoms**: `Access to fetch ... has been blocked by CORS policy`

**Solution**:
1. Add frontend URL to backend CORS origins
2. Ensure `credentials: 'include'` in fetch requests
3. Check backend allows credentials in CORS config

---

### Dark Mode Not Applying

**Symptoms**: Styles don't change when toggling theme

**Solution**:
1. Use `[data-theme='dark']` selectors in CSS
2. Use Docusaurus CSS variables (not hardcoded colors)
3. Clear browser cache
4. Check localStorage for `theme` key

---

### Authentication Session Lost

**Symptoms**: User logged out after page refresh

**Solution**:
1. Verify cookie is HTTP-only and has correct domain
2. Check cookie `maxAge` / `expires`
3. Ensure `credentials: 'include'` in all API calls
4. Check backend session validation logic

---

## 📊 Performance Optimization

### Code Splitting

**Automatic** (Docusaurus):
- Each route is a separate bundle
- Lazy-loaded on navigation

**Manual** (React.lazy):
```typescript
const HeavyComponent = React.lazy(() => import('./HeavyComponent'));

<Suspense fallback={<div>Loading...</div>}>
  <HeavyComponent />
</Suspense>
```

---

### Image Optimization

**Use WebP**:
```markdown
![Alt text](./image.webp)
```

**Lazy Loading**:
```jsx
<img src="image.jpg" loading="lazy" alt="Description" />
```

---

### Bundle Analysis

```bash
# Build with stats
DOCUSAURUS_BUNDLE_ANALYZER=true npm run build
```

Opens interactive bundle visualization.

---

## 📚 Additional Resources

- **Docusaurus Docs**: https://docusaurus.io/docs
- **React Docs**: https://react.dev/
- **TypeScript Handbook**: https://www.typescriptlang.org/docs/handbook/
- **CSS Modules**: https://github.com/css-modules/css-modules

---

**Last Updated**: 2025-12-14
