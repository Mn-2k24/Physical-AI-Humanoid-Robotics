# Responsive Design Testing Checklist

**Project**: Physical AI & Humanoid Robotics Book
**Purpose**: Ensure consistent responsive behavior across all devices and browsers
**Last Updated**: 2025-12-07

---

## Quick Start

### Local Testing Setup

```bash
# Build production version
npm run build

# Serve production build
npm run serve

# Open browser to http://localhost:3000
```

### Browser DevTools Setup

1. Open Chrome/Firefox DevTools (F12)
2. Click "Toggle Device Toolbar" (Ctrl+Shift+M / Cmd+Shift+M)
3. Select device or set custom viewport width
4. Test both portrait and landscape orientations

---

## Testing Matrix

### Viewport Widths

| Device Category | Viewport Width | Device Examples | Priority |
|----------------|----------------|-----------------|----------|
| **Mobile Small** | 375px | iPhone SE, iPhone 12/13 Mini | HIGH |
| **Mobile Standard** | 390px | iPhone 14/15 | HIGH |
| **Mobile Large** | 414px | iPhone 14 Pro Max | MEDIUM |
| **Tablet Portrait** | 768px | iPad Mini, iPad | HIGH |
| **Tablet Landscape** | 1024px | iPad, iPad Air | MEDIUM |
| **Laptop Small** | 1280px | MacBook Air 13" | MEDIUM |
| **Laptop Standard** | 1440px | MacBook Pro 14" | LOW |
| **Desktop** | 1920px | Full HD monitors | LOW |

---

## Test Cases

### 1. Home Page (`/`)

#### Hero Section

- [ ] **Title Scaling** (All viewports)
  - [ ] Title is readable without horizontal scrolling
  - [ ] Title size scales smoothly (no abrupt changes)
  - [ ] Title doesn't wrap to more than 2 lines on mobile

- [ ] **Subtitle Text** (All viewports)
  - [ ] Subtitle is legible (min 16px equivalent)
  - [ ] Subtitle scales appropriately with viewport

- [ ] **Call-to-Action Buttons** (All viewports)
  - [ ] Buttons are horizontally centered
  - [ ] Buttons have visible spacing/gap
  - [ ] Buttons don't overflow viewport width

- [ ] **Mobile Button Layout** (<768px)
  - [ ] Buttons stack vertically (one per row)
  - [ ] Each button is at least 44px tall
  - [ ] Buttons are centered and full-width (max 300px)
  - [ ] Gap between buttons is visible (0.75rem)

- [ ] **Hero Subtext** (All viewports)
  - [ ] Text is readable without horizontal scrolling
  - [ ] Bold keywords stand out
  - [ ] Text has appropriate side padding on mobile

#### Feature Cards Section

- [ ] **Section Header** (All viewports)
  - [ ] "4 Progressive Modules" heading is centered
  - [ ] Subtitle is readable and centered

- [ ] **Card Grid Layout**
  - [ ] **Desktop (>=1024px)**: 2 columns (2×2 grid)
  - [ ] **Tablet (768-1023px)**: 2 columns
  - [ ] **Mobile (<768px)**: 1 column (vertical stack)

- [ ] **Individual Cards** (All viewports)
  - [ ] Card has visible border and shadow
  - [ ] Hover effect works (lift + shadow) on desktop
  - [ ] Card padding scales appropriately

- [ ] **Emoji Icons** (All viewports)
  - [ ] Emojis scale smoothly (not too large on mobile)
  - [ ] Emojis maintain 1:1 aspect ratio
  - [ ] Emojis don't dominate card on small screens

- [ ] **Card Text** (All viewports)
  - [ ] Module title is readable
  - [ ] Description text doesn't overflow
  - [ ] Text has appropriate padding on all sides

- [ ] **Card Touch Targets** (Mobile <768px)
  - [ ] Entire card is tappable
  - [ ] Cards have visible spacing between them
  - [ ] No accidental taps on adjacent cards

- [ ] **Learning Objectives Button** (All viewports)
  - [ ] Button is centered
  - [ ] Button is at least 44px tall on mobile
  - [ ] Button has visible spacing from cards above

### 2. Documentation Pages (`/docs/*`)

#### Article Content

- [ ] **Headers** (All viewports)
  - [ ] H1 scales smoothly (clamp: 1.75rem → 2.5rem)
  - [ ] H2 scales smoothly (clamp: 1.5rem → 2rem)
  - [ ] Headers don't wrap excessively on mobile

- [ ] **Body Text** (All viewports)
  - [ ] Base font size is readable (min 16px)
  - [ ] Line length is comfortable (not too wide)
  - [ ] Code blocks don't cause horizontal scroll

- [ ] **Code Blocks** (All viewports)
  - [ ] Code wraps or scrolls horizontally (no page scroll)
  - [ ] Copy button is accessible on mobile

- [ ] **Blockquotes** (All viewports)
  - [ ] Border is visible
  - [ ] Background color is distinct
  - [ ] Padding is appropriate for content

- [ ] **Images** (All viewports)
  - [ ] Images scale to container width
  - [ ] Images maintain aspect ratio
  - [ ] No image overflow

#### Sidebar Navigation

- [ ] **Desktop (>=997px)**
  - [ ] Sidebar is visible on left
  - [ ] Sidebar doesn't overlap content
  - [ ] Sidebar is scrollable independently

- [ ] **Tablet/Mobile (<997px)**
  - [ ] Sidebar is hidden by default
  - [ ] Hamburger menu icon is visible (top left)
  - [ ] Hamburger menu is at least 44×44px
  - [ ] Sidebar slides in when hamburger clicked
  - [ ] Sidebar overlay closes when tapping outside

#### Table of Contents

- [ ] **Desktop (>=997px)**
  - [ ] TOC is visible on right
  - [ ] TOC scrolls with page
  - [ ] TOC links are clickable

- [ ] **Tablet/Mobile (<997px)**
  - [ ] TOC is hidden or collapsed
  - [ ] Page content uses full width

### 3. Navigation Bar

#### Desktop Navigation (>=997px)

- [ ] **Logo**
  - [ ] Logo is visible and scaled appropriately
  - [ ] Logo links to home page

- [ ] **Menu Items**
  - [ ] "Book" link is visible
  - [ ] "Modules" dropdown is visible
  - [ ] GitHub links are visible

- [ ] **Dropdown Menu**
  - [ ] Hovering "Modules" opens dropdown
  - [ ] All 4 module links are visible
  - [ ] Dropdown closes when clicking outside

#### Mobile Navigation (<997px)

- [ ] **Hamburger Menu**
  - [ ] Hamburger icon is visible (top left)
  - [ ] Icon is at least 44×44px
  - [ ] Icon has visible touch target

- [ ] **Mobile Menu**
  - [ ] Menu slides in from left
  - [ ] All links are accessible
  - [ ] Modules dropdown expands on tap
  - [ ] All 4 module links are tappable
  - [ ] Menu closes when tapping outside

- [ ] **Logo on Mobile**
  - [ ] Logo is visible and appropriately sized
  - [ ] Logo doesn't overlap hamburger icon

### 4. Footer

- [ ] **Layout** (All viewports)
  - [ ] **Desktop**: Columns side-by-side
  - [ ] **Mobile (<768px)**: Columns stack vertically

- [ ] **Links** (All viewports)
  - [ ] All footer links are clickable
  - [ ] Links have at least 44px height on mobile
  - [ ] Links have visible spacing

- [ ] **Copyright** (All viewports)
  - [ ] Copyright text is centered
  - [ ] Text is readable

---

## Browser-Specific Tests

### Chrome/Edge (Chromium)

- [ ] Fluid typography (`clamp()`) works correctly
- [ ] Flexbox gaps render properly
- [ ] CSS custom properties are applied
- [ ] Hover effects work

### Firefox

- [ ] Fluid typography (`clamp()`) works correctly
- [ ] Flexbox gaps render properly
- [ ] CSS custom properties are applied
- [ ] Hover effects work

### Safari (macOS/iOS)

- [ ] Fluid typography (`clamp()`) works correctly
- [ ] Flexbox gaps render properly
- [ ] Touch interactions work on iOS
- [ ] Dropdown menus work on touch

---

## Accessibility Tests

### Touch Target Size (Mobile Only)

- [ ] All buttons are at least 44×44px
- [ ] All navigation links are at least 44px tall
- [ ] Feature cards have adequate touch area
- [ ] Dropdown menu items are tappable
- [ ] No targets overlap or are too close (<8px gap)

### Font Size & Readability

- [ ] Base font is at least 16px (no zoom needed)
- [ ] Headers are proportionally larger
- [ ] Code font is legible (not too small)
- [ ] All text has sufficient contrast

### Keyboard Navigation

- [ ] Tab order is logical
- [ ] Focus indicators are visible
- [ ] All interactive elements are reachable
- [ ] Escape key closes modals/dropdowns

---

## Performance Tests

### Lighthouse Audit

Run Lighthouse on both mobile and desktop:

```bash
# Serve production build
npm run serve

# Open Chrome DevTools > Lighthouse
# Run audit for both Mobile and Desktop
```

#### Target Scores

| Metric | Mobile Target | Desktop Target |
|--------|---------------|----------------|
| Performance | ≥90 | ≥90 |
| Accessibility | ≥95 | ≥95 |
| Best Practices | ≥95 | ≥95 |
| SEO | ≥90 | ≥90 |

#### Core Web Vitals

| Metric | Target |
|--------|--------|
| First Contentful Paint (FCP) | <1.5s |
| Largest Contentful Paint (LCP) | <2.5s |
| Cumulative Layout Shift (CLS) | <0.1 |
| Total Blocking Time (TBT) | <300ms |

### Network Throttling

Test with simulated slow connections:

- [ ] **Fast 3G** (750ms RTT, 1.4Mbps down)
  - [ ] Page loads in <5s
  - [ ] Interactive in <7s

- [ ] **Slow 3G** (2000ms RTT, 400Kbps down)
  - [ ] Page loads in <10s
  - [ ] Basic content visible in <5s

---

## Visual Regression Tests

### Screenshot Comparison

Capture screenshots at key viewports and compare after changes:

1. Home page hero - 375px, 768px, 1440px
2. Feature cards grid - 375px, 768px, 1440px
3. Documentation page - 375px, 768px, 1440px
4. Navigation menu (open) - 375px, 768px

### Print Preview

- [ ] Open print preview (Ctrl+P / Cmd+P)
- [ ] Navbar is hidden
- [ ] Footer is hidden
- [ ] TOC is hidden
- [ ] Content is full-width
- [ ] Page breaks are logical

---

## Edge Cases

### Very Large Screens (>1920px)

- [ ] Content doesn't stretch excessively
- [ ] Max-width containers are respected
- [ ] Typography doesn't exceed max size

### Very Small Screens (<375px)

- [ ] No horizontal scrolling
- [ ] Content is accessible
- [ ] Buttons remain tappable

### Long Content

- [ ] Long article titles wrap appropriately
- [ ] Long paragraphs don't cause overflow
- [ ] Long code blocks scroll horizontally (not page)

### No JavaScript

- [ ] CSS-only responsive features work
- [ ] Media queries apply correctly
- [ ] Print styles work

---

## Regression Test (After Future Updates)

Run this checklist after:
- Adding new components
- Updating Docusaurus version
- Modifying CSS/styling
- Changing layout structure

### Quick Regression Checks

1. **Build succeeds**: `npm run build` → no errors
2. **Home page loads**: Open `/` → verify hero + cards
3. **Mobile menu works**: <768px → tap hamburger → verify menu
4. **Buttons stack on mobile**: <768px → verify vertical layout
5. **Cards single column**: <768px → verify 1-column grid

---

## Issue Tracking Template

When finding issues, document with:

```markdown
### Issue: [Brief description]

**Viewport**: [Width in px]
**Browser**: [Chrome/Firefox/Safari + version]
**Location**: [Page URL + component]

**Expected**: [What should happen]
**Actual**: [What actually happens]

**Screenshot**: [Attach if applicable]

**Steps to Reproduce**:
1. [Step 1]
2. [Step 2]
3. [Step 3]
```

---

## Sign-Off Checklist

Before marking Phase 11 complete:

- [ ] All test cases passed on mobile (375px)
- [ ] All test cases passed on tablet (768px)
- [ ] All test cases passed on desktop (1440px)
- [ ] Lighthouse mobile score ≥90
- [ ] Lighthouse desktop score ≥90
- [ ] No horizontal scrolling on any viewport
- [ ] All touch targets ≥44px on mobile
- [ ] No visual regressions from Phase 10

**Tester Name**: _______________
**Test Date**: _______________
**Sign-Off**: _______________

---

**Version**: 1.0.0
**Created**: 2025-12-07
**Project**: Physical AI & Humanoid Robotics Book
