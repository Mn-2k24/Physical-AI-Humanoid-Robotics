# Recommendations System Integration Guide

This guide shows how to integrate the reading progress tracker with personalized recommendations.

## Components

### 1. `useReadingProgress` Hook
Tracks user scroll position and time spent on chapters. Sends updates to backend every 30 seconds.

### 2. `RecommendationsSidebar` Component
Displays 3-5 personalized chapter recommendations based on user profile and progress.

## Integration Example

```typescript
import React, { useState } from 'react';
import { useReadingProgress } from '@site/src/hooks/useReadingProgress';
import { RecommendationsSidebar } from '@site/src/components/recommendations/RecommendationsSidebar';

export const ChapterPage: React.FC<{ chapterId: string }> = ({ chapterId }) => {
  const [refreshTrigger, setRefreshTrigger] = useState(0);

  // Track reading progress and refresh recommendations on completion
  const { completionPercentage, timeSpent } = useReadingProgress(chapterId, {
    onComplete: () => {
      // Trigger recommendation refresh when chapter reaches 90%
      setRefreshTrigger(Date.now());
    },
  });

  return (
    <div>
      {/* Chapter content */}
      <main>
        <h1>Chapter Content</h1>
        <p>Completion: {completionPercentage}%</p>
        <p>Time spent: {Math.floor(timeSpent / 60)} minutes</p>
      </main>

      {/* Recommendations sidebar */}
      <RecommendationsSidebar refreshTrigger={refreshTrigger} />
    </div>
  );
};
```

## How It Works

1. **Progress Tracking**: `useReadingProgress` calculates completion based on scroll position
2. **Periodic Updates**: Sends progress to `/progress` endpoint every 30 seconds
3. **Completion Detection**: When completion reaches 90%, triggers `onComplete` callback
4. **Recommendation Refresh**: Parent component updates `refreshTrigger`, causing sidebar to refetch
5. **Backend Generation**: Backend generates new recommendations based on updated progress

## API Flow

```
User scrolls → useReadingProgress tracks position
             ↓
Every 30s → POST /progress (completion_percentage, time_spent)
             ↓
Reaches 90% → onComplete() callback
             ↓
Parent updates refreshTrigger → RecommendationsSidebar refetches
             ↓
GET /recommendations → Backend generates new personalized recommendations
```

## Mobile Responsiveness

The sidebar is fully responsive:

- **Desktop (>1024px)**: Fixed sidebar on right (320px width)
- **Tablet (768-1023px)**: Narrower side panel (280px width)
- **Mobile (<768px)**: Bottom sheet with 50vh max height
- **Touch devices**: All interactive elements have 44px minimum tap targets

## Customization

### Changing Completion Threshold

By default, completion is triggered at 90%. To change this, modify `useReadingProgress.ts`:

```typescript
// Change from 90 to your desired threshold
if (data.completionPercentage >= 90 && !isCompleted) {
  // ...
}
```

### Changing Recommendation Limit

By default, 5 recommendations are shown. To change this, modify `RecommendationsSidebar.tsx`:

```typescript
{recommendations.slice(0, 5).map((rec, index) => (
  // Change 5 to your desired limit
```

Or modify the backend in `recommendation_engine.py`:

```python
def generate_recommendations(
    self,
    user_profile: Dict,
    completed_chapters: List[str],
    in_progress_chapters: List[str],
    limit: int = 5,  # Change default here
) -> List[Dict]:
```
