/**
 * Text Selection Detection Hook
 * Purpose: Detect when user selects text on book content pages and return selected text with position
 * Date: 2025-12-14
 */

import { useEffect, useState, useCallback } from 'react';

interface SelectionPosition {
  x: number;
  y: number;
  width: number;
  height: number;
}

interface TextSelection {
  text: string;
  position: SelectionPosition | null;
}

export const useTextSelection = (): TextSelection => {
  const [selection, setSelection] = useState<TextSelection>({
    text: '',
    position: null,
  });

  const handleSelectionChange = useCallback(() => {
    const selectedText = window.getSelection()?.toString().trim() || '';

    if (selectedText.length > 0) {
      const range = window.getSelection()?.getRangeAt(0);
      if (range) {
        const rect = range.getBoundingClientRect();
        setSelection({
          text: selectedText,
          position: {
            x: rect.left + rect.width / 2, // Center horizontally
            y: rect.top - 10, // Position above selection
            width: rect.width,
            height: rect.height,
          },
        });
      }
    } else {
      // Clear selection if nothing is selected
      setSelection({
        text: '',
        position: null,
      });
    }
  }, []);

  useEffect(() => {
    // Listen for selection changes
    document.addEventListener('selectionchange', handleSelectionChange);

    // Also listen for mouseup to catch selections
    document.addEventListener('mouseup', handleSelectionChange);

    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mouseup', handleSelectionChange);
    };
  }, [handleSelectionChange]);

  return selection;
};
