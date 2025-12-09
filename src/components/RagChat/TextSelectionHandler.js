/**
 * Text Selection Handler Hook
 *
 * This hook detects when users highlight/select text on the page
 * and provides the selected text to the chatbot for focused explanations.
 *
 * Usage:
 *   const selectedText = useTextSelection();
 */

import { useState, useEffect } from 'react';

/**
 * Custom hook to track text selection in the browser
 *
 * @returns {string} Currently selected text, or empty string if no selection
 */
export function useTextSelection() {
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    /**
     * Handle text selection events
     * Fires when user selects/deselects text
     */
    const handleSelection = () => {
      // Get the current selection from the browser
      const selection = window.getSelection();

      if (selection && selection.toString().trim()) {
        // User has selected some text
        const text = selection.toString().trim();

        // Only update if selection is meaningful (more than a few characters)
        if (text.length > 5) {
          setSelectedText(text);
        }
      } else {
        // No selection or empty selection
        setSelectedText('');
      }
    };

    // Listen for mouseup events (user finished selecting text)
    document.addEventListener('mouseup', handleSelection);

    // Listen for keyup events (keyboard selection with Shift+Arrow)
    document.addEventListener('keyup', handleSelection);

    // Listen for selectionchange events (more comprehensive, but can be noisy)
    // Uncomment if you want real-time updates as user drags selection
    // document.addEventListener('selectionchange', handleSelection);

    // Cleanup: remove event listeners when component unmounts
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
      // document.removeEventListener('selectionchange', handleSelection);
    };
  }, []);

  return selectedText;
}

/**
 * Helper function to manually get current selection
 * Useful for click handlers or manual checks
 *
 * @returns {string} Currently selected text
 */
export function getCurrentSelection() {
  const selection = window.getSelection();
  return selection ? selection.toString().trim() : '';
}

/**
 * Helper function to clear current selection
 * Useful after processing the selected text
 */
export function clearSelection() {
  if (window.getSelection) {
    const selection = window.getSelection();
    if (selection) {
      selection.removeAllRanges();
    }
  }
}
