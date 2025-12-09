/**
 * Docusaurus Root Component (Theme Swizzle)
 *
 * This wraps all pages and provides global components like the floating chat.
 * Place at: src/theme/Root.js
 */

import React from 'react';
import FloatingChat from '@site/src/components/FloatingChat';

export default function Root({ children }) {
  return (
    <>
      {children}
      <FloatingChat />
    </>
  );
}
