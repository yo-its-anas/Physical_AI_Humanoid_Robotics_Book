/**
 * RAG Chat Component - Main Export
 *
 * This is the main entry point for the RAG chatbot component.
 * Import this component into your Docusaurus pages or layouts.
 *
 * Usage in a Docusaurus page:
 *
 * ```jsx
 * import RagChat from '@site/src/components/RagChat';
 *
 * export default function MyPage() {
 *   return (
 *     <div>
 *       <h1>My Page</h1>
 *       <RagChat />
 *     </div>
 *   );
 * }
 * ```
 */

import React from 'react';
import ChatKitWrapper from './ChatKitWrapper';

export default function RagChat() {
  return <ChatKitWrapper />;
}

// Also export subcomponents for advanced use cases
export { default as ChatKitWrapper } from './ChatKitWrapper';
export { useTextSelection, getCurrentSelection, clearSelection } from './TextSelectionHandler';
export { BACKEND_URL, API_ENDPOINTS, CHATBOT_CONFIG } from './config';
