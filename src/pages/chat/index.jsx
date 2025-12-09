import React from 'react';
import Layout from '@theme/Layout';
import RagChat from '@site/src/components/RagChat';

export default function ChatPage() {
  return (
    <Layout 
      title="AI Assistant" 
      description="Chat with the AI Robotics Textbook"
      noFooter={true}
    >
      <div style={{ 
        height: 'calc(100vh - 60px)',
        width: '100%',
        display: 'flex'
      }}>
        <RagChat />
      </div>
    </Layout>
  );
}
