/**
 * ChatKit Wrapper Component
 *
 * This component provides a ChatGPT-style UI using the ChatKit SDK.
 * It supports two modes:
 *   1. Normal RAG: User asks questions, chatbot searches the textbook
 *   2. Selected Text: User highlights text and asks about it
 *
 * The component automatically detects which mode to use based on whether
 * text is selected when the user submits their question.
 */

import React, { useState, useRef, useEffect } from 'react';
import {
  MainContainer,
  ChatContainer,
  MessageList,
  Message,
  MessageInput,
  TypingIndicator,
  Avatar
} from '@chatscope/chat-ui-kit-react';
import '@chatscope/chat-ui-kit-styles/dist/default/styles.min.css';

import { API_ENDPOINTS, CHATBOT_CONFIG } from './config';
import { useTextSelection } from './TextSelectionHandler';
import styles from './styles.module.css';

/**
 * Main ChatKit Wrapper Component
 */
export default function ChatKitWrapper() {
  // State for chat messages
  const [messages, setMessages] = useState([
    {
      message: CHATBOT_CONFIG.welcomeMessage,
      sender: 'bot',
      direction: 'incoming',
      timestamp: Date.now()
    }
  ]);

  // State for loading indicator
  const [isTyping, setIsTyping] = useState(false);

  // Track selected text from the page
  const selectedText = useTextSelection();
  const [lastSelectedText, setLastSelectedText] = useState('');

  // Ref for message input (to show selected text hint)
  const inputRef = useRef(null);

  /**
   * Update hint message when text is selected
   */
  useEffect(() => {
    if (selectedText && selectedText !== lastSelectedText) {
      setLastSelectedText(selectedText);
      // Could show a toast notification here: "Text selected! Ask me about it."
    }
  }, [selectedText, lastSelectedText]);

  /**
   * Handle sending a message
   *
   * @param {string} userMessage - The user's question
   */
  const handleSend = async (userMessage) => {
    if (!userMessage.trim()) return;

    // Determine if we're in Selected Text mode
    const currentSelectedText = selectedText || '';
    const mode = currentSelectedText ? 'selected_text' : 'normal_rag';

    // Add user message to chat
    const newUserMessage = {
      message: userMessage,
      sender: 'user',
      direction: 'outgoing',
      timestamp: Date.now(),
      // Show selected text indicator if in Selected Text mode
      metadata: mode === 'selected_text' ? {
        selectedTextPreview: currentSelectedText.substring(0, 100) + '...'
      } : null
    };

    setMessages(prev => [...prev, newUserMessage]);
    setIsTyping(true);

    try {
      // Call backend API
      const response = await fetch(API_ENDPOINTS.query, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          question: userMessage,
          selectedText: currentSelectedText || null
        }),
        signal: AbortSignal.timeout(CHATBOT_CONFIG.requestTimeout)
      });

      if (!response.ok) {
        // Handle HTTP errors
        if (response.status === 404) {
          throw new Error(CHATBOT_CONFIG.errors.noResults);
        } else if (response.status === 429) {
          throw new Error(CHATBOT_CONFIG.errors.rateLimit);
        } else {
          const errorData = await response.json().catch(() => ({}));
          throw new Error(errorData.detail || CHATBOT_CONFIG.errors.generic);
        }
      }

      const data = await response.json();

      // Add bot response to chat
      const botMessage = {
        message: data.answer,
        sender: 'bot',
        direction: 'incoming',
        timestamp: Date.now(),
        metadata: {
          mode: data.mode,
          sources: data.sources || [],
          responseTime: data.response_time_ms
        }
      };

      setMessages(prev => [...prev, botMessage]);

      // Clear selected text after using it
      if (currentSelectedText) {
        setLastSelectedText('');
      }

    } catch (error) {
      // Handle errors
      let errorMessage = CHATBOT_CONFIG.errors.generic;

      if (error.name === 'AbortError') {
        errorMessage = CHATBOT_CONFIG.errors.timeout;
      } else if (error.message.includes('fetch')) {
        errorMessage = CHATBOT_CONFIG.errors.network;
      } else if (error.message) {
        errorMessage = error.message;
      }

      const errorBotMessage = {
        message: `⚠️ ${errorMessage}`,
        sender: 'bot',
        direction: 'incoming',
        timestamp: Date.now(),
        isError: true
      };

      setMessages(prev => [...prev, errorBotMessage]);
    } finally {
      setIsTyping(false);
    }
  };

  /**
   * Render a custom message with sources (for Normal RAG mode)
   */
  const renderMessage = (msg, index) => {
    const hasMetadata = msg.metadata && msg.sender === 'bot';
    const hasSources = hasMetadata && msg.metadata.sources && msg.metadata.sources.length > 0;

    return (
      <Message
        key={index}
        model={{
          message: msg.message,
          sender: msg.sender,
          direction: msg.direction
        }}
        className={msg.isError ? styles.errorMessage : ''}
      >
        {msg.sender === 'bot' && (
  		<Avatar
  name="Bot"
  style={{
    background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    color: '#fff',
    fontSize: '1.4rem',
    fontWeight: 'bold',
    width: '42px',
    height: '42px',
    borderRadius: '50%',
    display: 'inline-flex',
    alignItems: 'center',
    justifyContent: 'center',
    boxShadow: '0 4px 10px rgba(0,0,0,0.15)',
    border: '2px solid rgba(255,255,255,0.4)',
    animation: 'pulse 3s infinite ease-in-out'
  }}
>
  <img src="https://cdn-icons-png.flaticon.com/512/4712/4712104.png" alt="Bot" style={{ width: '100%', height: '100%', objectFit: 'cover', borderRadius: '50%' }} />
</Avatar>
	)}
        {msg.sender === 'user' && (
  		<Avatar
  name="You"
  style={{
    background: 'linear-gradient(135deg, #ff9a9e 0%, #fad0c4 100%)',
    color: '#fff',
    fontSize: '1.4rem',
    fontWeight: 'bold',
    width: '42px',
    height: '42px',
    borderRadius: '50%',
    display: 'inline-flex',
    alignItems: 'center',
    justifyContent: 'center',
    boxShadow: '0 4px 10px rgba(0,0,0,0.15)',
    border: '2px solid rgba(255,255,255,0.4)',
    animation: 'pulse 3s infinite ease-in-out'
  }}
>
  <img src="https://upload.wikimedia.org/wikipedia/commons/9/99/Sample_User_Icon.png" alt="User" style={{ width: '100%', height: '100%', objectFit: 'cover', borderRadius: '50%' }} />
</Avatar>
	)}

        {/* Show sources if available (Normal RAG mode) */}
        {hasSources && (
          <Message.Footer>
            <div className={styles.sources}>
              <strong>📚 Sources:</strong>
              <ul>
                {msg.metadata.sources.slice(0, 3).map((source, idx) => (
                  <li key={idx}>
                    <em>{source.source.split('/').pop()}</em>
                    {source.section && ` - ${source.section}`}
                    {' '}(score: {(source.score * 100).toFixed(0)}%)
                  </li>
                ))}
              </ul>
            </div>
          </Message.Footer>
        )}

        {/* Show mode indicator for debugging (can be removed in production) */}
        {hasMetadata && msg.metadata.mode === 'selected_text' && (
          <Message.Footer>
            <div className={styles.modeIndicator}>
              💡 <em>Explaining selected text</em>
            </div>
          </Message.Footer>
        )}
      </Message>
    );
  };

  return (
    <div className={styles.chatContainer}>
      <MainContainer>
        <ChatContainer>
          <MessageList
            scrollBehavior="smooth"
            typingIndicator={isTyping ? <TypingIndicator content="Bot is thinking..." /> : null}
          >
            {messages.map((msg, idx) => renderMessage(msg, idx))}
          </MessageList>

          <MessageInput
            placeholder={
              selectedText
                ? `💡 Ask about the selected text: "${selectedText.substring(0, 30)}..."`
                : CHATBOT_CONFIG.placeholderText
            }
            onSend={handleSend}
            disabled={isTyping}
            attachButton={false}
            ref={inputRef}
          />
        </ChatContainer>
      </MainContainer>

      {/* Selected text indicator */}
      {selectedText && (
        <div className={styles.selectionHint}>
          <strong>💡 Text Selected:</strong> "{selectedText.substring(0, 50)}..."
          <br />
          <em>Ask me to explain it!</em>
        </div>
      )}
    </div>
  );
}
