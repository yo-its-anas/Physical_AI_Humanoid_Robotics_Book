/**
 * RAG Chatbot Configuration
 *
 * This file contains environment-specific settings for the chatbot.
 * Update BACKEND_URL based on your deployment environment.
 */

// Detect environment and set backend URL accordingly
//const isDevelopment = process.env.NODE_ENV === 'development';
//const isProduction = typeof window !== 'undefined' && window.location.hostname !== 'localhost';

// Detect dev environment
const isDevelopment = typeof window !== 'undefined'
  ? window.location.hostname === 'localhost'
  : true;

// Backend URLs
export const BACKEND_URL = isDevelopment
  ? 'http://localhost:8000'
  : 'https://airobotixbook-cj8xap92x-yo-its-anas-projects.vercel.app/physical-ai'; // <-- update after deployment

// API endpoints
export const API_ENDPOINTS = {
  query: `${BACKEND_URL}/api/query`,
  health: `${BACKEND_URL}/api/health`,
};


export const CHATBOT_CONFIG = {
  // UI Configuration
  welcomeMessage: "Hi! I'm your AI Robotics assistant. Ask me anything about the textbook, or highlight text and ask me to explain it!",
  placeholderText: "Ask a question about robotics...",

  // Behavior Configuration
  enableSelectedTextMode: true,  // Enable/disable text selection feature
  maxMessageLength: 1000,        // Maximum characters per message

  // Performance Configuration
  requestTimeout: 30000,         // 30 seconds timeout for API calls

  // Error Messages
  errors: {
    network: "Network error. Please check your connection and try again.",
    timeout: "Request timed out. Please try again.",
    rateLimit: "Too many requests. Please wait a moment and try again.",
    noResults: "I couldn't find relevant information in the textbook for your question.",
    generic: "An error occurred. Please try again."
  }
};
