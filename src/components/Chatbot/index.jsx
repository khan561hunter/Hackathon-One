/**
 * RAG Chatbot Component for Docusaurus
 *
 * Features:
 * - Ask questions using RAG (retrieval from Qdrant + Claude generation)
 * - Ask questions about selected text (direct Claude generation)
 * - Floating widget with expandable chat window
 * - Dark mode friendly
 */

import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

// Use window.location.origin to determine backend URL
// In development: http://localhost:8000
// In production: configure via docusaurus.config.js customFields
const BACKEND_URL = typeof window !== 'undefined' && window.location.hostname === 'localhost'
  ? 'http://localhost:8000'
  : 'https://38aaaf1fe32c.ngrok-free.app'; // ngrok tunnel URL (temporary)

export default function Chatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      role: 'assistant',
      content: 'Hi! 👋 I can answer questions about Physical AI and Humanoid Robotics.\n\n**Try asking:**\n- What is Physical AI?\n- How does ROS2 work?\n- Tell me about VLA models\n- What is the hardware setup?\n\nOr highlight any text and click "Ask About Selection" to ask about specific content!',
      timestamp: new Date(),
      showSuggestions: true
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [showSelectionButton, setShowSelectionButton] = useState(false);
  const [selectionMode, setSelectionMode] = useState(false);
  const [showSuggestions, setShowSuggestions] = useState(false);

  const messagesEndRef = useRef(null);
  const chatContainerRef = useRef(null);

  // Suggested questions for quick access
  const suggestedQuestions = [
    "What is Physical AI?",
    "How does ROS2 work?",
    "Tell me about digital twins",
    "What is the VLA capstone project?",
    "How do I set up the hardware?",
    "Explain humanoid navigation"
  ];

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text.length > 10) {
        setSelectedText(text);
        setShowSelectionButton(true);
      } else {
        setShowSelectionButton(false);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
    };
  }, []);

  const addMessage = (role, content, sources = null) => {
    setMessages(prev => [...prev, {
      role,
      content,
      sources,
      timestamp: new Date()
    }]);
  };

  const sendMessage = async (question, useSelection = false) => {
    if (!question.trim()) return;

    // Add user message
    addMessage('user', question);
    setInputValue('');
    setIsLoading(true);

    try {
      let response;

      if (useSelection && selectedText) {
        // Use /ask-selected endpoint
        response = await fetch(`${BACKEND_URL}/ask-selected`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            selected_text: selectedText,
            question: question,
            stream: false
          })
        });
      } else {
        // Use /ask endpoint (RAG)
        response = await fetch(`${BACKEND_URL}/ask`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            question: question,
            stream: false
          })
        });
      }

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Add assistant response
      addMessage('assistant', data.answer, data.sources || null);

      // Reset selection mode
      if (useSelection) {
        setSelectionMode(false);
        setShowSelectionButton(false);
        setSelectedText('');
      }

    } catch (error) {
      console.error('Error sending message:', error);
      addMessage('assistant', `Sorry, I encountered an error: ${error.message}`);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    if (inputValue.trim() && !isLoading) {
      sendMessage(inputValue, selectionMode);
    }
  };

  const handleAskAboutSelection = () => {
    if (!selectedText) return;

    setIsOpen(true);
    setSelectionMode(true);
    setInputValue(`About the selected text: `);
    setShowSelectionButton(false);
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const formatTimestamp = (date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <>
      {/* Selection Button */}
      {showSelectionButton && !isOpen && (
        <div className={styles.selectionButton}>
          <button onClick={handleAskAboutSelection}>
            💬 Ask About Selection
          </button>
        </div>
      )}

      {/* Floating Chat Widget */}
      <div className={`${styles.chatWidget} ${isOpen ? styles.open : ''}`}>
        {/* Toggle Button */}
        {!isOpen && (
          <button
            className={styles.toggleButton}
            onClick={toggleChat}
            aria-label="Open chat"
          >
            💬
          </button>
        )}

        {/* Chat Window */}
        {isOpen && (
          <div className={styles.chatWindow}>
            {/* Header */}
            <div className={styles.chatHeader}>
              <h3>📚 Documentation Assistant</h3>
              <button
                className={styles.closeButton}
                onClick={toggleChat}
                aria-label="Close chat"
              >
                ✕
              </button>
            </div>

            {/* Messages */}
            <div className={styles.chatMessages} ref={chatContainerRef}>
              {messages.map((message, index) => (
                <div
                  key={index}
                  className={`${styles.message} ${styles[message.role]}`}
                >
                  <div className={styles.messageContent}>
                    {message.content}
                  </div>

                  {/* Sources (if available) */}
                  {message.sources && message.sources.length > 0 && (
                    <div className={styles.sources}>
                      <strong>Sources:</strong>
                      <ul>
                        {message.sources.map((source, idx) => (
                          <li key={idx}>
                            {source.doc_name} (score: {source.score.toFixed(3)})
                          </li>
                        ))}
                      </ul>
                    </div>
                  )}

                  <div className={styles.timestamp}>
                    {formatTimestamp(message.timestamp)}
                  </div>
                </div>
              ))}

              {isLoading && (
                <div className={`${styles.message} ${styles.assistant}`}>
                  <div className={styles.messageContent}>
                    <div className={styles.typingIndicator}>
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                  </div>
                </div>
              )}

              <div ref={messagesEndRef} />
            </div>

            {/* Suggested Questions - Always available, collapsible */}
            <div className={styles.suggestedQuestionsContainer}>
              <button
                className={styles.suggestionsToggle}
                onClick={() => setShowSuggestions(!showSuggestions)}
                type="button"
              >
                <span className={styles.toggleIcon}>{showSuggestions ? '▼' : '▲'}</span>
                Quick Questions
                <span className={styles.toggleBadge}>{suggestedQuestions.length}</span>
              </button>

              {showSuggestions && (
                <div className={styles.suggestedQuestions}>
                  <div className={styles.suggestionsGrid}>
                    {suggestedQuestions.map((question, idx) => (
                      <button
                        key={idx}
                        className={styles.suggestionButton}
                        onClick={() => {
                          sendMessage(question, false);
                          setShowSuggestions(false);
                        }}
                        type="button"
                      >
                        {question}
                      </button>
                    ))}
                  </div>
                </div>
              )}
            </div>

            {/* Selection Mode Indicator */}
            {selectionMode && (
              <div className={styles.selectionModeIndicator}>
                ✨ Asking about selected text ({selectedText.length} chars)
                <button
                  onClick={() => {
                    setSelectionMode(false);
                    setInputValue('');
                  }}
                >
                  ✕
                </button>
              </div>
            )}

            {/* Input Form */}
            <form className={styles.chatInput} onSubmit={handleSubmit}>
              <input
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                placeholder={
                  selectionMode
                    ? "Ask about the selected text..."
                    : "Ask a question..."
                }
                disabled={isLoading}
              />
              <button
                type="submit"
                disabled={isLoading || !inputValue.trim()}
              >
                {isLoading ? '...' : '→'}
              </button>
            </form>
          </div>
        )}
      </div>
    </>
  );
}
