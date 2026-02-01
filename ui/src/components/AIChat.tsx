/**
 * AI Chat component for slicer settings
 */
import { useChat } from '@ai-sdk/react';
import { Button } from './ui/button';
import { Send } from 'lucide-react';

export const AIChat = () => {
  const { messages, input, handleInputChange, handleSubmit, isLoading } = useChat({
    api: '/api/chat',
  });

  return (
    <div className="ai-chat-container">
      {/* Messages Area */}
      <div className="ai-messages">
        {messages.length === 0 ? (
          <div className="ai-empty-state">
            <h3>AI Slicer Assistant</h3>
            <p>Ask me to configure your print settings</p>
            <div className="ai-examples">
              <p className="example-label">Try asking:</p>
              <div className="example-chips">
                <button 
                  type="button"
                  className="example-chip"
                  onClick={() => {
                    handleInputChange({ target: { value: 'Make it strong and durable' } } as React.ChangeEvent<HTMLInputElement>);
                  }}
                >
                  "Make it strong and durable"
                </button>
                <button 
                  type="button"
                  className="example-chip"
                  onClick={() => {
                    handleInputChange({ target: { value: 'Fast draft print with minimal infill' } } as React.ChangeEvent<HTMLInputElement>);
                  }}
                >
                  "Fast draft print"
                </button>
                <button 
                  type="button"
                  className="example-chip"
                  onClick={() => {
                    handleInputChange({ target: { value: 'High quality print with fine details' } } as React.ChangeEvent<HTMLInputElement>);
                  }}
                >
                  "High quality print"
                </button>
              </div>
            </div>
          </div>
        ) : (
          <>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`ai-message ${message.role === 'user' ? 'ai-message-user' : 'ai-message-assistant'}`}
              >
                <div className="ai-message-content">
                  {message.content}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className="ai-message ai-message-assistant">
                <div className="ai-message-content ai-typing">
                  <span />
                  <span />
                  <span />
                </div>
              </div>
            )}
          </>
        )}
      </div>

      {/* Input Area */}
      <form onSubmit={handleSubmit} className="ai-input-form">
        <input
          type="text"
          value={input || ''}
          onChange={handleInputChange}
          placeholder="Describe your print requirements..."
          className="ai-input"
          disabled={isLoading}
        />
        <Button 
          type="submit" 
          size="sm" 
          disabled={!input || !input.trim() || isLoading}
          className="ai-submit-button"
        >
          <Send className="h-4 w-4" />
        </Button>
      </form>
    </div>
  );
};
