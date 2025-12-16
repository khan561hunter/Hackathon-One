import React from 'react';
import { AuthProvider } from '@site/src/contexts/AuthContext';
import Chatbot from '@site/src/components/Chatbot';
import OAuthCallbackHandler from '@site/src/components/Auth/OAuthCallbackHandler';

export default function Root({children}) {
  return (
    <AuthProvider>
      {children}
      <Chatbot />
      <OAuthCallbackHandler />
    </AuthProvider>
  );
}
