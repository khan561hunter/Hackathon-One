import React from 'react';
import { AuthProvider } from '@site/src/contexts/AuthContext';
import { LanguageProvider } from '@site/src/contexts/LanguageContext';
import Chatbot from '@site/src/components/Chatbot';
import OAuthCallbackHandler from '@site/src/components/Auth/OAuthCallbackHandler';

export default function Root({children}) {
  return (
    <LanguageProvider>
      <AuthProvider>
        {children}
        <Chatbot />
        <OAuthCallbackHandler />
      </AuthProvider>
    </LanguageProvider>
  );
}
