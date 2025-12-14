import React from 'react';
import { AuthProvider } from '@site/src/contexts/AuthContext';
import Chatbot from '@site/src/components/Chatbot';

export default function Root({children}) {
  return (
    <AuthProvider>
      {children}
      <Chatbot />
    </AuthProvider>
  );
}
