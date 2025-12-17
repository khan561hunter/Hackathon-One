import React from 'react';
import Content from '@theme-original/Navbar/Content';
import AuthButton from '@site/src/components/Auth/AuthButton';
import LanguageToggle from '@site/src/components/LanguageToggle';

export default function ContentWrapper(props) {
  return (
    <>
      <Content {...props} />
      <div style={{ marginLeft: '16px', display: 'flex', alignItems: 'center', gap: '12px' }}>
        <LanguageToggle />
        <AuthButton />
      </div>
    </>
  );
}
