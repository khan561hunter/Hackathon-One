import React from 'react';
import Content from '@theme-original/Navbar/Content';
import AuthButton from '@site/src/components/Auth/AuthButton';

export default function ContentWrapper(props) {
  return (
    <>
      <Content {...props} />
      <div style={{ marginLeft: '16px', display: 'flex', alignItems: 'center', gap: '12px' }}>
        <AuthButton />
      </div>
    </>
  );
}
