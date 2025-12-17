import React from 'react';
import { useLanguage } from '../../contexts/LanguageContext';
import styles from './styles.module.css';

export default function LanguageToggle() {
  const { language, setLanguage } = useLanguage();

  const toggleLanguage = () => {
    const newLang = language === 'en' ? 'ur' : 'en';
    setLanguage(newLang);
  };

  return (
    <button
      onClick={toggleLanguage}
      className={styles.languageToggle}
      aria-label={`Switch to ${language === 'en' ? 'Urdu' : 'English'}`}
    >
      <span className={styles.icon}>🌐</span>
      <span className={styles.text}>
        {language === 'en' ? 'اردو' : 'English'}
      </span>
    </button>
  );
}
