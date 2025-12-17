import React, { createContext, useContext, useState, useEffect } from "react";

const LanguageContext = createContext(undefined);

export function LanguageProvider({ children }) {
  const [language, setLanguage] = useState("en");

  // Load language from localStorage on mount
  useEffect(() => {
    if (typeof window !== "undefined") {
      const saved = localStorage.getItem("language");
      if (saved === "ur" || saved === "en") {
        setLanguage(saved);
        // Apply RTL if Urdu
        if (saved === "ur") {
          document.documentElement.setAttribute("dir", "rtl");
        }
      }
    }
  }, []);

  // Save to localStorage and update RTL when language changes
  const changeLanguage = (newLang) => {
    if (newLang !== "en" && newLang !== "ur") return;

    setLanguage(newLang);

    if (typeof window !== "undefined") {
      localStorage.setItem("language", newLang);
    }

    // Update RTL direction
    if (newLang === "ur") {
      document.documentElement.setAttribute("dir", "rtl");
    } else {
      document.documentElement.setAttribute("dir", "ltr");
    }
  };

  const value = {
    language,
    setLanguage: changeLanguage,
    isUrdu: language === "ur",
    isEnglish: language === "en",
  };

  return (
    <LanguageContext.Provider value={value}>
      {children}
    </LanguageContext.Provider>
  );
}

export function useLanguage() {
  const context = useContext(LanguageContext);
  if (!context) {
    throw new Error("useLanguage must be used within LanguageProvider");
  }
  return context;
}
