import { useLanguage } from "../contexts/LanguageContext";
import enTranslations from "../translations/en.json";
import urTranslations from "../translations/ur.json";

const translations = {
  en: enTranslations,
  ur: urTranslations,
};

export function useTranslation() {
  const { language } = useLanguage();

  const t = (key) => {
    const keys = key.split(".");
    let value = translations[language];

    // Navigate through nested object
    for (const k of keys) {
      if (value && typeof value === "object" && k in value) {
        value = value[k];
      } else {
        // Fallback to English if key not found
        value = translations.en;
        for (const fallbackKey of keys) {
          if (value && typeof value === "object" && fallbackKey in value) {
            value = value[fallbackKey];
          } else {
            return key; // Return key if translation missing
          }
        }
        break;
      }
    }

    return value;
  };

  return { t, language };
}
