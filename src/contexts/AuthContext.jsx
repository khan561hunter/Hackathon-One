import React, { createContext, useContext, useState, useEffect } from "react";
import { useSession, signIn, signUp, signOut } from "../lib/auth-client";

const AuthContext = createContext(undefined);

// Keys for storing data in localStorage
const INTENDED_ROUTE_KEY = "auth_intended_route";
const USER_PROGRESS_KEY_PREFIX = "user_progress_";
const USER_PROFILE_KEY_PREFIX = "user_profile_";

// Helper to get user-specific keys
const getUserProgressKey = (userId) => `${USER_PROGRESS_KEY_PREFIX}${userId}`;
const getUserProfileKey = (userId) => `${USER_PROFILE_KEY_PREFIX}${userId}`;

export function AuthProvider({ children }) {
  const { data: session, isPending: isLoading, error, refetch } = useSession();
  const [intendedRoute, setIntendedRoute] = useState(null);
  const [userProgress, setUserProgress] = useState(null);
  const [userProfile, setUserProfile] = useState(null);
  const [profileLoaded, setProfileLoaded] = useState(false);

  // Load intended route, user progress, and profile from localStorage when user changes
  useEffect(() => {
    if (typeof window !== "undefined") {
      const storedRoute = localStorage.getItem(INTENDED_ROUTE_KEY);
      if (storedRoute) {
        setIntendedRoute(storedRoute);
      }
    }
  }, []);

  // Load user-specific data when session changes
  useEffect(() => {
    if (!session?.user?.id) {
      // No user logged in - clear profile and progress
      setUserProfile(null);
      setUserProgress(null);
      setProfileLoaded(false);
      return;
    }

    const userId = session.user.id;

    if (typeof window !== "undefined") {
      // Load user-specific progress
      const progressKey = getUserProgressKey(userId);
      const storedProgress = localStorage.getItem(progressKey);
      if (storedProgress) {
        try {
          setUserProgress(JSON.parse(storedProgress));
        } catch (e) {
          console.error("Failed to parse user progress:", e);
        }
      }

      // Load user-specific profile
      const profileKey = getUserProfileKey(userId);
      const storedProfile = localStorage.getItem(profileKey);
      if (storedProfile) {
        try {
          const parsed = JSON.parse(storedProfile);
          setUserProfile(parsed);
        } catch (e) {
          console.error("Failed to parse user profile:", e);
        }
      } else {
        setUserProfile(null);
      }

      // Mark profile as loaded (either found or confirmed doesn't exist)
      setProfileLoaded(true);
    }
  }, [session?.user?.id]);

  // Save user progress whenever it changes (user-specific)
  useEffect(() => {
    if (typeof window !== "undefined" && userProgress && session?.user?.id) {
      const progressKey = getUserProgressKey(session.user.id);
      localStorage.setItem(progressKey, JSON.stringify(userProgress));
    }
  }, [userProgress, session?.user?.id]);

  // Save user profile whenever it changes (user-specific)
  useEffect(() => {
    if (typeof window !== "undefined" && userProfile && session?.user?.id) {
      const profileKey = getUserProfileKey(session.user.id);
      localStorage.setItem(profileKey, JSON.stringify(userProfile));
    }
  }, [userProfile, session?.user?.id]);

  // Function to store intended route before authentication
  const storeIntendedRoute = (route) => {
    if (typeof window !== "undefined") {
      localStorage.setItem(INTENDED_ROUTE_KEY, route);
      setIntendedRoute(route);
    }
  };

  // Function to get and clear intended route after successful auth
  const getAndClearIntendedRoute = () => {
    if (typeof window !== "undefined") {
      const route = localStorage.getItem(INTENDED_ROUTE_KEY);
      localStorage.removeItem(INTENDED_ROUTE_KEY);
      setIntendedRoute(null);
      return route;
    }
    return null;
  };

  // Track page visit for user progress
  const trackPageVisit = (pageId, pageTitle) => {
    if (!session?.user) return;

    const newProgress = {
      ...userProgress,
      lastVisited: {
        pageId,
        pageTitle,
        timestamp: new Date().toISOString(),
      },
      visitedPages: {
        ...userProgress?.visitedPages,
        [pageId]: {
          title: pageTitle,
          lastVisit: new Date().toISOString(),
          visits: (userProgress?.visitedPages?.[pageId]?.visits || 0) + 1,
        },
      },
    };
    setUserProgress(newProgress);
  };

  // Update user profile
  const updateUserProfile = async (profileData) => {
    const newProfile = {
      ...userProfile,
      ...profileData,
      updatedAt: new Date().toISOString(),
    };
    setUserProfile(newProfile);
    return newProfile;
  };

  const value = {
    user: session?.user ?? null,
    session: session ?? null,
    isLoading,
    isAuthenticated: !!session?.user,
    error,
    signIn,
    signUp,
    signOut,
    refetch,
    // Redirect management
    intendedRoute,
    storeIntendedRoute,
    getAndClearIntendedRoute,
    // User progress tracking
    userProgress,
    trackPageVisit,
    // User profile
    userProfile,
    profileLoaded,
    updateUserProfile,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error("useAuth must be used within an AuthProvider");
  }
  return context;
}
