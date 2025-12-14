import React, { createContext, useContext } from "react";
import { useSession, signIn, signUp, signOut } from "../lib/auth-client";

const AuthContext = createContext(undefined);

export function AuthProvider({ children }) {
  const { data: session, isPending: isLoading, error, refetch } = useSession();

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
