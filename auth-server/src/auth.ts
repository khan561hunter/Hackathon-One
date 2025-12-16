import { betterAuth } from "better-auth";
import { Pool } from "pg";
import "dotenv/config";

// Create PostgreSQL connection pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false,
  },
});

export const auth = betterAuth({
  // Database configuration - Neon Postgres
  database: pool,

  // Base URL of the auth server
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3001",

  // Secret for signing cookies/tokens (REQUIRED - 32+ characters)
  secret: process.env.BETTER_AUTH_SECRET,

  // Email and password authentication
  emailAndPassword: {
    enabled: true,
    // Minimum password length
    minPasswordLength: 8,
  },

  // Social OAuth providers
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID!,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET!,
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID!,
      clientSecret: process.env.GITHUB_CLIENT_SECRET!,
    },
  },

  // Trusted origins for CORS
  trustedOrigins: (process.env.CORS_ORIGINS || "http://localhost:3000")
    .split(",")
    .map((origin) => origin.trim()),

  // Advanced cookie configuration for cross-origin
  advanced: {
    // Cookie settings for cross-origin requests
    defaultCookieAttributes: {
      // Use 'none' for cross-domain cookies to work
      sameSite: "none" as const,
      secure: true,
      httpOnly: true,
      path: "/",
    },
    // Use secure cookies in production
    useSecureCookies: true,
  },

  // Session configuration
  session: {
    // Cookie expiry (7 days)
    expiresIn: 60 * 60 * 24 * 7, // 7 days in seconds
    // Update session on activity
    updateAge: 60 * 60 * 24, // Update every 24 hours
  },

  // User configuration
  user: {
    // Additional fields to store
    additionalFields: {
      // You can add custom fields here if needed
    },
  },
});

export type Session = typeof auth.$Infer.Session;
export type User = typeof auth.$Infer.Session.user;
