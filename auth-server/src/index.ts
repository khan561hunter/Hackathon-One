import express, { Request, Response } from "express";
import cors from "cors";
import { toNodeHandler } from "better-auth/node";
import { auth } from "./auth.js";
import "dotenv/config";

const app = express();
const PORT = process.env.PORT || 3001;

// Parse CORS origins from environment (trim whitespace from each origin)
const corsOrigins = (process.env.CORS_ORIGINS || "http://localhost:3000")
  .split(",")
  .map((origin) => origin.trim());

console.log("CORS Origins configured:", corsOrigins);

// CORS configuration - MUST be before routes
app.use(
  cors({
    origin: (origin, callback) => {
      // Allow requests with no origin (mobile apps, curl, etc.)
      if (!origin) return callback(null, true);

      if (corsOrigins.includes(origin)) {
        callback(null, true);
      } else {
        console.log("CORS blocked origin:", origin);
        callback(null, false);
      }
    },
    credentials: true, // CRITICAL: Allow cookies
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allowedHeaders: ["Content-Type", "Authorization"],
  })
);

// BetterAuth handler - handles all /api/auth/* routes
// This MUST come before express.json() middleware
app.all("/api/auth/*", toNodeHandler(auth));

// JSON middleware AFTER auth handler
app.use(express.json());

// Health check endpoint
app.get("/health", (_req: Request, res: Response) => {
  res.json({
    status: "ok",
    service: "auth-server",
    timestamp: new Date().toISOString(),
  });
});

// Root endpoint
app.get("/", (_req: Request, res: Response) => {
  res.json({
    name: "BetterAuth Server",
    version: "1.0.0",
    endpoints: {
      health: "/health",
      auth: "/api/auth/*",
    },
  });
});

// Start server
app.listen(PORT, () => {
  console.log(`
╔════════════════════════════════════════════════════════╗
║           BetterAuth Server Started                    ║
╠════════════════════════════════════════════════════════╣
║  URL:     http://localhost:${PORT}                       ║
║  Auth:    http://localhost:${PORT}/api/auth/*            ║
║  Health:  http://localhost:${PORT}/health                ║
╠════════════════════════════════════════════════════════╣
║  CORS Origins: ${corsOrigins.join(", ").padEnd(38)}║
╚════════════════════════════════════════════════════════╝
  `);
});
