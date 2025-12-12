/**
 * Better Auth server entry point
 * Handles all authentication requests via HTTP
 */
import { toNodeHandler } from "better-auth/node";
import { createServer, IncomingMessage, ServerResponse } from "http";
import { auth } from "./auth";
import "dotenv/config";

const PORT = parseInt(process.env.PORT || "3000", 10);

// Create HTTP handler from Better Auth instance
const authHandler = toNodeHandler(auth);

// Create HTTP server
const server = createServer(async (req: IncomingMessage, res: ServerResponse) => {
  // Handle CORS preflight requests
  if (req.method === "OPTIONS") {
    const origin = req.headers.origin || "*";
    res.setHeader("Access-Control-Allow-Origin", origin);
    res.setHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    res.setHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
    res.setHeader("Access-Control-Allow-Credentials", "true");
    res.setHeader("Access-Control-Max-Age", "86400");
    res.statusCode = 204;
    res.end();
    return;
  }

  // Add CORS headers to all responses
  const origin = req.headers.origin || "*";
  res.setHeader("Access-Control-Allow-Origin", origin);
  res.setHeader("Access-Control-Allow-Credentials", "true");

  // Health check endpoint
  if (req.url === "/health" && req.method === "GET") {
    res.setHeader("Content-Type", "application/json");
    res.statusCode = 200;
    res.end(JSON.stringify({ status: "healthy", service: "auth" }));
    return;
  }

  // Delegate all other requests to Better Auth handler
  try {
    await authHandler(req, res);
  } catch (error) {
    console.error("Auth handler error:", error);
    res.statusCode = 500;
    res.setHeader("Content-Type", "application/json");
    res.end(JSON.stringify({ error: "Internal server error" }));
  }
});

// Start server
server.listen(PORT, () => {
  console.log(`Auth server running on http://localhost:${PORT}`);
  console.log(`Health check: http://localhost:${PORT}/health`);
  console.log(`Environment: ${process.env.NODE_ENV || "development"}`);
});

// Graceful shutdown
process.on("SIGTERM", () => {
  console.log("SIGTERM received, shutting down...");
  server.close(() => {
    console.log("Server closed");
    process.exit(0);
  });
});

// Export for Vercel serverless
export default authHandler;
