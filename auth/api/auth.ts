/**
 * Vercel serverless function for Better Auth
 * All auth routes are handled by this single endpoint
 */
import { toNodeHandler } from "better-auth/node";
import { auth } from "../src/auth";
import type { IncomingMessage, ServerResponse } from "http";

const handler = toNodeHandler(auth);

export default async function (req: IncomingMessage, res: ServerResponse) {
  // Handle CORS preflight
  const origin = req.headers.origin || "*";
  res.setHeader("Access-Control-Allow-Origin", origin);
  res.setHeader("Access-Control-Allow-Credentials", "true");

  if (req.method === "OPTIONS") {
    res.setHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    res.setHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
    res.setHeader("Access-Control-Max-Age", "86400");
    res.statusCode = 204;
    res.end();
    return;
  }

  // Health check
  if (req.url === "/health") {
    res.setHeader("Content-Type", "application/json");
    res.statusCode = 200;
    res.end(JSON.stringify({ status: "healthy" }));
    return;
  }

  return handler(req, res);
}
