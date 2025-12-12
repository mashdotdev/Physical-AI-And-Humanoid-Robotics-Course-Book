/**
 * Test endpoint to verify serverless functions work
 */
import type { VercelRequest, VercelResponse } from "@vercel/node";

export default function handler(req: VercelRequest, res: VercelResponse) {
  return res.status(200).json({
    message: "Serverless function is working!",
    cookies: req.headers.cookie ? req.headers.cookie.split(';').map(c => c.trim().split('=')[0]) : [],
    hasCookie: !!req.headers.cookie,
    timestamp: new Date().toISOString(),
  });
}
