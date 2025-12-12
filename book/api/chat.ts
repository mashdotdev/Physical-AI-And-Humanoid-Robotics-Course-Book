/**
 * Vercel serverless function to proxy chat requests
 * This forwards cookies from the browser to the backend
 */
import type { VercelRequest, VercelResponse } from "@vercel/node";

const BACKEND_URL = "https://physical-ai-and-humanoid-robotics-c-iota.vercel.app";

export default async function handler(req: VercelRequest, res: VercelResponse) {
  // Debug: Log that this function was called
  console.log("[Chat Proxy] Function called, method:", req.method);
  console.log("[Chat Proxy] Cookies received:", req.headers.cookie ? "YES" : "NO");
  console.log("[Chat Proxy] Cookie value:", req.headers.cookie?.substring(0, 50) + "...");

  // Only allow POST
  if (req.method !== "POST") {
    return res.status(405).json({ error: "Method not allowed" });
  }

  try {
    // Forward the request to the backend, including cookies
    const response = await fetch(`${BACKEND_URL}/api/chat`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        // Forward the cookie header from the browser
        Cookie: req.headers.cookie || "",
      },
      body: JSON.stringify(req.body),
    });

    const data = await response.json();

    console.log("[Chat Proxy] Backend response status:", response.status);

    // If auth error, add debug info
    if (response.status === 401) {
      return res.status(401).json({
        ...data,
        _debug: {
          proxyRecevedCookie: !!req.headers.cookie,
          cookieNames: req.headers.cookie
            ? req.headers.cookie.split(';').map(c => c.trim().split('=')[0])
            : [],
        }
      });
    }

    // Return the same status and data
    return res.status(response.status).json(data);
  } catch (error) {
    console.error("[Chat Proxy] Error:", error);
    return res.status(500).json({ error: "Proxy error", message: String(error) });
  }
}
