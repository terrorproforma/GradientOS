import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import tailwindcss from "@tailwindcss/vite";

export default defineConfig({
  plugins: [react(), tailwindcss()],
  server: {
    port: 8000,
    host: "0.0.0.0",
    allowedHosts: ["gradientrobotics.local", "jetson.local", "mini-arm.local"]
  },
  test: {
    environment: "jsdom",
  },
});
