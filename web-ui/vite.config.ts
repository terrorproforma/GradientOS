import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import tailwindcss from "@tailwindcss/vite";

const __dirname = dirname(fileURLToPath(import.meta.url));

export default defineConfig({
  plugins: [react(), tailwindcss()],
  define: {
    __GRADIENT_PUBLIC_DIR_FS__: JSON.stringify(resolve(__dirname, "public")),
  },
  build: {
    rollupOptions: {
      input: {
        app: resolve(__dirname, "index.html"),
        j6ManualRotateDataset: resolve(__dirname, "j6-manual-rotate-dataset.html"),
      },
    },
  },
  server: {
    port: 8000,
    host: "0.0.0.0",
    allowedHosts: ["gradientrobotics.local", "jetson.local", "mini-arm.local"]
  },
  test: {
    environment: "jsdom",
  },
});
