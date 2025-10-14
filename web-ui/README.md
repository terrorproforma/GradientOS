# GradientOS Web UI

Early React/TypeScript frontend for the GradientOS API. It connects to the
`/monitor` SSE endpoint and shows the raw streaming telemetry; later we can add
controls and 3D visualization with `@react-three/fiber`.

## Prerequisites

- Node.js 18+ (recommended) and npm, pnpm, or yarn
- The GradientOS API (`gradient-api`) running locally or accessible over the
  network

## Install

```bash
cd web-ui
npm install
```

## Scripts

- `npm run dev` – Start the Vite dev server with hot reload
- `npm run build` – Production build (Tailwind CSS v4 + React)
- `npm run preview` – Serve the production build locally for testing

## Usage

1. Ensure `gradient-api` is running (e.g. `gradient-api --dev`).
2. Start the dev server: `npm run dev`.
3. Open the printed URL (default `http://localhost:4000`). The UI auto-fills the
   API host to the same origin on port 8000 (e.g. `http://192.168.2.140:8000`
   when you browse from another machine). Click **Connect** to subscribe to
   telemetry or change the host if you need a different endpoint.

The page will list the latest joint angles/gripper value and maintain an event
log. Tailwind CSS v4 powers the styling, so utility classes are available
directly in the JSX. Future iterations will replace the text view with a 3D
scene built on `@react-three/fiber`.
