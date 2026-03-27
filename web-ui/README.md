# GradientOS Web UI

Production-facing React/TypeScript operator interface for GradientOS.

The app provides:

- live arm visualization in a 3D scene
- telemetry and controller alerts
- controller runtime mode hot switching between `LIVE` and `SIM`
- trajectory preview and execution controls
- STEP topology loading and weld planning tools
- Program Tree inspection/editing for planned motion
- global Tool Library management (create/save/select end effectors)

## Prerequisites

- Node.js 18+ and npm
- `gradient-api` running and reachable (default `http://localhost:4000`)
- controller/simulator running if you want live execution

## Install

```bash
cd web-ui
npm install
```

## Scripts

- `npm run dev` - start Vite dev server (default port `8000`)
- `npm run build` - production build
- `npm run preview` - serve the production build locally
- `npm run sync:assets` - sync robot and tool assets into `public/assets`
- `npm run sync:robot-assets` - sync robot URDF/mesh assets
- `npm run sync:tool-assets` - sync tool mesh assets

## Run (Local)

1. Start backend services from repo root:
   - `./run-sim.sh` and `./run-api.sh` (or PowerShell `.ps1` variants)
   - These launchers now bootstrap the repo environment first (`start.sh` semantics on bash, `.venv\Scripts\Activate.ps1` on PowerShell) before starting Python modules.
2. Start UI:

```bash
cd web-ui
npm run dev
```

3. Open `http://localhost:8000`
4. Confirm API host, then click **Connect**

## Feature Overview

### Scene + Telemetry

- 3D arm, workcell overlays, and path rendering
- SSE telemetry via `/monitor`
- weld-active and alert overlays

### Trajectory Planning

- point-based preview planning
- execute planned preview
- recorded trajectory load/run flows

### Weld Planning

- STEP topology edge selection
- per-segment weld configuration
- work/travel angles and transition clearance
- post-actions (`none`, `lift`, `return_to_start`)
- work/travel inputs interpreted as desired torch angles, compensated by active tool definition

### Tool Library

- filter tools by robot compatibility, tool type, and keyword
- create/update/delete tool definitions (XYZ mm + RPY deg)
- stage desired active tool in runtime policy and apply it live without restarting the controller
- optional STL mesh preview in the visualizer (fallback marker is shown when no mesh is provided)
- tools are auto-discovered from `tools/library/<tool_id>/tool.json` with per-tool local asset files
- quick load/select flow is available directly in the left sidebar `Tool Library` drawer
- full tool parameter editing is isolated under the `Tool Library` tab in Settings
- mesh placement can be defined separately from TCP (`mesh.position_mm` + `mesh.rotation_deg`, relative to J6/flange anchor) to decouple visual mesh origin from tool-tip offset

### Runtime Mode

- the header `LIVE` / `SIM` toggle now hot-switches the controller runtime immediately after operator confirmation
- the switch stops active motion, waits for idle, and swaps backends in-process through the controller
- restart actions remain only for real restart-bound runtime policy changes such as robot/backend/profile differences

### Program Tree

- exact execution path sample inspection (no low-resolution trim view)
- control-point editing from Program Tree context
- apply edits back into planner flow

## Operational Notes

- Weld run flow refreshes preview planning before execution to capture current pre-run robot state.
- `return_to_start` targets the trajectory start pose captured at run time.
- Runtime execution uses stability guards to avoid sub-step state races during weld playback.

## Build Check

Before merging UI changes, run:

```bash
npm run build
```
