---
description: "Primary project execution guide, skills catalog, and run commands."
alwaysApply: true
---

# AGENTS

This file was previously `QUICK_START.md` and is now the primary startup + execution guide.


### Agent workflow pointers

To keep implementation quality and session continuity high, maintain these repo-local files:

- `.cursor/memory/DEVLOG.md` - chronological engineering timeline (what changed, validation, risks). **MUST be updated for every meaningful task.**
- `.cursor/memory/AGENT_SCRATCHPAD.md` - persistent execution memory (mistakes, preferences, guardrails). **MUST be updated for every meaningful task.**
- `web-ui/design.md` - living UI consistency spec (typography hierarchy, panel rules, spacing, review checklist).
- `.cursor/skills/devlog-loop/SKILL.md` -> writes to `.cursor/memory/DEVLOG.md` (template: `.cursor/skills/devlog-loop/references/devlog-entry-template.md`).
- `.cursor/skills/learning-scratchpad-loop/SKILL.md` -> writes to `.cursor/memory/AGENT_SCRATCHPAD.md` (template: `.cursor/skills/learning-scratchpad-loop/references/scratchpad-template.md`).

Non-negotiable workflow rule:

- Never hand off or stop after implementation without appending both `.cursor/memory/DEVLOG.md` and `.cursor/memory/AGENT_SCRATCHPAD.md`.
- Treat missing updates to either file as an incomplete task (blocker), even if code is correct.

### Primary GradientOS architecture entrypoint

For any task touching the GradientOS control stack, proactively start with:

- `.cursor/skills/gradientos-sop/SKILL.md` - default routed entrypoint for controller, RTCore, EtherCAT, telemetry, API/frontend boundaries, commissioning, and config ownership work.
- `.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md` - long-form canonical SOP when architecture, safety, ownership, or workflow details matter.

When maintaining or consolidating the shared GradientOS skill corpus itself, also use:

- `.cursor/skills/gradientos-skill-maintainer/SKILL.md`

### Skills catalog (all installed) and when to use each

Use these skills proactively based on task type.

- `create-rule` (`.cursor/skills-cursor/create-rule/SKILL.md`) - creating or updating project rules under `.cursor/rules`, coding standards, or persistent guidance.
- `create-skill` (`.cursor/skills-cursor/create-skill/SKILL.md`) - authoring a new reusable Cursor skill (`SKILL.md`) with strong structure and triggers.
- `update-cursor-settings` (`.cursor/skills-cursor/update-cursor-settings/SKILL.md`) - changing editor/IDE settings, `settings.json`, formatting, autosave, or related config.
- `agent-browser` (`.cursor/skills/agent-browser/SKILL.md`) - browser automation/testing (navigate, click, fill, screenshot, extract data).
- `canvas-design` (`.cursor/skills/canvas-design/SKILL.md`) - generating static visual design artifacts (posters/artwork in `.png`/`.pdf`).
- `devlog-loop` (`.cursor/skills/devlog-loop/SKILL.md`) - meaningful work logging in `.cursor/memory/DEVLOG.md` for traceability and handoff.
- `find-skills` (`.cursor/skills/find-skills/SKILL.md`) - when user asks "is there a skill for X" or wants capability discovery/installation.
- `frontend-design` (`.cursor/skills/frontend-design/SKILL.md`) - UI building/polish for React/web interfaces where visual quality and production finish matter.
- `gradientos-sop` (`.cursor/skills/gradientos-sop/SKILL.md`) - primary GradientOS control-stack skill for controller, RTCore, EtherCAT, telemetry, API/frontend boundaries, commissioning, and config ownership. Use proactively for motion-stack work and read `.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md` as the long-form canonical source.
- `gradientos-skill-maintainer` (`.cursor/skills/gradientos-skill-maintainer/SKILL.md`) - maintains the shared GradientOS skill corpus and decides when scratchpad/devlog learnings are stable enough to consolidate into the canonical skill set.
- `learning-scratchpad-loop` (`.cursor/skills/learning-scratchpad-loop/SKILL.md`) - persistent execution memory updates in `.cursor/memory/AGENT_SCRATCHPAD.md`.
- `next-best-practices` (`.cursor/skills/next-best-practices/SKILL.md`) - Next.js architecture, routing, RSC boundaries, metadata, data handling.
- `next-cache-components` (`.cursor/skills/next-cache-components/SKILL.md`) - Next.js 16 caching strategy (`use cache`, cache tags/lifetimes, PPR patterns).
- `vercel-composition-patterns` (`.cursor/skills/vercel-composition-patterns/SKILL.md`) - React component API architecture and composition refactors at scale.
- `next-upgrade` (`.cursor/skills/next-upgrade/SKILL.md`) - migrating/upgrading Next.js versions safely with codemods and migration steps.
- `vercel-next-deploy` (`.cursor/skills/vercel-next-deploy/SKILL.md`) - Vercel deployment workflows, linking, env vars, preview/prod setup, domains.
- `vercel-react-best-practices` (`.cursor/skills/vercel-react-best-practices/SKILL.md`) - React/Next performance optimization and production-grade rendering patterns.
- `vercel-react-native-skills` (`.cursor/skills/vercel-react-native-skills/SKILL.md`) - React Native/Expo performance, animations, and native module patterns.
- `web-design-guidelines` (`.cursor/skills/web-design-guidelines/SKILL.md`) - design/accessibility/UX audit pass against web interface guideline checks.

### Design skill guidance for UI fixes in this repo

- For implementing visual/layout fixes: use `frontend-design`.
- For auditing UI quality/accessibility after implementation: use `web-design-guidelines`.
- For non-UI-code static design assets only: use `canvas-design`.
- For enforcing local visual consistency standards: follow `web-ui/design.md` and update it whenever UI conventions change.

### Initial setup

Run the setup script and select what modules you want to install.

```bash
./setup.sh
```

This will install the required system packages and create a virtual environment.
It targets Python 3.12 (preferred) for broad wheel compatibility.
If 3.12 is unavailable, setup falls back to 3.11, then 3.14.
Use a single repo-local virtualenv at `.venv` (the same one used by `start.sh`).
You can force-create the venv with 3.12:

```bash
uv venv .venv --python python3.12
source ./.venv/bin/activate
uv pip install -e .[core]
```

### EtherCAT on RevPi Connect 5 (important port note)

If you're bringing up the **RTOS/EtherCAT** path on a **RevPi Connect 5**, pay attention to which RJ45
port you use for the EtherCAT drive chain.

- **Both ports are “gigabit”, but they are different NICs/drivers**:
  - **`eth0`**: Linux driver **`macb`** (SoC MAC) → **use this for EtherCAT** (stable with IgH `ec_generic` in our bring-up)
  - **`eth1`**: Linux driver **`lan743x`** (PCI NIC) → use this for LAN/uplink
- **Why**: during bring-up we consistently saw slave discovery work on `macb` (`eth0`) and fail on `lan743x` (`eth1`)
  (Tx-only, Rx=0, 100% loss) despite Link=UP and tuning.

How to verify on the RevPi:

```bash
sudo ethtool -i eth0
sudo ethtool -i eth1
sudo ethercat master  # check Rx frames > 0 and Slaves > 0
```

Example output (shows SoC vs PCI NIC):

```bash
$ sudo ethtool -i eth0
driver: macb
bus-info: 1f00100000.ethernet

$ sudo ethtool -i eth1
driver: lan743x
bus-info: 0001:03:00.0
```

More details + IgH notes: `docs/ethercat/bringup.md` and `docs/ethercat/igh.md`.

### Run the components

All commands below automatically activate the venv
You can also activate manually if you want to run other scripts or tests

```bash
# Run controller, wait to confirm it connected to the arm
./run.sh

# Alternatively, if your servo burned out, you can use the built-in servo sim
./run-sim.sh

# Run the api
./run-api.sh

# Optionally, run the vision
./run-vision.sh

# Run the control UI (will be superseded by the web ui soon)
# to control the robot arm
gradient-ui

# Run the web ui
./run-web.sh

```

On Windows PowerShell, use the `.ps1` launcher variants (same behavior):

```powershell
# From repository root
.\.venv\Scripts\Activate.ps1

# Terminal 1: controller in simulator mode
.\run-sim.ps1

# Terminal 2: API service
.\run-api.ps1
```

If you want to invoke the existing bash launchers directly on Windows,
run them through a bash shell (`bash ./run-sim.sh` / `bash ./run-api.sh`) or use WSL/Git Bash.

The web UI should be available at http://localhost:8000

#### Controller alerts in the Web UI

- When the controller detects servo communication issues or status errors (for example: no SyncRead reply from certain servo IDs, or a servo status byte like 32), these are now forwarded to the API telemetry stream and shown in the web UI as alerts on the right side.
- Typical messages include a human-readable cause, e.g. “Servo 30 reported: Position Fault” or “No feedback from servos 30, 31 (SyncRead). Check power/wiring/baud.”
- If you do not see alerts but the terminal shows errors, ensure the API is running and the UI is connected to `/monitor`.

### Performance profiling

The loop benchmarking utilities now live under `scripts/`:

```bash
python scripts/performance_tester.py     # creates scripts/performance_log.csv
python scripts/chart_generator.py        # reads the CSV and saves scripts/performance_charts.png
```

### Distributed setup

Clone the repo on the different machines and run the setup script.
Then run only the specific components you want. Don't forget to use the command line arguments to point to the right ports and addresses for their dependencies.

# Manual setup

A quick checklist to bring up the development environment manually on Raspberry Pi or any Debian-based host. For detailed guides, see the documentation links at the end.

1. Install system packages (camera + OpenGL + libcap):

2. Create a dedicated Python virtual environment with `uv`:

    ```bash
    uv venv .venv
    ```

3. Activate the environment (required for the next steps and CLI aliases):

    ```bash
    source .venv/bin/activate
    # or: source ./start.sh  # adds project-specific PYTHONPATH/aliases
    ```

4. Install GradientOS (builds the IK solver extension and Python deps):
    ```bash
    uv pip install -e .
    ```

Optional extras:

-   Vision AI stack (YOLO + Torch): `uv pip install -e '.[ai]'`
-   Dataset tooling (LeRobot export helpers): `uv pip install -e '.[datasets]'`
-   Dev/test utilities (pytest, pre-commit): `uv pip install -e '.[dev]'`

Next steps:

-   UI setup & troubleshooting: `docs/UI_readme.md`
-   Vision module usage: `src/gradient_os/vision/README.md`
-   Full project docs & command references: `docs/README.md`


### TODO - New model takeover (high priority) - reqwrite this as necessary when the user requests you prepare a to do for a new  or fresh context model. 

Current UI layout still needs one more production pass.
User-reported issue: left drawer should align vertically with the right robot-control panel and not run to the edge.

What you must do:

1. Rework `web-ui/src/components/SidebarDrawer.tsx` vertical sizing so drawer uses consistent top/bottom insets (same visual margin style as right panel).
2. Keep long panel content fully scrollable inside the drawer body.
3. Validate with real weld content (`WELD PLANNING` + status badge + dense sections).
4. Confirm no regressions for STEP / Trajectory / Telemetry drawer panels.
5. Keep close control and header content non-overlapping at narrow and wide viewport sizes.

Acceptance criteria:

- Left drawer has a visible bottom inset and no longer appears to run to the viewport edge.
- No overlap between title, badges, and close button in left drawer.
- Header controls remain clickable and visible at narrow and wide window sizes.
- Drawer still respects viewport height with internal scrolling.
- `npm run build` passes after the change.
