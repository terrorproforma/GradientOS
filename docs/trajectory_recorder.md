# Trajectory Authoring And Recorder

> **Status:** Current workflow supports both the legacy recorder op-codes and the newer waypoint-program editor used by the web UI.
>
> The system now has two complementary ways to create robot motion:
>
> 1. **Recorder flow** for quick teach-by-demonstration capture.
> 2. **Waypoint authoring flow** for editable pose waypoints, IK-planned interpolation, saved program records, and explicit SIM-vs-LIVE execution.

---

## Two Authoring Modes
### 1. Recorder flow
Teaching by demonstration is still useful when you want to jog the robot manually and snapshot poses in sequence. With the recorder op-codes you can:

1. **Start** a recording session (`PLAN_TRAJECTORY`).
2. **Move** the robot with any normal command (`MOVE_LINE`, `SET_ORIENTATION`, …).
3. **Drop** poses at interesting points (`REC_POS`).
4. **Finish** and save to disk (`END_TRAJECTORY,<name>`).

The result is a human-readable runnable JSON file in `recorded_trajectories/` that you can replay with a single `RUN_TRAJECTORY` command.

### 2. Waypoint program flow
The web trajectory editor is the main production authoring path. It lets you:

1. Create pose waypoints from workspace clicks, numeric edits, or `Capture Pose`.
2. Store both **position** and **orientation** per waypoint.
3. Re-plan the whole sequence through the existing IK / trajectory planner.
4. Save the editable source program separately from the runnable trajectory artifact.
5. Load the editable program later and keep the program tree plus 3D preview in sync.
6. Execute with an explicit intent:
   - `Simulate Trajectory` when the controller runtime is in `SIM`
   - `Run Trajectory` when the controller runtime is in `LIVE`
7. Switch the controller between `SIM` and `LIVE` directly from the header toggle or via `POST /control/runtime-mode` when the wrong runtime is active.

---

## Legacy Recorder Commands

| Command | Syntax | Blocking? | Description |
|---------|--------|-----------|-------------|
| `PLAN_TRAJECTORY` | `PLAN_TRAJECTORY` | ✔ (instant) | Arms the recorder.  Subsequent `REC_POS` calls will be buffered until `END_TRAJECTORY`. |
| `REC_POS` | `REC_POS` | ✔ (instant) | Calculates Forward Kinematics for the current joint state and appends a way-point to the buffer. |
| `END_TRAJECTORY` | `END_TRAJECTORY,<name>` | ✔ (instant) | Writes the buffered way-points to `recorded_trajectories/<name>.json` and disables recorder mode. |

All **other** commands remain fully functional while recording—this is key.  You can build complex motions interactively and decide *when* to snapshot poses.

### Command Flow Example
```text
PLAN_TRAJECTORY              # Enable recording
MOVE_LINE_RELATIVE,0,0,-0.1  # Jog down
REC_POS                      # Way-point #1
SET_ORIENTATION,0,30,0       # Tilt tool head
REC_POS                      # Way-point #2
MOVE_LINE_RELATIVE,0.05,0,0  # Small lateral move
REC_POS                      # Way-point #3
END_TRAJECTORY,my_pick_place # Save & exit recorder
```

Play it back:
```text
RUN_TRAJECTORY,my_pick_place,true  # Use cache for faster start-up
```

---

## Waypoint Planning And Saved Programs
### Editable saved program APIs

The editable source-of-truth records live behind the shared robot-program API:

| Endpoint | Purpose |
|----------|---------|
| `POST /robot-program/save` | Save an editable robot program (`kind: "trajectory"` or `kind: "weld"`). |
| `GET /robot-program/list?kind=trajectory` | List saved editable trajectory programs. |
| `GET /robot-program/list?kind=weld` | List saved editable weld programs. |
| `GET /robot-program/{name}?kind=trajectory` | Load an editable trajectory program. |
| `GET /robot-program/{name}?kind=weld` | Load an editable weld program. |

Trajectory program records store:

- program metadata (`name`, `kind`, `saved_at`)
- `authoring.waypoints` with XYZ plus roll/pitch/yaw
- the latest `planned_trajectory` preview payload used by the UI

Weld program records use the same outer envelope but store weld-specific authoring data under `authoring`.

### Trajectory planning API

`POST /trajectory/plan-points` now accepts pose-aware waypoint payloads from the UI:

```json
{
  "waypoints": [
    {
      "x": 0.20,
      "y": 0.10,
      "z": 0.30,
      "orientation_euler_deg": {
        "roll": 0.0,
        "pitch": 90.0,
        "yaw": 0.0
      }
    }
  ]
}
```

The planner:

- interpolates waypoint-to-waypoint motion with the existing IK stack
- preserves the ordinary runnable `move_absolute` trajectory format
- returns dense preview path samples for the visualizer and program tree

### Execution mode contract

`POST /trajectory/run` accepts an optional `execution_mode`:

- `simulate` requires the controller runtime to be in `SIM`
- `live` requires the controller runtime to be in `LIVE`

When the controller is in the wrong runtime mode, operators can hot-switch it first:

- UI: use the header `LIVE` / `SIM` toggle and confirm the switch
- API: call `POST /control/runtime-mode` with `{ "mode": "simulate" | "live" }`

The controller owns that switch end-to-end: it stops motion, waits for idle, swaps the active backend in-process, updates desired `sim_mode`, and returns the refreshed runtime snapshot.

The response echoes both `execution_mode` and `runtime_mode` so the UI can show exactly what happened.

---

## Runnable Trajectory File Format
Each runnable recorded file still matches the structure of an entry in `trajectories.json`:
```json
{
  "description": "Recorded on 2023-11-05 14:37:01",
  "loop": false,
  "orientation_euler_angles_deg": null,
  "moves": [
    { "command": "move_absolute", "vector": [0.21, 0.27, 0.20] },
    { "command": "pause", "duration": 1.0 },
    { "command": "move_absolute", "vector": [0.21, 0.27, 0.05] },
    { "command": "pause", "duration": 1.0 }
  ]
}
```
Details:
* **move list** – Recorder-generated files store each way-point as a `move_absolute` command.  A default 1-second `pause` is inserted *between* way-points so you can see the arm settle.
* **pose-aware planning** – Planner-generated trajectories may also include `orientation_euler_deg` per `move_absolute` step.
* **editable vs runnable** – The editable source program is no longer the same thing as the runnable trajectory JSON. Editable programs live in `recorded_programs/`; runnable controller trajectories stay in `recorded_trajectories/`.

---

## Internals (Developers)
### Recorder internals
See `command_api._recording_state` and the three handlers:
  * `handle_plan_trajectory_start()`
  * `handle_record_position()`
  * `handle_end_trajectory(name)`

### Waypoint-planner internals

- Shared saved-program persistence lives behind the robot-program API in `src/gradient_os/api/main.py`.
- Trajectory planning flows through `command_api.plan_preview_trajectory_points(...)`.
- Cartesian interpolation / IK solving stays in `src/gradient_os/arm_controller/trajectory_execution.py`.
- Runnable preview files are still emitted under `recorded_trajectories/`.
- Editable source program records are stored under `recorded_programs/`.
- Playback still runs through `handle_run_trajectory()`.

### Telemetry recorder (episodes)
- Start/stop from UI Real Control page under "Telemetry Recorder" (leave camera URLs blank to auto-start the built-in MJPEG server on port 8080 and use both cameras, if available; server starts with `--vflip --hflip` by default).
- Controller commands:
  - `START_RECORDER,episodes_dir,prompt,base_cam,wrist_cam,fps,resize[,state_udp[,action_udp]]` — set `base_cam`/`wrist_cam` to empty or `auto` to auto-start the internal streamer and use `http://127.0.0.1:8080/cam0.mjpg` and `/cam1.mjpg`.
  - `STOP_RECORDER`
- Additionally, you can control the raw telemetry stream directly:
  - `START_TELEMETRY,host:port[,hz]`
  - `STOP_TELEMETRY`
- Episodes are created under `episodes_dir/YYYYMMDD_HHMMSS/` with `metadata.json`, `steps.jsonl`, and image subfolders `base/` and `wrist/`. Default `episodes_dir` is `recorded_episodes/` at the repo root; the folder is tracked but contents are git-ignored.

---

## Roadmap
* Richer waypoint operations such as multi-select edits and bulk retiming.
* GUI teach-pendant shortcuts for the legacy recorder path. 