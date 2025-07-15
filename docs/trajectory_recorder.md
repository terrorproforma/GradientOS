# Interactive Trajectory Recorder

> **Status:** Added in PR x.x (November 2023)
>
> This feature lets you “teach” the Mini-Arm by jogging it manually and dropping way-points on the fly.  The recorder stores these poses in exactly the same JSON format that `RUN_TRAJECTORY` already understands, so you can play them back immediately without post-processing.

---

## Why a Recorder?
Teaching by demonstration is often quicker than scripting a complex move by hand.  With the new *Recorder* op-codes you can:

1. **Start** a recording session (`PLAN_TRAJECTORY`).
2. **Move** the robot with any normal command (`MOVE_LINE`, `SET_ORIENTATION`, …).
3. **Drop** poses at interesting points (`REC_POS`).
4. **Finish** and save to disk (`END_TRAJECTORY,<name>`).

The result is a human-readable JSON file in `recorded_trajectories/` that you can replay with a single `RUN_TRAJECTORY` command.

---

## New UDP Commands

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

## File Format
Each recorded file matches the structure of an entry in `trajectories.json`:
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
* **move list** – The recorder stores each way-point as a `move_absolute` command.  A default 1-second `pause` is inserted *between* way-points so you can see the arm settle.  Edit times or vectors afterwards if desired.
* **orientation lock** – Currently `null`.  In future updates we may let you freeze tool orientation while replaying.

---

## Internals (Developers)
* **Implementation:** see `command_api._recording_state` and the three handlers:
  * `handle_plan_trajectory_start()`
  * `handle_record_position()`
  * `handle_end_trajectory(name)`
* **Storage path:** `recorded_trajectories/` (auto-created).
* **Playback:** `handle_run_trajectory()` now searches the recorded folder *before* falling back to `trajectories.json`.

---

## Roadmap
* Configurable pause duration / speed multiplier per way-point.
* Option to capture orientation Euler angles and enforce them during playback.
* GUI “teach pendant” that sends recorder op-codes automatically. 