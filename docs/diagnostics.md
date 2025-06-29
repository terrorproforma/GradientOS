# Diagnostics Guide

The controller includes a powerful set of diagnostic tools designed to help analyze and debug the performance of the IK solver and the trajectory executors. This is particularly useful for identifying and understanding behavior near joint singularities.

## 1. Live Diagnostic Logging

The primary way to capture performance data is through live logging. When enabled, every motion command that involves path planning (like `MOVE_LINE`, `MOVE_LINE_RELATIVE`, or `SET_ORIENTATION`) will automatically generate a detailed report.

### How to Enable

To enable live logging, you must set the `MINI_ARM_IK_LOG` environment variable to `1` before starting the controller.

```bash
# From your shell in the project root
export MINI_ARM_IK_LOG=1
python3 run_controller.py
```

Now, any relevant move command you send to the running controller will create a new diagnostic session.

**Note:** When live diagnostics are active, the open-loop executor will be capped at **400 Hz** (instead of its usual 1300 Hz) to account for the overhead of reading servo feedback for error reporting. A warning will be printed to the console when this happens.

### Output Folder Structure

Each diagnostic session creates a unique, timestamped folder to keep all related files together. The structure is:

```
diagnostics/
├── closed_loop/
│   └── 20231027_143000/  <-- Session ID
│       ├── ik_plan.csv
│       ├── timing.png
│       ├── error.png
│       └── sync.png
└── open_loop/
    └── 20231027_143115/  <-- Session ID
        ├── ik_plan.csv
        ├── timing.png
        └── error.png
```

### Generated Files

-   **`ik_plan.csv` (Most Important)**  
    This file contains the raw, unfiltered output from the batch IK solver. Each row contains the target Cartesian coordinate and the exact joint angles (in radians) the solver chose for that point. This is the key file for analyzing solver behavior, as you can plot the joint columns to visually identify large, sudden "flips" that indicate a singularity.

-   **`timing.png`**  
    This chart shows the performance of the executor thread, breaking down how long each part of the control loop took in milliseconds (e.g., reading feedback, writing commands). It's useful for spotting performance bottlenecks.

-   **`error.png`**  
    This chart displays the physical tracking error for each joint over the course of the move (commanded position vs. actual measured position). A spike in this chart almost always corresponds to a joint flip seen in the `ik_plan.csv`.

-   **`sync.png`** (Closed-loop only)  
    A more detailed breakdown of the timing for the `SyncRead` operation, which is often the main bottleneck in the closed-loop controller.

## 2. Standalone Path Analysis

For offline analysis or to test specific paths without running the full controller, you can use the `ik_path_diagnostics.py` script.

### How to Use

The script can generate its own straight-line path or load one from a JSON file.

**A. Generate a straight-line path:**

This command generates a 200-point path starting from the robot's current "home" position and moving 10cm straight down (`-0.1` in Z).

```bash
python3 diagnostics/ik_path_diagnostics.py --line 0 0 -0.1 200
```

**B. Load a path from a JSON file:**

The JSON file should contain a simple list of `[x, y, z]` coordinates.

```json
// my_test_path.json
[
  [0.4, 0.0, 0.3],
  [0.4, 0.05, 0.28],
  [0.4, 0.1, 0.26]
]
```

```bash
python3 diagnostics/ik_path_diagnostics.py --json my_test_path.json
```

### Output

The standalone script generates its output in the `diagnostics/ik_path/` directory. It produces a similar set of files (`ik_log_<timestamp>.csv`, `ik_angles_<timestamp>.png`, `ik_error_<timestamp>.png`) that allow you to analyze a specific path in isolation. 