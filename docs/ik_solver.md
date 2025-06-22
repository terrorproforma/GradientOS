## `ik_solver.py` - Inverse Kinematics Python Wrapper

**Primary Responsibility:** To provide a high-level, easy-to-use Python interface for the compiled C++ IKFast solver, and to handle the crucial geometric translation between the robot's wrist and the tool tip.

### File Description

This module acts as the bridge between the Python-based control logic and the high-performance, compiled C++ IK solver. It exposes the complex power of the IKFast library through simple, Pythonic functions like `solve_ik` and `get_fk`.

Its most important secondary responsibility is managing the **End-Effector Offset**. The mathematical model of the robot (`.urdf`) only defines the kinematics up to the center of the wrist flange. However, we want to control the position of the *tool tip*. This module contains the logic to translate a desired tool tip pose into the corresponding wrist pose that the IK solver requires.

---

### Core Concepts

#### End-Effector Offset

This is the most critical concept to understand when using the solver. The IKFast solver only knows about the 6 joints of the robot arm itself; it has no concept of a tool. Therefore, it solves for the position of the **wrist center**, not the tool tip.

To command the tool tip to a specific `(x, y, z)` coordinate, we must first tell the IK solver to target a different coordinate for the wrist. This new target is calculated by taking the tool's offset vector (defined by `END_EFFECTOR_OFFSET`), rotating it to match the desired tool orientation, and subtracting it from the desired tool tip position. This module handles this translation automatically.

```mermaid
graph TD
    A[Desired Tool Tip Pose<br/>(Position + Orientation)] --> B{ik_solver.py};
    B --> C{1. Rotate END_EFFECTOR_OFFSET<br/>to match desired orientation};
    C --> D{2. Calculate Wrist Position<br/>Wrist Pos = Tool Pos - Rotated Offset};
    D --> E[Target Wrist Pose];
    E --> F[IKFast C++ Solver];
    F --> G[Joint Angles Solution];
    G --> B;
```

#### Closest Solution

Inverse Kinematics can often have multiple valid solutions for a given pose (e.g., "elbow up" vs. "elbow down"). To ensure smooth, predictable motion, this module's `_find_closest_solution` function takes the list of valid solutions from the C++ solver and compares it to the robot's current joint angles. It then selects the solution that requires the smallest overall change in joint angles, preventing unnecessary large movements.

### Key Functions

*   **`solve_ik(target_position, ...)`**: The primary inverse kinematics function. It takes a desired `target_position` for the **tool tip**, performs the end-effector offset calculation described above, and returns the single best joint angle solution.

*   **`solve_ik_path_batch(path_points, ...)`**: A performance-critical function that takes a list of Cartesian points and solves for the entire path in a single call to the C++ backend. This is dramatically faster than calling `solve_ik` in a Python loop and is the foundation of the high-fidelity trajectory planner.

*   **`get_fk(active_joint_angles)` / `get_fk_matrix(...)`**: The forward kinematics functions. They take a set of joint angles and use the C++ solver to calculate the resulting Cartesian pose of the wrist. They then apply the `END_EFFECTOR_OFFSET` in the forward direction to return the final pose of the **tool tip**. 