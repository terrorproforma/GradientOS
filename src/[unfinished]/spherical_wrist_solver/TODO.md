# Project TODO / Roadmap

This living document captures **all** currently known or imagined future
improvements for the *spherical-wrist-solver* repository.  Items are grouped
by *priority* (impact × effort) so that the most valuable upgrades rise to the
surface.  Within each tier, tasks are listed roughly in recommended execution
order but can obviously be re-shuffled as the project evolves.

Legend for priority labels
--------------------------
| Tag | Meaning                                            |
|-----|----------------------------------------------------|
| **P0** | Critical for correctness or basic usability. Must-have before any serious deployment. |
| **P1** | High value enhancements that noticeably improve robustness, performance or developer UX. |
| **P2** | Nice-to-have features or refactors that bring long-term pay-offs. |
| **P3** | Low-hanging fruit, experimental ideas, stretch goals. |

-------------------------------------------------------------------------------
## P0 ‑ Immediate Blockers / Short-Term Must-Haves
-------------------------------------------------------------------------------
1. **Robust build & CI pipeline**  
   • GitHub Actions (or equivalent) covering Linux + Windows + macOS.  
   • Matrix for `python-versions × compiler` to ensure the C++/pybind11 module
     compiles everywhere.  
   • Artifact upload so end-users can `pip install` platform wheels.
2. **Unit-test coverage ≥ 90 %**  
   • `pytest` suite for: forward kinematics, IK branch enumeration, Jacobian, …  
   • Golden-file tests against reference data (e.g. MoveIt solutions).  
   • Randomised property-based tests with `hypothesis`.
3. **Numerical stability audit**  
   • Validate behaviour near singularities (e.g. wrist-flip).  
   • Double-precision vs single-precision comparison.  
   • Worst-case condition-number tracking in the Jacobian.
4. **Graceful runtime error handling**  
   • Meaningful Python exceptions rather than `std::abort()`.  
   • Optional retry logic with small perturbations for borderline cases.
5. **Enforce URDF joint-limit consistency**  
   • Pre-flight check that limits, mimic tags, and reductions match reality.

-------------------------------------------------------------------------------
## P1 ‑ High-Impact Enhancements
-------------------------------------------------------------------------------
1. **Collision-aware Cartesian planner (MoveIt-style "check + short-cut")**  
   • Use `trimesh` + `fcl` or Bullet for self + environment collision tests.  
   • Incremental shortcutting to smooth the path while staying collision-free.
2. **Time-parameterisation (velocity/acceleration/jerk limits)**  
   • Integrate `toppra` or Reflexxes Type-IV online trajectory generator.  
   • Export CSV/JSON compatible with common industrial controllers.
3. **Singularity avoidance / dexterity weighting**  
   • Optimise branch choice based on manipulability index.  
   • Warn when approaching determinant(J) → 0.
4. **Continuous joint re-sampling**  
   • Replace greedy branch selection with global dynamic programming to
     minimise cumulative joint movement over *all* waypoints.
5. **Plugin architecture for alternative IK solvers**  
   • Swappable back-ends (numerical, closed-form, Newton-Raphson, Levenberg-Marquardt).  
   • Benchmark harness for objective comparison.
6. **Pythonic ¶nterface polish**  
   • `__call__` sugar (`ik(R, p)`), `dataclasses`, type hints everywhere.  
   • Sphinx docs with MyST markdown and auto-generated API pages.

-------------------------------------------------------------------------------
## P2 ‑ Medium-Term Quality-of-Life Improvements
-------------------------------------------------------------------------------
1. **ROS 2 rclpy wrapper**  
   • Publish `sensor_msgs/JointState` and `geometry_msgs/PoseStamped`.  
   • Action server for `FollowJointTrajectory`.
2. **Graphical debug tools**  
   • Real-time Matplotlib or PyQt 3-D visualiser for the chain, collision hulls
     and frames.  
   • Trajectory playback with step-through and joint limit heat-maps.
3. **Automated URDF sanity checker**  
   • Verify inertial tags, link orientations, zero offsets.  
   • Suggest corrections and generate a diff patch.
4. **Binding generator for other languages**  
   • Rust (`pyo3` cross-compile) and JavaScript (WebAssembly + Emscripten) demos.
5. **Performance profiling & SIMD vectorisation**  
   • Identify hotspots with `py-spy` / `perf`.  
   • Experiment with `Eigen::VectorXd::unroll()` and `OpenMP`.
6. **Parameterized calibration utilities**  
   • Least-squares estimation of DH/PoE parameters from measurement data.

-------------------------------------------------------------------------------
## P3 ‑ Experimental / Long-Term Ambitions
-------------------------------------------------------------------------------
1. **Learning-based error correction**  
   • Residual neural network predicting joint offsets given encoder feedback.  
   • Online adaptation via incremental SGD.
2. **Task-level redundancy resolution**  
   • Null-space optimisation for secondary objectives (energy, payload sway).  
   • Constrained optimisation using quadratic programming.
3. **Realtime embedded deployment**  
   • Cross-compile to ARM Cortex-A53 (Raspberry Pi) and Cortex-R (micro-ROS).  
   • Evaluate Xeno-based hard RT patch.
4. **Model-based vibration damping**  
   • Input-shaping filters and active damping control law examples.
5. **Full digital twin integration**  
   • Co-simulation with Gazebo Sim or NVIDIA Isaac Sim.  
   • Scenario API for stress-testing with thousands of random tasks.
6. **Autotuned path planners**  
   • Genetic algorithms or Bayesian optimisation to tune planner heuristics.

-------------------------------------------------------------------------------
## Miscellaneous Ideas (unranked / backlog)
-------------------------------------------------------------------------------
* Support for mixed translational + rotational joint types.  
* IEC-61131 exports (Structured-Text function blocks).  
* Formal verification (SMT-solver reachability proofs).  
* End-to-end tutorials mirroring FANUC / KUKA workflows.  
* Automatic changelog and semantic-versioning release pipeline.

> **Tip:** When you complete an item, please strike-through the text and attach
> the pull request number for traceability. 