# RTOS EtherCAT Control Stack

This document distills the final architecture, lessons learned, and operational design principles for the GradientOS live control stack, with a specific focus on the relationship between:

- the Python controller layer,
- the RT motion daemon (`gradient-rt-motion`),
- and the EtherCAT master / host configuration.

It is based on:

- `.cursor/memory/AGENT_SCRATCHPAD.md`
- `.cursor/memory/DEVLOG.md`
- `DEVLOG.md`
- `docs/rtcore_owned_motion_contract.md`
- `docs/ethercat/bringup.md`
- `docs/ethercat/igh.md`
- `src/gradient_os/run_controller.py`
- `src/gradient_os/arm_controller/command_api.py`
- `src/gradient_os/arm_controller/backends/ethercat_rtcore/backend.py`
- `src/gradient_rt_motion/main.cpp`
- `src/gradient_rt_motion/ipc_v1.hpp`
- `systemd/ethercat-host/*`
- `systemd/rt-motion/*`
- `systemd/controller/*`

## 1. Final Architectural Outcome

The final stack is intentionally split into three layers with different responsibilities and privileges:

### 1.1 Python controller owns orchestration and policy

The controller is the system orchestrator, not the realtime executor.

It owns:

- runtime selection and activation,
- controller/API command dispatch,
- robot/tool/IK policy,
- Cartesian planning and IK,
- high-level program sequencing,
- safety orchestration around backend lifecycle,
- operator-facing motion state and power-transition gating.

It does **not** own cycle timing for EtherCAT motion once RTCore is active.

This is the key finalized rule from `docs/rtcore_owned_motion_contract.md`:

- Python owns planning and policy.
- RTCore owns deterministic replay timing, jog timeout handling, and execution truth.

### 1.2 RTCore owns deterministic motion execution

`gradient-rt-motion` is the realtime process. It runs as root, talks to IgH/libecrt, owns the cyclic loop, and writes the actual process data to the drives.

It owns:

- the realtime cycle,
- EtherCAT process data exchange,
- DS402 controlword sequencing inside the RT cycle,
- scheduled trajectory replay,
- jog target integration,
- command timeout enforcement,
- motion execution state publication.

The Python EtherCAT backend is only a transport/proxy into RTCore. It does not perform EtherCAT I/O itself.

### 1.3 EtherCAT master ownership stays inside RTCore

The final design avoids “two masters” or split authority:

- IgH `ethercat.service` binds the NIC and provides the master infrastructure,
- `gradient-rt-motion` activates libecrt and owns active fieldbus operation,
- the controller communicates only through IPC.

This separation is deliberate and necessary:

- root + RT privileges stay in RTCore,
- the controller can remain unprivileged (`User=pi`, `Group=pi`),
- fieldbus faults and timing are localized to one process.

## 2. Final Ownership Boundary

The most important architecture decision is the ownership boundary.

### Python owns

- Cartesian planning and waypoint generation
- IK / batched IK
- singularity-aware shaping
- move semantics (`move`, `move_absolute`, `home`, etc.)
- runtime mode orchestration
- program-level sequencing
- REST/UI/controller contracts

### RTCore owns

- 1 kHz loop timing
- command ring consumption
- trajectory interpolation at RT cadence
- jog integration at RT cadence
- stale jog timeout behavior
- per-cycle target clamping
- DS402 transition-safe target management
- final execution status

### EtherCAT master / host owns

- correct NIC binding
- IRQ / NIC tuning
- DC-capable process-data transport
- stable slave discovery and operational state

One of the strongest lessons from the trial-and-error process was that these boundaries must stay sharp. Reintroducing policy into RTCore or realtime timing into Python made bugs harder to reason about and safety harder to validate.

## 3. Service Topology and Privilege Model

The final appliance-style startup topology is:

1. `ethercat.service`
2. `gradient-rt-motion.service`
3. `arm-controller.service`

For the local staged launcher, the equivalent operator-facing sequence is:

1. `./start-stack.sh`
2. controller stage becomes reachable
3. launcher waits for RTCore bus readiness, not just controller liveness
4. API starts
5. web UI starts

That distinction matters:

- service ordering expresses dependency,
- launcher staging expresses operational readiness.

From `systemd/rt-motion/gradient-rt-motion.service`:

```ini
[Unit]
Requires=ethercat.service
After=ethercat.service

[Service]
User=root
Group=pi
RuntimeDirectory=gradient-rt-motion
RuntimeDirectoryMode=0770
ExecStart=/usr/local/bin/gradient-rt-motion ...
LimitRTPRIO=95
LimitMEMLOCK=infinity
```

From `systemd/controller/arm-controller.service`:

```ini
[Unit]
Wants=gradient-rt-motion.service
After=network.target gradient-rt-motion.service

[Service]
User=pi
Group=pi
Environment=GRADIENT_RTCORE_AUTOSTART=1
ExecStart=/home/pi/GradientOS/run.sh
```

Design intent:

- RTCore runs privileged because EtherCAT + RT scheduling require it.
- The controller runs as `pi`.
- RTCore exposes a group-owned IPC socket so the controller can connect without root.

This is a clean final outcome. Earlier variants were more brittle because the controller, launcher, and EtherCAT bring-up all partially overlapped in responsibility.

## 3.1 Core Startup Sequence

The safe startup sequence that future developers and agents should preserve is:

1. Host OS boots with the dedicated EtherCAT NIC role/tuning installed.
2. `ethercat.service` binds the configured NIC and provides the master infrastructure.
3. `gradient-rt-motion.service` starts and creates:
   - `/run/gradient-rt-motion/ipc.sock`
   - `/run/gradient-rt-motion/metrics.json`
4. The controller resolves the effective runtime policy.
5. If the selected live backend is `ethercat_rtcore`, the controller:
   - ensures RTCore is available,
   - waits for RTCore metrics to report the bus ready,
   - only then creates the backend proxy and starts normal command handling.
6. The controller lands in a synchronized, disarmed state first.
7. Explicit operator/API power-up is then required to arm/enable the drives.

The critical safety property is:

- **startup should land `BUS_UP_DISARMED`, not `ACTIVE`.**

This is the final, intentional behavior of the EtherCAT path. A “successful” startup that auto-enables the drives is not considered correct behavior for this stack.

The staged local launcher documents this explicitly in `docs/README.md`:

```text
startup does not advance past the controller stage until the RTCore metrics report
the full bus online and operational for all configured axes
```

### 3.2 Controller startup logic that must not be bypassed

The controller startup path in `run_controller.py` now does the right things in the right order:

```python
if servo_backend_local == "ethercat_rtcore" and not target_sim_mode:
    _ensure_ethercat_rtcore_available()
    _wait_for_ethercat_rtcore_ready(selected_robot_local.num_physical_actuators)

backend_registry.set_active_backend(servo_backend_local)
...
active_backend_local = backend_registry.create_backend(...)
backend_ready_local = bool(active_backend_local.initialize())
...
_sync_joint_and_gripper_state(...)
```

The ordering here is important:

- do not instantiate the EtherCAT backend before RTCore is available,
- do not declare the live system ready before the controller has synchronized its internal state to real feedback,
- do not auto-arm by default on connect.

### 3.3 RTCore startup and shutdown behavior

RTCore itself implements best-effort safe shutdown behavior on `SIGINT`/`SIGTERM`:

```cpp
// If a shutdown is requested (SIGINT/SIGTERM), disable all axes for a short
// grace window while we still have cyclic communication.
if (g_stop.load(std::memory_order_relaxed) && !shutdown_active) {
  shutdown_active = true;
  shutdown_until_ns = diag_now_ns + kShutdownGraceNs;
  armed.store(false, std::memory_order_relaxed);
  axis_enable_mask.store(0, std::memory_order_relaxed);
  mode_of_operation.store(0, std::memory_order_relaxed);
}
```

This is not a substitute for the controller’s safe power-down path. It is a final safety net so the daemon does not simply disappear while the drives are still expecting cyclic control.

## 3.4 Safe Restart and No-Sudden-Move Rules

The final stack distinguishes three different operations:

### Soft stop

Normal operator stop:

- use `./start-stack.sh stop`
- or `POST /control/power-down` with `{"wait_for_idle": true}`

This should:

- stop jog,
- abort queued RTCore trajectory ownership,
- wait for motion neutrality,
- disable axes,
- disarm,
- leave RTCore + EtherCAT alive on a synchronized bus.

This is the preferred path because it avoids provoking EtherCAT sync-loss drive faults during normal shutdown. Historically this was tied to A6-EC `ErC1.1`-style synchronization-loss behavior when the bus was torn down too early.

### Hard stop

Full teardown:

- use `./start-stack.sh stop --hard`

This is allowed to stop:

- controller
- API
- web UI
- RTCore
- `ethercat.service`

Use it only when you explicitly want the full stack down.

### Controller restart

For restart-bound policy changes:

- use `REQUEST_RESTART`
- or `POST /control/restart-controller`

The restart path is not a raw process kill. It is intended to:

- acknowledge the restart request,
- begin stop/idle shutdown sequencing,
- then exit so the supervisor can restart it cleanly.

From `docs/command_api.md`:

```text
REQUEST_RESTART ... returns ACK before the controller begins stop/idle shutdown sequencing and exits.
```

### The “no sudden move” contract

The final safety contract that prevents surprise motion is:

1. Startup lands disarmed.
2. Power-up is blocked unless `safe_for_power_transition=true`.
3. Power-up synchronizes command targets to live feedback before enable.
4. STOP on RTCore aborts motion ownership but does not inject a new hold-position trajectory.
5. Jog stop/timeout collapses targets onto live feedback.
6. Shutdown keeps cyclic communication alive briefly while disabling axes.

If a future change weakens any of those six guarantees, it should be treated as a likely safety regression.

### 3.5 Safe stop / restart acceptance criteria

A normal safe stop or restart path should satisfy all of the following before RTCore or EtherCAT are torn down:

- controller motion/program thread is no longer active,
- RTCore reports no active trajectory,
- RTCore reports `queue_depth=0`,
- RTCore reports no active jog,
- `motion_done=true`,
- `stale_command=false`,
- no live drive faults are present,
- command targets are synchronized to live feedback,
- physical state is `BUS_UP_DISARMED`,
- `/control/motion-status` reports `safe_for_power_transition=true`.

If the system is only “process stopped” but not at those criteria, it is not a fully safe software outcome yet.

## 4. Host / EtherCAT Master Setup Principles

### 4.1 NIC role must be explicit and reproducible

The repo’s host setup in `systemd/ethercat-host/install.sh` converged on:

- deterministic NIC naming,
- one NIC dedicated to EtherCAT,
- one NIC kept for uplink,
- EtherCAT NIC marked unmanaged by NetworkManager,
- `/etc/ethercat.conf` generated from a single repo-owned role source.

Key lesson:

- historical docs about “which port worked” are not authoritative if a fresh live probe contradicts them.

This came directly from the March 19 debugging cycle:

- stale docs said `eth0` worked and `eth1` failed,
- live diagnostics proved the actual slave chain was on `eth1`,
- templates had fossilized an older wiring assumption.

Final rule:

- trust the current live probe over old bring-up lore.

### 4.2 Slave discovery and RTCore readiness are different checks

One of the most important operational lessons was that these are not the same:

- `ethercat master` / `ethercat slaves -v` can show the bus is discoverable,
- while RTCore can still have `wkc=0`, slaves stuck in `INIT`, or no useful process data.

Related IgH nuance from `docs/ethercat/igh.md`:

- `ethercat master` showing `Active: no` does **not** mean the kernel master is broken.
- It only means no userspace libecrt application has activated the master yet.
- That is normal during bare discovery with the CLI and should not be confused with link/bus failure.

The stack looked “alive” multiple times while the fieldbus was still functionally dead.

The final readiness gate in `run_controller.py` reflects that:

```python
def _rtcore_metrics_ready(metrics: dict[str, object] | None, *, expected_axes: int) -> tuple[bool, str]:
    ...
    if link_up == 0:
        return False, "link_up=0"
    if responding < expected_axes:
        return False, f"responding={responding}/{expected_axes}"
    if online < expected_axes:
        return False, f"online={online}/{expected_axes}"
    if operational < expected_axes:
        return False, f"operational={operational}/{expected_axes}"
    if startup_ready == 0:
        return False, (
            f"startup_ready=0 operational={operational}/{expected_axes} "
            f"wkc={wkc_actual}/{wkc_expected}"
        )
    if wkc_actual <= 0:
        return False, f"wkc={wkc_actual}/{wkc_expected}"
    return True, ...
```

Final principle:

- do not treat controller socket availability or process startup as proof the motion stack is ready.
- readiness must be based on RTCore metrics and live bus health.

### 4.3 NIC tuning matters, but it is not the whole story

The repo learned that:

- host tuning drift on the actual EtherCAT NIC can materially worsen RX/frame-loss behavior,
- but bad WKC is not automatically a CPU pinning problem,
- and IRQ affinity assumptions can fail on real hardware/kernel combinations.

The strongest mature lesson here is practical:

- keep tuning scripts driven from the actual configured EtherCAT NIC,
- but debug bus state, PDO/state sequencing, and slave operational state separately from CPU isolation theory.

## 5. Controller Runtime Activation and Lifecycle

### 5.1 One activation path for startup and hot-switch

The controller now uses the same runtime activation path for:

- initial startup,
- live/sim hot-switches.

This was a major cleanup. Earlier approaches risked:

- startup doing one thing,
- hot-switch doing another,
- and hidden side effects such as stale monkeypatch activation paths.

The final controller behavior from `run_controller.py`:

- resolve runtime policy,
- ensure RTCore is present when live EtherCAT is selected,
- wait for RTCore readiness,
- instantiate the selected backend,
- synchronize software state to live hardware,
- then serve commands.

### 5.2 Live/sim switching is controller-owned, not UI-owned

The finalized rule is:

- API/UI forward a single command,
- the controller performs stop, wait-for-idle, backend shutdown, backend creation, and desired runtime persistence.

From `run_controller.py`:

```python
elif command == "SWITCH_RUNTIME_MODE":
    ...
    if current_sim_mode != target_sim_mode:
        command_api.handle_stop_command()
        idle_payload = command_api.handle_wait_for_idle()
        runtime_request = runtime_config.derive_runtime_request_from_active_runtime(
            active_runtime_config
        )
        runtime_request["sim_mode"] = target_sim_mode
        _shutdown_active_runtime_backend()
        _activate_runtime(runtime_request, reason=f"hot-switch:{mode_token}")
```

Key lesson from the scratchpad:

- build hot-switch requests from the active runtime, not from pending desired config, otherwise unrelated staged changes can be accidentally applied.

### 5.2.1 Dedicated LIVE <-> SIM hot-switch contract

This needs to be explicit because it is one of the easiest places for future changes to accidentally break safety.

The final contract is:

- `LIVE -> SIM` is a **controller-owned hot switch**, not a process relaunch.
- `SIM -> LIVE` is a **controller-owned hot switch**, not a direct drive re-enable.
- the API and frontend must forward intent, but they must not perform lifecycle sequencing themselves.

The exact implementation anchor is the controller command handler in [run_controller.py](/Users/angusmuffatti/Desktop/My_Apps/GradientOS/src/gradient_os/run_controller.py#L1640):

```python
if current_sim_mode != target_sim_mode:
    command_api.handle_stop_command()
    idle_payload = command_api.handle_wait_for_idle()
    runtime_request = runtime_config.derive_runtime_request_from_active_runtime(
        active_runtime_config
    )
    runtime_request["sim_mode"] = target_sim_mode
    _shutdown_active_runtime_backend()
    _activate_runtime(runtime_request, reason=f"hot-switch:{mode_token}")
```

That sequence is not accidental. Each step exists for a safety reason.

### 5.2.2 `LIVE -> SIM` sequence

When switching from live EtherCAT/RTCore to SIM, the correct sequence is:

1. Validate the request at the API/UI layer and forward `SWITCH_RUNTIME_MODE,simulate`.
2. In the controller, issue `STOP`.
3. Wait for composite motion idle using `WAIT_FOR_IDLE`.
4. Build the new runtime request from the **active runtime**, not the pending desired config.
5. Set only `sim_mode=True` on that derived runtime request.
6. Shut down the active backend instance.
7. During shutdown of the EtherCAT RTCore backend:
   - stop active jog ownership,
   - abort any active RTCore trajectory ownership,
   - perform best-effort safe power-down,
   - disable axes,
   - disarm.
8. Activate the simulation backend in-process.
9. Persist desired `sim_mode=true`.
10. Return the refreshed runtime snapshot to the API/UI.

Important design details:

- normal hot-switching to SIM does **not** require stopping `gradient-rt-motion.service` or `ethercat.service`.
- the live fieldbus services may remain up on the host, but the controller must no longer hold an active live backend instance and the drives must be left in the safe disarmed state.
- the switch is only safe if motion has been stopped and the live backend has been shut down cleanly first.

Operationally, the postcondition for `LIVE -> SIM` is:

- controller mode is simulate,
- active backend is simulation,
- live RTCore motion ownership has been released,
- no queued/live EtherCAT motion remains owned by the controller,
- the physical robot is not left armed because the operator toggled into SIM.

### 5.2.3 `SIM -> LIVE` sequence

When switching from SIM back to live EtherCAT/RTCore, the correct sequence is:

1. Validate the request at the API/UI layer and forward `SWITCH_RUNTIME_MODE,live`.
2. In the controller, issue `STOP`.
3. Wait for composite motion idle using `WAIT_FOR_IDLE`.
4. Build the new runtime request from the **active runtime**, not from staged desired config.
5. Set only `sim_mode=False` on that derived runtime request.
6. Shut down the active simulation backend instance.
7. Activate the live runtime in-process.
8. If the selected live backend is `ethercat_rtcore`, the activation path must:
   - ensure RTCore is available,
   - wait for RTCore readiness metrics,
   - create the EtherCAT RTCore backend proxy,
   - connect IPC,
   - receive axis/status config,
   - synchronize software state to real feedback.
9. Persist desired `sim_mode=false`.
10. Return the refreshed runtime snapshot to the API/UI.

The critical postcondition for `SIM -> LIVE` is:

- the system should land `BUS_UP_DISARMED`, **not** automatically back in `ACTIVE`.

This is one of the most important missing details future developers need:

- hot-switching back to LIVE must not restore a previous armed/enabled state automatically.
- returning to live means “live backend ready and synchronized”, not “robot enabled and ready to move immediately”.
- the operator must still explicitly request `SAFE_POWER_UP` if they want the drives energized.

If a future implementation makes `SIM -> LIVE` auto-enable the drives because “that’s more convenient”, that should be treated as a safety regression.

### 5.2.4 Why the hot-switch request must be derived from the active runtime

This is one of the most important historical lessons from the hot-switch work.

The controller must derive the switch request from the active runtime:

```python
runtime_request = runtime_config.derive_runtime_request_from_active_runtime(
    active_runtime_config
)
runtime_request["sim_mode"] = target_sim_mode
```

The reason is subtle but important:

- the desired runtime config may already contain unrelated staged robot/backend/profile/tool changes,
- and a mode switch is only supposed to change the mode.

If future code instead builds the switch from desired config, a user could toggle LIVE/SIM and accidentally apply:

- a different robot,
- a different backend override,
- a different drive profile,
- a different RT clamp,
- or a different active tool selection.

That is exactly the kind of hidden side effect that makes runtime switching unsafe and hard to reason about.

### 5.2.5 Restart semantics around LIVE/SIM

The hot-switch work also finalized an important policy rule:

- pure `LIVE <-> SIM` mismatches are **hot-switchable**,
- they are not restart-bound changes.

That behavior is encoded in `compute_restart_required()` in [runtime_config.py](/Users/angusmuffatti/Desktop/My_Apps/GradientOS/src/gradient_os/runtime_config.py#L529), which compares desired runtime against a simulated hot-switched version of the current active runtime rather than blindly treating all servo/runtime changes as restart-required.

The practical result is:

- toggling between live and sim should not show up as a restart-required policy difference,
- but real restart-bound differences should still be detected.

### 5.2.6 What must never happen during LIVE/SIM switching

Future developers and agents should treat the following as forbidden behaviors:

- the API or frontend directly tearing down/creating backends,
- skipping `STOP` before switching runtime mode,
- skipping `WAIT_FOR_IDLE` before switching runtime mode,
- building the switch from desired config instead of active runtime,
- leaving the live EtherCAT backend armed while entering SIM,
- auto-powering-up the robot when switching from SIM back to LIVE,
- reintroducing one-way `sim_backend.activate()` monkeypatch behavior into the normal switch path.

### 5.3 RTCore autostart belongs in the controller path

Another finalized outcome was adding best-effort RTCore autostart from the controller.

From `run_controller.py`:

```python
def _ensure_ethercat_rtcore_available() -> None:
    ...
    if os.path.exists(socket_path):
        return
    ...
    started = _run_service_command(["systemctl", "start", _RTCORE_SERVICE_NAME], require_root=True)
```

This mattered because `run.sh` previously did not reliably bring RTCore up for `ethercat_rtcore`, which made the high-level path less appliance-like and harder to trust.

### 5.4 Handoff between layers

The final handoff model is:

- frontend expresses user intent,
- API validates/request-shapes and forwards,
- controller decides policy, sequencing, and backend lifecycle,
- RTCore executes live EtherCAT motion.

That means the frontend and API should be thin and boring by design.

Concrete examples from `src/gradient_os/api/main.py`:

```python
@api.post("/control/power-down")
async def control_power_down(payload: dict[str, Any] | None = None):
    command = "SAFE_POWER_DOWN,wait" if wait_for_idle else "SAFE_POWER_DOWN"
    detail, structured = await run_in_threadpool(
        _controller_structured_call, command, "SAFE_POWER_DOWN", timeout=5.0
    )
```

```python
@api.post("/control/runtime-mode")
async def control_runtime_mode(payload: dict[str, Any] | None = None):
    ...
    detail, structured = await run_in_threadpool(
        _controller_structured_call,
        f"SWITCH_RUNTIME_MODE,{mode}",
        "SWITCH_RUNTIME_MODE",
        timeout=20.0,
    )
```

These routes are intentionally wrappers. The API should not orchestrate:

- backend teardown,
- RTCore state transitions,
- drive safety sequencing,
- or runtime policy activation.

That all belongs in the controller.

### 5.5 Runtime payload normalization warning

Another subtle lesson from the hot-switch and SIM work:

- `GET_RUNTIME_CONFIG` from the controller returns the raw active runtime payload,
- but some higher-level helpers reason about the wrapped `/info/runtime-config` API shape.

That mismatch caused false SIM-mode rejections in the past.

Future developers should preserve this rule:

- any helper that inspects runtime mode/backend/tool state must accept either the raw controller payload or normalize it first.

Do not assume every caller is already using the wrapped API shape.

## 6. Python <-> RTCore Communications Architecture

The final comms model is robust and intentionally explicit:

- UDS handshake for discovery/versioning,
- `SCM_RIGHTS` to pass shared-memory and eventfd handles,
- one shared memory region for commands,
- one shared memory region for status,
- fixed-size ring entries,
- eventfds for wakeups,
- structured motion-status snapshots instead of inferred completion.

### 6.1 Handshake and FD passing

Python backend:

```python
hello = _HELLO_STRUCT.pack(
    _MAGIC_GIPC,
    _VER_MAJOR,
    _VER_MINOR,
    _HELLO_STRUCT.size,
    _ROLE_CONTROLLER,
    os.getpid(),
    0, 0, 0, 0,
)
sock.sendall(hello)
data, ancdata, _flags, _addr = sock.recvmsg(
    _WELCOME_STRUCT.size,
    socket.CMSG_SPACE(fd_size * 4),
)
```

RTCore:

```cpp
gradient::ipc::v1::WelcomeV1 welcome{};
welcome.magic = gradient::ipc::v1::kMagicGipc;
welcome.num_axes = opt.num_axes;
welcome.cycle_ns = opt.cycle_ns;
...
int fds[4] = {cmd_shm.fd, status_shm.fd, cmd_eventfd, status_eventfd};
...
cmsg->cmsg_level = SOL_SOCKET;
cmsg->cmsg_type = SCM_RIGHTS;
std::memcpy(CMSG_DATA(cmsg), fds, sizeof(fds));
sendmsg(controlling_client_fd, &msg, 0)
```

This is a strong final design because:

- Python gets a zero-copy command/status path,
- RTCore stays in control of buffer ownership and layout,
- ABI validation is explicit.

### 6.2 Ring-based command publishing

Python writes command messages into the RTCore command ring:

```python
header = _MSG_HEADER_STRUCT.pack(
    int(msg_type) & 0xFFFF,
    0,
    _MSG_HEADER_STRUCT.size + len(payload),
    msg_seq,
    int(time_ns),
)
...
self._cmd_shm[off : off + msg_bytes] = blob
self._cmd_shm[ring_hdr_off + 12 : ring_hdr_off + 16] = struct.pack("<I", write_idx)
os.write(self._cmd_eventfd, struct.pack("<Q", 1))
```

Design implications:

- messages are ordered and sequenced,
- producer overflow is explicit,
- wakeup is decoupled from transport,
- RTCore is free to consume on its own cadence.

### 6.3 Status publication is first-class

RTCore publishes:

- `STATUS_SNAPSHOT`,
- `STATUS_MOTION_STATE`,
- `STATUS_JOG_DEBUG`.

This is one of the biggest architectural improvements over the earlier mixed model:

- higher layers no longer need to infer completion from Python thread joins, ad hoc polling, or “the last move probably finished”.

## 6.4 Command and status ownership rules for future development

When adding new motion-related features:

- if the feature is about user intent, request shaping, workflow, or planning, it probably belongs in Python/controller.
- if the feature is about per-cycle timing, stale command behavior, CSP target generation, or motor-rate safety, it probably belongs in RTCore.
- if the feature is about rendering, forms, UX, or status display, it belongs in frontend/UI.
- if the feature is about HTTP validation, serialization, or exposing controller commands to clients, it belongs in the API layer.

The API and frontend should not become alternative control stacks.

## 6.5 “As much inside RTCore as possible” means the RT-critical parts

The user goal of keeping “as much inside RTCore as possible” should be interpreted carefully.

It does **not** mean:

- move IK into C++ by default,
- move robot/program authoring semantics into RTCore,
- or duplicate planning logic in multiple layers.

It **does** mean:

- move anything RT-critical or drive-safety-critical into RTCore,
- keep RTCore as the owner of final live execution truth,
- keep timeout/queue/jog/hold semantics close to the cycle loop,
- and avoid Python-timed live motion where RTCore already has a proper contract.

This is the correct balance between safety/performance and cross-backend maintainability.

## 7. Motion Contract: Finalized Modes and State Truth

The final motion contract is now mature and explicit.

### 7.1 Motion modes

From `runtime.py` and `ipc_v1.hpp`:

- `idle`
- `legacy_setpoint`
- `trajectory`
- `jog`

Important final rule:

- `legacy_setpoint` is compatibility scaffolding only.
- real scheduled motion should use buffered RTCore trajectories.

### 7.2 Execution states

Final RTCore execution states:

- `idle`
- `accepted`
- `queued`
- `executing`
- `completed`
- `aborted`
- `faulted`
- `underrun`

This is the core design principle that cleaned up a lot of earlier ambiguity:

- acceptance is not completion,
- controller program-thread state is not the same as RTCore physical segment state,
- UI/API should read structured motion status, not guess.

### 7.3 Completion truth must come from status, not just ACKs

The repo repeatedly hit bugs where:

- commands were accepted but not completed,
- very short trajectories completed between polls,
- post-power-down motion status retained stale active trajectory IDs.

The final answer was not “poll harder”. It was:

- make RTCore publish better truth,
- refresh status after submission,
- carry submitted command sequence IDs,
- distinguish controller-program status from RTCore segment status.

Example of the short-trajectory race hardening in the backend:

```python
elif (
    not saw_target_trajectory
    and submitted_command_seq is not None
    and active_command_seq >= int(submitted_command_seq)
    and status.motion_done
    and queue_depth == 0
    and state_name in terminal_state_names
):
    return status
```

Lesson:

- realtime systems can complete valid work too fast for naive polling logic.
- sequence-aware status is more reliable than “did I observe `active_traj_id` long enough?”.

## 8. Why RTCore Owns Trajectories and Jog Separately

This was one of the strongest design clarifications from the iteration history.

### 8.1 Scheduled motion should be queued, not streamed from Python

The final model is:

- Python plans joint paths,
- RTCore quantizes timing to the cycle,
- RTCore replays the path.

From the backend:

```python
timing = self.resolve_trajectory_frequency(frequency)
...
traj_id = self.begin_trajectory(expected_points=len(joint_path))
...
self.enqueue_trajectory_points(traj_id, points)
submitted_command_seq = self.commit_trajectory(traj_id)
return self.wait_for_trajectory_complete(...)
```

From RTCore:

```cpp
case gradient::ipc::v1::MSG_CMD_TRAJECTORY_BEGIN:
case gradient::ipc::v1::MSG_CMD_TRAJECTORY_POINT:
case gradient::ipc::v1::MSG_CMD_TRAJECTORY_COMMIT:
```

This is better than Python-timed streaming because:

- cycle timing stays deterministic,
- queue semantics are explicit,
- execution state is owned by the thing doing the execution.

### 8.2 Jog is a different control mode, not a fake trajectory

The final design keeps jog as its own RT mode with timeout semantics.

This matters because jog needs:

- watchdog expiry,
- quick stop behavior,
- hold-to-feedback collapse when motion stops,
- continuous target integration.

From RTCore:

```cpp
if (active_jog_deadline_ns != 0 && diag_now_ns > active_jog_deadline_ns) {
  ...
  snap_jog_hold_to_feedback_mask |= active_jog_axis_mask;
  last_jog_stop_reason = gradient::ipc::v1::JOG_STOP_REASON_TIMEOUT;
  active_jog = false;
  ...
}
```

And critically:

```cpp
if ((snap_jog_hold_to_feedback_mask & (1u << i)) != 0u) {
  hold_target_counts[i] = pos;
  have_hold[i] = true;
}
if (jog_stop_arrest_cycles_left[i] > 0) {
  hold_target_counts[i] = pos;
  have_hold[i] = true;
}
```

This was a hard-won best practice:

- when jog stops or times out, immediately collapse the hold target to live feedback.
- otherwise the drive can keep chasing an old CSP target after the operator thinks motion has stopped.

## 9. DS402 / A6-EC Design Principles

### 9.1 Decode state in shared profile code

The repo explicitly settled on:

- DS402 decoding belongs in shared drive profile code,
- EtherCAT AL state decoding belongs in shared fieldbus profile code,
- not in launcher scripts or controller ad hoc logic.

From `cia402.py`:

```python
elif (sw & 0x006F) == 0x0027:
    state = "OperationEnabled"
elif (sw & 0x006F) == 0x0007:
    state = "QuickStopActive"
elif (sw & 0x004F) == 0x0008:
    state = "Fault"
```

From `ethercat.py`:

```python
def describe_master_state(*, link_up: int, responding: int, operational: int, num_axes: int) -> str:
    if int(link_up) and int(responding) > 0:
        return "OP" if int(num_axes) > 0 and int(operational) >= int(num_axes) else "BUS_UP"
    return "DOWN"
```

This design keeps:

- fieldbus interpretation centralized,
- backend-specific fault decoding pluggable,
- telemetry/UI consistent.

### 9.2 Drive fault interpretation must be backend-specific

Another finalized lesson:

- A6-EC-specific error decoding should only be applied when the active live path is actually the A6-EC EtherCAT backend/profile.

This prevents misleading interpretation if another backend/profile is active.

### 9.2A EtherCAT drive bring-up must be descriptor-driven, not hardcoded in RTCore

Another major finalized architecture change was separating:

- robot mechanical policy,
- backend transport behavior,
- and EtherCAT drive-family descriptors.

This matters because the same robot may eventually run with different EtherCAT drive families.

Final rule:

- robot config owns mechanics, scaling, mapping, and limits,
- backend config owns backend behavior/capabilities,
- drive profiles own manufacturer-specific EtherCAT descriptors.

For EtherCAT, the drive catalog is now the source of truth for:

- slave identity (`vendor_id`, `product_code`, `revision_no`),
- sync manager indices,
- DC cycle multiple requirements,
- PDO layout defaults,
- startup SDO defaults and schema.

That source of truth must match the drive's real assigned PDO map exactly. A live A6-EC bring-up failure proved that even a single extra field in the catalog can silently poison RTCore feedback while EtherCAT still appears healthy.

Concrete rule:

- never add a cyclic PDO entry to the catalog unless the configured drive PDO actually includes it,
- and treat PDO entry order/width as safety-critical, not cosmetic.

Example:

```python
ETHERCAT_DRIVE_CATALOG = {
    "a6ec_ds402": {
        "rtcore": {
            "vendor_id": 0x00400000,
            "product_code": 0x00000715,
            "rx_sync_index": 2,
            "tx_sync_index": 3,
            "dc_cycle_multiple_ns": 250000,
            "rx_pdo": 0x1702,
            "tx_pdo": 0x1B02,
            "rx_pdo_layout": [...],
            "tx_pdo_layout": [...],
        },
        "startup_defaults": {
            "a6ec_encoder_position_tracking_mode": 1,
        },
        "startup_schema": {
            "a6ec_encoder_position_tracking_mode": {
                "type": "u16",
                "object": {"index": 0x2000, "subindex": 0x08},
            }
        },
    }
}
```

That catalog is then rendered into the generic RTCore/systemd loader contract. RTCore consumes:

- `--slave-vendor-id`
- `--slave-product-code`
- `--rx-sync-index`
- `--tx-sync-index`
- `--dc-cycle-multiple-ns`
- `--rx-pdo-layout`
- `--tx-pdo-layout`
- `--startup-sdo-config`

Future best practice:

- do not add vendor branches in `main.cpp` for new drives,
- do not let the robot config own EtherCAT manufacturer defaults,
- and do not hand-edit service defaults when the drive catalog should be the source of truth.

### 9.2B Startup verification telemetry must stay generic even when settings are profile-specific

Another finalized lesson was that manufacturer-specific startup settings still need a generic telemetry contract.

The final design is:

- profile modules define setting keys, labels, object addresses, and value labels,
- RTCore reports a generic startup verification object,
- controller telemetry transports that object without inventing backend-specific top-level fields,
- UI displays descriptive labels, not just raw integers.

Example payload shape:

```json
{
  "startup_drive_config": {
    "profile_id": "a6ec_ds402",
    "setting_key": "a6ec_encoder_position_tracking_mode",
    "setting_label": "A6-EC encoder position tracking mode",
    "object": "C00.07 / 0x2000:08",
    "configured": true,
    "commanded": 1,
    "commanded_value_label": "Battery-backed limited multi-turn absolute encoder mode",
    "readback_valid": true,
    "readback": 1,
    "readback_value_label": "Battery-backed limited multi-turn absolute encoder mode",
    "verified": true
  }
}
```

This is a much better final outcome than older vendor-named fields because it:

- keeps the generic controller/UI contract stable,
- lets other drive families add different startup settings without new API surface area,
- and reduces operator confusion by showing descriptive mode names instead of only `0..5`.

### 9.2C Drive fault telemetry must carry both generic bus faults and manufacturer faults

Another important lesson from this work was that a single fault field is not enough for reliable operator diagnosis.

For EtherCAT DS402 drives, the stack should preserve both:

- the generic bus/drive fault code (`0x603F`),
- and the manufacturer-specific fault code when available (for A6-EC, `0x203F`).

Final rule:

- RTCore should publish both raw values,
- controller/API telemetry should forward both raw values,
- profile code should decode them in profile-specific modules,
- frontend/operator views should show both the generic fault context and the manufacturer-specific detail when present.
- but only through an acquisition path that is truthful to the drive's real object map.

This matters because:

- `0x603F` is necessary for portable DS402/EtherCAT reasoning,
- `0x203F` is necessary for precise vendor diagnostics such as battery and multi-turn encoder faults,
- and hiding one or the other creates either ambiguity or loss of actionable detail.

Critical nuance learned in validated live bring-up:

- "carry both" does not mean "force both into the cyclic TxPDO layout",
- for A6-EC fixed TxPDO `0x1B02`, `0x203F` is not part of the assigned cyclic map,
- inserting `manufacturer_err|0x203F|0x00|32` shifted all later feedback fields by 4 bytes,
- the result was a dangerous false picture: EtherCAT reached `OP`, but RTCore misread cyclic feedback and reported `statusword=0`, `pos_counts=0`, and `ds402=NotReady`.

Canonical rule:

- only declare cyclic PDO entries that are actually present in the assigned PDO,
- if a manufacturer-specific diagnostic is needed and is not part of that cyclic PDO, acquire it through a separate non-RT path instead of falsifying the cyclic layout.

Example normalized fault payload shape:

```json
{
  "error_code": 29445,
  "error_code_hex": "0x7305",
  "manufacturer_error_code": 520,
  "manufacturer_error_code_hex": "0x00000208",
  "fault": {
    "decoded": true,
    "bus_fault_name": "Encoder error"
  },
  "manufacturer_fault": {
    "decoded": true,
    "code": "Er20.8",
    "name": "Encoder battery failure"
  }
}
```

Best practice:

- do not collapse manufacturer-specific detail into only a generic DS402 field,
- do not decode manufacturer faults unless the active drive profile actually matches,
- and do not make the frontend guess fault meaning from raw numbers when the profile layer already knows how to interpret them.

### 9.3 Safe enable requires target/feedback synchronization

One of the most important drive-safety rules is:

- before enable, synchronize the commanded hold target to live feedback.

This avoids large target steps during DS402 transitions, which were explicitly associated with avoiding `Er87.*` behavior.

From RTCore comments:

```cpp
// While not operation-enabled (or not wanting enable), keep the target aligned to feedback
// so we don't present a "big step" when DS402 transitions to OperationEnabled (manual: Er87.*).
```

And from controller power-up logic:

- power-up is blocked unless the stack is neutral and synchronized.

## 10. Safe Power-Transition Contract

This is one of the biggest finalized outcomes in the entire stack.

The safe state is no longer “processes happen to be stopped”. It is a structured neutral state.

### 10.1 Neutral-state requirements

The final neutral contract includes:

- no controller motion thread running,
- no active RTCore trajectory,
- no queued RTCore motion,
- no active jog,
- `motion_done=true`,
- `stale_command=false`,
- no live drive faults,
- command targets synchronized to feedback.

### 10.2 Controller-side power guard

From `command_api.py`:

```python
if active_traj_id > 0:
    blockers.append({"code": "active_trajectory", ...})
if queue_depth > 0:
    blockers.append({"code": "queued_motion", ...})
if stale_command:
    blockers.append({"code": "stale_command", ...})
...
if active_jog:
    blockers.append({"code": "active_jog", ...})
if faulted_axis_count > 0:
    blockers.append({"code": "fault_present", ...})
if backend is not None and not feedback_synchronized:
    blockers.append({"code": "not_synchronized", ...})
```

This is a much better final design than earlier implicit “probably safe now” behavior.

### 10.3 STOP must not re-inject motion on RTCore

One of the most subtle and important lessons:

- a generic controller `STOP` path that sends a legacy hold-current-position write is safe for legacy servo backends,
- but unsafe for RTCore-backed EtherCAT because it becomes a one-point RTCore trajectory.

Final fix in `command_api.py`:

```python
if backend is not None and callable(getattr(backend, "get_execution_status", None)):
    print("[Controller] RTCore-backed stop: skipping legacy brake write.")
    return
```

This directly fixed the stale `active_traj_id` post-power-down bug seen during live hardware validation.

### 10.4 Power-up and power-down are first-class operations

The final power transition flow is:

- `power-down`: stop jog, abort trajectory ownership, optionally wait for neutral, disable axes, disarm.
- `power-up`: reject unless `safe_for_power_transition=true`, synchronize targets from feedback, then arm + CSP + enable.

This is the right model for a fieldbus drive system. It treats drive power as a controlled transition, not an incidental side effect of process startup or shutdown.

### 10.5 Validated no-motion power-cycle reference

One of the most important historical validations was the live no-motion power-cycle test:

- hard stop the stack,
- restart headless,
- verify `BUS_UP_DISARMED`,
- verify `/control/motion-status` reports `safe_for_power_transition=true`,
- power up,
- verify all axes `OperationEnabled`,
- send no motion commands,
- power down with wait-for-idle,
- verify return to `BUS_UP_DISARMED`,
- verify `/control/motion-status` returns to `idle` and `safe_for_power_transition=true`.

That sequence is the reference validation for future safety changes. Any change to power transitions, startup, shutdown, STOP semantics, or RTCore motion ownership should be revalidated against this sequence.

## 11. Startup Readiness and Why `startup_ready=1` Matters

Another major lesson from trial and error:

- “socket exists” is not readiness,
- “controller responds” is not readiness,
- “EtherCAT service active” is not readiness,
- “slaves enumerated” is still not full readiness.

Final readiness uses RTCore metrics that roll up:

- link up,
- responding slaves,
- online slaves,
- operational slaves,
- WKC,
- startup pass completion.

This is exactly the kind of derived signal that a multi-layer stack needs. It prevents higher layers from racing ahead just because one component booted faster than the rest.

### 11.1 Startup metrics must be honest about configured expectations

One of the harder debugging lessons was that misleading metrics are worse than missing metrics.

The stack previously had failure modes where diagnostics looked superficially healthy because:

- `wkc_expected` effectively followed observed traffic and could stay `0`,
- or higher-level state was derived from local control state instead of real EtherCAT AL/bus state.

Final principle:

- expected values should be derived from configured topology and protocol expectations,
- actual values should be derived from live fieldbus state,
- and readiness/state summaries must not be inferred from unrelated convenience signals like `armed`.

For future changes, preserve these rules:

- `wkc_expected` should represent the configured expected process-data exchange, not just what has already been seen,
- master/bus state should come from live EtherCAT AL/link/responding/operational state,
- startup readiness should only go true after the startup pass and live process data are both healthy.

This is especially important during partial-failure bring-up, where the controller, RTCore process, and API can all be alive while the fieldbus is still unusable.

## 12. Axis Mapping, Scaling, and Configuration

### 12.1 Direct-order mapping is the final default

This was explicitly corrected during bring-up.

Final policy:

- axis0 -> J1
- axis1 -> J2
- ...
- axisN -> J(N+1)

unless explicitly overridden.

From the backend:

```python
# Default mapping policy: axis0..axisN maps to joint0..jointN in order.
return list(range(min(num_axes, num_joints)))
```

And it is locked by tests.

This replaced older special-case mappings that were useful during partial bring-up but too implicit and fragile to keep as defaults.

### 12.2 Runtime env must be generated from robot/runtime policy

`systemd/rt-motion/sync-runtime.sh` renders the RTCore systemd env from the resolved robot/runtime configuration.

That is a strong finalized design choice because it prevents:

- robot config drift,
- RTCore service defaults diverging from controller/runtime policy,
- drive profile / scaling mismatches after backend changes.

For `Gradient-05`, the robot config is the source of truth for live-axis scaling and direction:

```python
@property
def default_servo_backend(self) -> str:
    return "ethercat_rtcore"

@property
def actuator_encoder_counts_per_rev(self) -> list[int]:
    return [131072] * 6

@property
def actuator_gear_ratios(self) -> list[float]:
    return [100.0, 100.0, 100.0, 18.0, 18.1818181818, 10.0]

@property
def actuator_position_signs(self) -> list[int]:
    return [-1, 1, -1, -1, -1, -1]
```

And RTCore’s installed environment is rendered from that resolved runtime/robot policy, not manually duplicated:

```bash
PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}" "${python_bin}" - "${REPO_ROOT}" <<'PY'
...
resolved = runtime_config.resolve_effective_runtime(...)
robot = get_robot_config(str(resolved.get("robot", {}).get("name", "gradient05")))
print(
    render_rtcore_systemd_env(
        robot_config=robot.get_config_dict(),
        drive_profile=str(drive_profile).strip() or None,
        max_rpm=rtcore_max_rpm,
    ),
    end="",
)
PY
```

This is a key final principle:

- robot geometry, scaling, signs, limits, and default backend policy should live in robot/runtime config,
- the systemd RTCore environment should be generated from that source,
- not maintained as a drifting second copy.

### 12.2A EtherCAT drive-family policy is a separate layer from robot config

The runtime/env generation rules became more precise during the absolute-encoder work.

For EtherCAT RTCore, the rendered service environment is now a merge of:

- robot/runtime policy for axis scaling and high-level selection,
- drive-profile catalog data for EtherCAT manufacturer details.

That split is intentional.

Robot/runtime policy should answer:

- how many axes exist,
- what counts-per-rev / gear ratios / signs are,
- which backend/profile is selected,
- what global RT max RPM policy should be.

Drive-profile catalog policy should answer:

- which EtherCAT slave identity to match,
- which PDO/sync layout to register,
- which startup SDOs to enforce and verify,
- which manufacturer fault tables/labels to decode.

Do not collapse those layers back together.

If a future change makes the same robot unusable with a different EtherCAT drive family unless `RobotConfig` is edited, that is a design regression.

### 12.3 Logical zeroing and drive-native homing are separate operations

The EtherCAT commissioning workflow ultimately converged on two intentionally different concepts:

- logical zeroing (`ZERO_JOINT` / software offsets),
- and drive-native homing (`NATIVE_HOME_JOINT` / drive internal home reference).

Logical zeroing means:

- read live `0x6064` counts,
- convert using RTCore axis config,
- persist repo-local logical master offsets,
- preserve the operator’s logical joint frame without rewriting drive identity every time.

Drive-native homing means:

- explicitly ask the drive to capture its internal reference/home,
- treat that as a drive operation with different semantics and safety considerations,
- expose it clearly in telemetry/UI as a separate action.

Final rule:

- do not blur logical zero with drive-native home,
- do not make a normal “zero” button secretly perform drive EEPROM-style behavior,
- and do not lose the software-offset workflow just because native homing exists.

The current stack intentionally supports both because they solve different problems:

- native home is for the drive’s physical reference model,
- software zero is for the robot/application logical frame.
- but the frontend should bias operators toward native home for normal absolute-encoder commissioning.

Current UI policy:

- keep software zero implemented in the OS/controller/API layers,
- hide the software-zero button from the normal commissioning UI by default,
- and only re-expose it through an explicit Settings toggle when an advanced logical-offset workflow is actually needed.

### 12.3A Commissioning jog/zero/home flows must reuse existing controller pathways and stay conservative

The commissioning workflow accumulated several important practical rules:

- expose per-joint commissioning controls in the existing controller/UI path,
- keep commissioning motion conservative by default,
- and do not invent a separate unsafe control stack just for setup operations.

Final operational model:

- per-joint commissioning jog uses the controller’s existing joint command path,
- per-joint logical zero uses the software-offset capture flow,
- per-joint native home uses the dedicated drive-native home command path,
- jog and native home remain the default visible operator actions,
- while software zero remains supported but is hidden by default behind an explicit UI settings toggle.

Commissioning-specific best practices:

- preserve conservative RTCore speed limits during setup work (for example `--max-rpm 100`),
- treat commissioning joint jog as a small relative absolute-position operation, not a new ad hoc realtime velocity loop,
- reuse backend-aware controller/run-controller pathways rather than bypassing them from the frontend,
- and keep UI messaging explicit about what is logical calibration vs drive-native reference capture.

This distinction prevents a common class of safety and operator-understanding regressions:

- a “jog” path that behaves differently from the rest of the stack,
- a “zero” button that secretly performs drive EEPROM-style behavior,
- or a commissioning-only path that bypasses normal backend safety checks.

### 12.4 Encoder retention verification is a first-class commissioning workflow

Absolute encoders with battery-backed multi-turn retention must be validated with evidence, not assumptions.

The final design added a structured retention experiment workflow:

- capture a snapshot before power-down,
- power-cycle the drives,
- capture a snapshot after power-up,
- compare raw counts, logical angles, startup setting verification, and active battery/multi-turn faults,
- store the artifacts under `logs/encoder-retention/<experiment-id>/`.

The comparison output is intentionally broader than “counts match”:

- raw encoder mismatch,
- logical-angle mismatch,
- startup drive-config mismatch,
- battery/multi-turn fault presence.

Future best practice:

- use retention capture whenever encoder cables, batteries, startup mode defaults, or homing behavior change,
- do not rely on informal operator memory for absolute-encoder validation,
- and keep the generated JSON/Markdown artifacts as commissioning evidence.

## 12A. Robots, Geometry, Tools, and Backend Configuration Model

This section captures the other half of the finalized architecture: where robots are defined, where geometry lives, how tools are modeled, how backends are loaded, and which mistakes already caused drift or safety regressions.

The short version is:

- `RobotConfig` defines runtime control policy for a robot.
- `robots/<robot_id>/robot.json` defines canonical robot assets and geometry entrypoints.
- `tool_library` defines TCP/tool metadata and optional tool meshes.
- `runtime_config` selects which robot/backend/tool is active or desired.
- backend config modules define backend/protocol constants.
- backend instances implement live behavior.

Do not let those layers blur together.

### 12A.1 Source-of-truth hierarchy

Future changes should preserve this source-of-truth stack:

1. `src/gradient_os/arm_controller/robots/*/config.py`
   This is the runtime control definition for a robot:
   - joint counts,
   - actuator IDs,
   - logical-to-physical mapping,
   - software joint limits,
   - gear ratios,
   - encoder counts,
   - axis signs,
   - default backend policy,
   - logical master offsets.
2. `robots/<robot_id>/robot.json`
   This is the canonical asset manifest for that robot:
   - DH CSV,
   - controller URDF,
   - web URDF,
   - web asset source dir,
   - optional OPW URDF,
   - optional numeric/tool-frame metadata.
3. `.gradient_runtime_config.json`
   This is the desired runtime selection layer:
   - active robot choice,
   - live vs sim,
   - active tool,
   - backend/profile overrides,
   - RTCore max RPM override.
4. `tools/library/*`
   This is the tool/TCP library:
   - tool offset semantics,
   - tool compatibility,
   - visualization mesh metadata,
   - weld metadata.
5. `src/gradient_os/arm_controller/backends/*`
   This is backend implementation detail:
   - hardware/protocol constants,
   - telemetry parsing,
   - backend-specific live I/O behavior.

Final principle:

- robot policy, robot geometry, tool geometry, runtime selection, and backend protocol constants are different layers on purpose.
- if one change needs the same value copied into three of those layers, the design is probably drifting.

### 12A.2 How robots are defined

Robots are defined in two linked places:

- a Python `RobotConfig` class under `src/gradient_os/arm_controller/robots/`
- a repo-level asset bundle under `robots/<robot_id>/`

The Python registry in `src/gradient_os/arm_controller/robots/__init__.py` is the runtime selector. The important distinction is:

- `name` is the human/runtime selector, for example `Gradient-05`
- `robot_id` is the stable asset-bundle identifier, for example `gradient-05`

That distinction matters because solver/UI/asset layers resolve manifests by `robot_id`, not by UI label.

Example from `Gradient05Config`:

```python
@property
def robot_id(self) -> str:
    return "gradient-05"

@property
def name(self) -> str:
    return "Gradient-05"

@property
def default_servo_backend(self) -> str:
    return "ethercat_rtcore"
```

Robot configs must own control-relevant properties such as:

- number of logical joints,
- number of physical actuators,
- actuator IDs,
- logical-to-physical mapping,
- logical joint limits,
- actuator limits for compatibility paths,
- gear ratios and counts-per-rev when live scaling depends on them,
- axis sign conventions,
- logical master offsets,
- default backend policy.

Example from `Gradient-05`:

```python
@property
def actuator_encoder_counts_per_rev(self) -> list[int]:
    return [131072] * 6

@property
def actuator_gear_ratios(self) -> list[float]:
    return [100.0, 100.0, 100.0, 18.0, 18.1818181818, 10.0]

@property
def actuator_position_signs(self) -> list[int]:
    return [-1, 1, -1, -1, -1, -1]
```

Just as important are the things that do **not** belong in `RobotConfig`:

- serial packet constants,
- servo register addresses,
- EtherCAT PDO layouts,
- DS402 bit decoding tables,
- protocol-specific telemetry parsers.

Those are backend concerns, not robot-definition concerns.

### 12A.3 Where geometry and kinematics are stored

Canonical robot geometry is stored in the repo-level `robots/<robot_id>/` catalog and loaded through the strict manifest resolver in `src/gradient_os/robot_assets.py`.

The manifest contract is intentionally strict:

- manifest file must be `robots/<robot_id>/robot.json`
- manifest `robot_id` must match the folder name
- required sections are:
  - `kinematics`
  - `models`
  - `web`
- required fields are:
  - `kinematics.dh_csv`
  - `models.controller_urdf`
  - `models.web_urdf`
  - `web.asset_source_dir`
- referenced paths must be relative
- referenced paths must exist
- there must be exactly one manifest with `"default": true`

Selection notes:

- `GRADIENT_ROBOT_ID` can override the default manifest selection
- without that env var, `robot_assets.get_default_robot_id()` requires exactly one default manifest
- manifest resolution is intentionally fail-loud, not “best effort”

This strictness is a feature, not a nuisance. The final architecture intentionally removed “guessy” path fallback behavior.

Example manifest:

```json
{
  "robot_id": "mini-6dof-arm",
  "name": "Mini 6-DOF Arm",
  "kinematics": {
    "dh_csv": "dh_params.csv"
  },
  "models": {
    "controller_urdf": "mini-6dof-arm.urdf",
    "web_urdf": "mini-6dof-arm.urdf",
    "opw_urdf": "opw-mini-arm.urdf"
  },
  "web": {
    "asset_source_dir": ".",
    "urdf": "mini-6dof-arm.urdf"
  }
}
```

Final storage rules:

- DH / solver-side kinematic coefficients live in `dh_params.csv` and related solver metadata.
- URDF is the geometry/joint-frame model for controller/web consumers.
- web meshes and URDF-served assets live under the manifest’s `web.asset_source_dir`.
- optional numeric calibration matrices belong in the manifest, not in ad hoc frontend constants.

The scratchpad lessons here were hard-earned:

- calibrated URDF joint origins from the real robot must be treated as upstream truth, not casually overwritten by old assumptions.
- the runtime clamp source is `robot.logical_joint_limits_rad`, not live URDF parsing during jog/move execution.
- if URDF limits change, sync the robot config deliberately; do not assume runtime consumers are reading the URDF directly.

The current `Gradient-05` robot config says that explicitly:

```python
@property
def logical_joint_limits_rad(self) -> list[tuple[float, float]]:
    """
    Operational contract:
    - Runtime checks consume this property (not URDF parsing at control time).
    - Keep this block synchronized from URDF via:
      `.\\.venv\\Scripts\\python scripts/sync_urdf_limits.py`
    """
```

That is the finalized rule future agents should follow:

- URDF is an upstream geometry reference.
- live controller/runtime clamps come from robot config.
- keep them synchronized intentionally, not implicitly.

### 12A.4 Tool and TCP model

Tool definitions are stored in the repo-local tool library, not embedded inside robot manifests or frontend-only constants.

Current layout:

```text
tools/
  library/
    library.json
    identity/
      tool.json
    tig-torch-65deg/
      tool.json
      tool_mesh.stl
```

The important storage rule is:

- `library.json` stores library-level metadata such as `default_tool_id`
- each tool gets its own folder
- each tool folder contains `tool.json`
- optional mesh assets live beside that `tool.json`

Path/loading notes:

- `GRADIENT_TOOL_LIBRARY_PATH` can point at a custom tool-library root
- if that env var points at a legacy `.json` file, the loader uses its parent directory as the library root for backward compatibility
- `load_tool_library()` still supports migrating old monolithic `tool_library.json` content into per-tool folders

Example tool definition:

```json
{
  "tool_id": "tig-torch-65deg",
  "display_name": "TIG Torch 65deg",
  "tool_type": "tig_torch",
  "compatible_robot_ids": [],
  "offset": {
    "position_mm": { "x": 0.0, "y": 37.5, "z": 347.773 },
    "rotation_deg": { "x": 65.0, "y": 0.0, "z": 0.0 }
  },
  "mesh": {
    "asset_path": "tig-torch-65deg/tool_mesh.stl",
    "scale": 1.0,
    "position_mm": { "x": 0.0, "y": 0.0, "z": 0.0 },
    "rotation_deg": { "x": 0.0, "y": 0.0, "z": 0.0 }
  }
}
```

The runtime selection path is:

1. desired runtime stores `active_tool_id`
2. `runtime_config.resolve_effective_runtime(...)` calls `tool_library.resolve_active_tool(...)`
3. selection order is:
   - requested tool if compatible,
   - library default,
   - identity tool fallback
4. active runtime publishes the resolved tool offset/mesh/weld block

The `identity` tool is not optional. It is the safe null object for “no tool offset”.

Two critical principles came out of the tool-frame debugging:

- the tool `offset` is control/solver TCP semantics
- the tool `mesh` is visualization metadata

Do not collapse those into one thing.

In practice this means:

- if the TCP is wrong, fix tool offset semantics first
- if the visual model is misplaced, fix mesh placement or scene anchoring first
- do not casually edit the robot URDF to fix what is actually a tool-frame bug

The scratchpad lessons here were very specific:

- tool-tip/EE visualization must come from tool offset semantics, not blame unchanged URDF files
- for `Gradient-05`, solver tool offsets are defined in DH-frame semantics while the physical URDF wrist frame is different
- when mapping those frames visually, prefer an explicit matrix mapping over `quaternion.setFromUnitVectors(...)`, because shortest-path quaternion alignment can silently invert secondary axes

The explicit matrix lesson is important enough to preserve verbatim as a pattern:

```python
tool_to_urdf = np.array([
    [0, 0, 1, 0],
    [0, -1, 0, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1],
], dtype=float)
```

That kind of mapping is preferable to “rotate until it looks right”.

### 12A.5 Backend structure, config modules, and loading order

Backend loading has two separate layers and future changes should keep them separate:

1. active backend **config module**
2. active backend **instance**

That is the core contract in `src/gradient_os/arm_controller/backends/registry.py`.

Correct loading order:

```python
backend_registry.set_active_backend(servo_backend_local)
active_backend_local = backend_registry.create_backend(
    servo_backend_local,
    selected_robot_local.get_config_dict(),
)
backend_ready_local = bool(active_backend_local.initialize())
backend_registry.set_active_backend_instance(active_backend_local)
```

That order matters because:

- config-module consumers need protocol/profile constants
- instance consumers need live I/O behavior
- many compatibility paths still expect the config layer to exist even if the backend is not a serial servo backend

Final rule:

- do not instantiate a backend by bypassing the registry
- do not treat “config module loaded” as equivalent to “live backend ready”
- do not monkeypatch one backend into another outside the normal activation path

Registration also stays centralized in `src/gradient_os/arm_controller/backends/__init__.py`:

```python
registry.register_backend_class(
    name="ethercat_rtcore",
    factory=_create_ethercat_rtcore_backend,
    config_module_path="gradient_os.arm_controller.backends.ethercat_rtcore.config",
)
```

That registry is the source of truth for backend availability.

### 12A.6 Servo-specific config belongs in backend config, not robot manifests

The finalized separation is:

- robot config describes the robot
- backend config describes the actuator/protocol family

`src/gradient_os/arm_controller/backends/feetech/config.py` is the model example. It owns:

- packet headers,
- instruction codes,
- register addresses,
- baud rate,
- telemetry block definitions,
- parser helpers,
- default PID values.

Example:

```python
DEFAULT_BAUD_RATE = 1000000
SERVO_INSTRUCTION_SYNC_WRITE = 0x83
SERVO_ADDR_TARGET_POSITION = 0x2A
TELEMETRY_BLOCK1_ADDRESS = 0x38
```

Those values belong to Feetech, not to any specific robot.

The EtherCAT RTCore backend config follows the same principle even though it is not a serial servo stack:

```python
SERVO_PROTOCOL_SUPPORTED = False
DEFAULT_FIELDBUS_PROFILE_ID = ethercat.PROFILE_ID
DEFAULT_DRIVE_PROFILE_ID = a6ec_ds402.PROFILE_ID
```

That config module exists mainly because the older controller stack expects every backend to provide a config module. Future work should preserve that compatibility contract unless the entire legacy path is intentionally removed.

But EtherCAT added an important extra layer beyond the older backend-config pattern:

- backend config tells the controller which fieldbus/drive profile family is in use,
- the EtherCAT drive catalog carries the manufacturer-specific loader data consumed by RTCore.

That separation is now part of the architecture and should be preserved.

For EtherCAT, manufacturer-specific constants should prefer the drive catalog/profile layer over generic backend modules when the values are truly drive-family-specific.

Important special case:

- the simulation backend intentionally reuses Feetech-compatible config constants

```python
from ..feetech.config import *  # noqa: F401,F403
```

This is why future hot-switch or telemetry work must remember that SIM can still look Feetech-shaped at the config-constant level even while runtime mode reports `simulation`.

### 12A.7 Backend behavior should operate on logical joints

The interface contract in `ActuatorBackend` is another core principle worth preserving:

- controllers and planners work in logical-joint space
- backends are responsible for mapping logical joints to physical actuators

From `actuator_interface.py`:

```python
class ActuatorBackend(ABC):
    """
    The interface operates on "logical joints" which represent the kinematic joints
    of the robot. The backend is responsible for mapping these to physical actuators.
    """
```

That rule is why:

- twin-motor mapping belongs in robot/backend mapping logic
- gear ratios belong in robot scaling metadata consumed by the backend
- planner code should not need to know low-level actuator duplication rules

It is also why the SIM hot-switch regression mattered so much:

- `prepare_sync_write_commands()` must emit backend-native low-level commands
- it must not leak a half-converted logical-radian payload into compatibility paths that expect actuator-native tuples

### 12A.8 Runtime configuration is the selector layer, not the definition layer

`src/gradient_os/runtime_config.py` does not define robots or tools. It selects among already-defined ones.

Path/selection notes:

- `GRADIENT_RUNTIME_CONFIG_PATH` can override the runtime config file location
- otherwise the controller uses repo-root `.gradient_runtime_config.json`
- runtime config stores the desired selection, not a second copy of robot/tool definitions

The desired config owns:

- selected robot,
- live vs sim,
- active tool,
- override flags,
- optional backend/profile/RPM overrides.

Example default shape:

```json
{
  "version": 1,
  "desired": {
    "robot": "Gradient-05",
    "sim_mode": false,
    "active_tool_id": null,
    "allow_unsafe_overrides": false,
    "overrides": {
      "ik_solver_backend": null,
      "servo_backend": null,
      "drive_profile": null,
      "rt_max_rpm": 6.0
    }
  }
}
```

The effective runtime then resolves that desired state against:

- robot defaults,
- sim-mode rules,
- backend default drive profiles,
- tool compatibility,
- safety override policy.

Final rule:

- do not turn runtime config into a second robot-definition system
- do not stash permanent robot geometry or backend protocol constants in runtime config
- use it to select, not redefine

### 12A.9 Templates for future robots, tools, and backends

Future developers and agents should follow templates like these.

New robot checklist:

1. Create `robots/<robot_id>/robot.json`
2. Add required assets referenced by the manifest
3. Add `src/gradient_os/arm_controller/robots/<name>/config.py`
4. Register the robot in `src/gradient_os/arm_controller/robots/__init__.py`
5. Keep `robot_id` in Python equal to the manifest folder/manifest `robot_id`
6. Put live limits/scaling/signs in `RobotConfig`
7. If limits originate from URDF, sync them deliberately into `RobotConfig`
8. Validate that the runtime, solver, and web loaders all resolve the same asset bundle

Minimal robot manifest template:

```json
{
  "robot_id": "new-robot",
  "name": "New Robot",
  "description": "Canonical assets for New Robot.",
  "default": false,
  "kinematics": {
    "dh_csv": "dh_params.csv"
  },
  "models": {
    "controller_urdf": "new-robot.urdf",
    "web_urdf": "new-robot.urdf"
  },
  "web": {
    "asset_source_dir": ".",
    "urdf": "new-robot.urdf"
  }
}
```

Minimal `RobotConfig` template:

```python
class NewRobotConfig(RobotConfig):
    @property
    def robot_id(self) -> str:
        return "new-robot"

    @property
    def name(self) -> str:
        return "New Robot"

    @property
    def default_servo_backend(self) -> str:
        return "ethercat_rtcore"

    @property
    def logical_joint_limits_rad(self) -> list[tuple[float, float]]:
        return [...]

    @property
    def actuator_ids(self) -> list[int]:
        return [...]

    @property
    def logical_to_physical_map(self) -> dict[int, list[int]]:
        return {...}
```

New tool checklist:

1. Create `tools/library/<tool_id>/tool.json`
2. Put optional mesh files in the same folder
3. Keep `offset` as TCP semantics
4. Keep `mesh` as visualization semantics
5. Set `compatible_robot_ids` if the tool is robot-specific
6. Verify the tool resolves through `resolve_active_tool(...)`
7. Confirm identity fallback still works if the new tool is incompatible

Minimal tool template:

```json
{
  "tool_id": "new-tool",
  "display_name": "New Tool",
  "description": "What this tool represents.",
  "tool_type": "generic",
  "keywords": [],
  "compatible_robot_ids": [],
  "offset": {
    "position_mm": { "x": 0.0, "y": 0.0, "z": 0.0 },
    "rotation_deg": { "x": 0.0, "y": 0.0, "z": 0.0 }
  },
  "mesh": null,
  "weld": {}
}
```

New backend checklist:

1. Implement the backend class against `ActuatorBackend`
2. Add a backend config module
3. Register the backend factory and config module path in `backends/__init__.py`
4. Ensure `registry.set_active_backend(...)` can load the config module without instantiating hardware
5. Ensure `registry.create_backend(...)` can create the backend from `robot.get_config_dict()`
6. Keep backend behavior in logical-joint terms
7. Decide whether compatibility paths like `servo_driver` need to support it
8. If the backend advertises a capability, implement the full safety semantics behind that capability

Minimal backend config template:

```python
SERVO_PROTOCOL_SUPPORTED = False
DEFAULT_FIELDBUS_PROFILE_ID = "my-fieldbus-profile"
DEFAULT_DRIVE_PROFILE_ID = "my-drive-profile"
```

Minimal backend registration template:

```python
registry.register_backend_class(
    name="my_backend",
    factory=_create_my_backend,
    config_module_path="gradient_os.arm_controller.backends.my_backend.config",
)
```

### 12A.10 The mistakes to avoid

These are the main failure patterns that already cost real time:

- Do not treat URDF parsing at control time as the live safety-limit source. Runtime clamps come from `robot.logical_joint_limits_rad`.
- Do not use URDF edits as the first fix for tool-tip or TCP visualization bugs. First check tool offset semantics and frame anchoring.
- Do not map DH-frame tool semantics into URDF visuals with ad hoc quaternion shortcuts if axis handedness matters. Use an explicit matrix mapping.
- Do not duplicate gear ratio, counts-per-rev, or sign conventions in systemd env files by hand. Generate them from resolved robot/runtime policy.
- Do not put actuator-family protocol constants into robot config.
- Do not put robot-specific geometry or kinematic calibration into backend config.
- Do not bypass the backend registry to instantiate or switch backends.
- Do not let runtime config become a shadow robot-definition system.
- Do not advertise backend capabilities that the backend does not actually implement safely.
- Do not break Feetech compatibility while adding RTCore-specific features unless the migration is explicit and complete.

The safest mental model is:

- manifests describe assets
- robot configs describe runtime robot policy
- tools describe TCP semantics plus optional visuals
- runtime config selects active policy
- backends implement hardware behavior

When those responsibilities stay cleanly separated, future work is much easier to extend without silently damaging safety or performance.

## 13. Building, Installing, and Using the Stack

This section is the practical “how to work on it without breaking it” guide.

### 13.1 EtherCAT host prerequisites

Install the repo-owned host configuration first:

```bash
cd systemd/ethercat-host
./install.sh
```

Optional but recommended for CPU isolation:

```bash
cd systemd/ethercat-host
sudo ./rtos-apply-cmdline.sh
sudo reboot
```

Then install IgH via the repo script:

```bash
./scripts/ethercat/install_igh.sh
```

Validate with:

```bash
./scripts/ethercat/diagnose_host.sh
sudo systemctl enable --now ethercat.service
sudo ethercat master
sudo ethercat slaves -v
```

Principles:

- do not manually hand-edit multiple unrelated host config files when the repo template/render path exists.
- keep NIC role assignment, `ethercat.conf`, and tuning scripts consistent.
- always distinguish raw slave discovery from full RTCore readiness.

### 13.2 Build RTCore locally

Fast path:

```bash
make -C src/gradient_rt_motion
```

Alternative CMake path:

```bash
cd src/gradient_rt_motion
mkdir -p build
cd build
cmake ..
cmake --build . -j
```

Principles:

- use the repo-owned build path, not ad hoc local binaries with unknown flags.
- if testing against real EtherCAT hardware, remember RTCore must run with the required privileges.

### 13.3 Install or refresh RTCore as a service

Preferred install/update path:

```bash
cd systemd/rt-motion
./install.sh
```

To sync updated runtime policy, service file, or binary:

```bash
cd systemd/rt-motion
./sync-runtime.sh --ensure-active
```

Principles:

- do not update `/usr/local/bin/gradient-rt-motion`, `/etc/default/gradient-rt-motion`, and the service unit by hand in different ways.
- use `sync-runtime.sh` so the installed RTCore environment stays aligned with the resolved robot/runtime policy.

### 13.4 Run RTCore manually for bring-up

For low-level bring-up:

```bash
sudo ./src/gradient_rt_motion/gradient-rt-motion --num-axes 2 --max-rpm 100
```

Useful commissioning overrides include:

- `--counts-per-rev`
- `--gear-ratio`
- `--sign`
- `--drive-profile`
- `--max-rpm`

Principles:

- keep `--max-rpm` conservative during commissioning,
- make axis-count and scaling choices explicit,
- and prefer repo-documented overrides over editing hard-coded defaults for temporary bring-up experiments.

### 13.5 Run the full stack

Preferred local/dev launcher:

```bash
./start-stack.sh
```

Headless:

```bash
./start-stack.sh --headless
```

Normal stop:

```bash
./start-stack.sh stop
```

Full stop:

```bash
./start-stack.sh stop --hard
```

Probe physical state:

```bash
./start-stack.sh probe
```

Principles:

- use `start-stack.sh` for staged startup and safety-aware stop behavior,
- not a grab bag of manual process launches,
- unless you are intentionally doing low-level bring-up.

### 13.6 Controller launcher semantics

`run.sh` is the unified controller launcher:

```bash
./run.sh
```

It bootstraps the repo environment, forces the repo `.venv`, and defaults:

```bash
export GRADIENT_RTCORE_AUTO_ARM="${GRADIENT_RTCORE_AUTO_ARM:-0}"
```

That default matters. A future change that flips controller startup back to implicit auto-arm would be a serious behavior change and should not be made casually.

## 14. Compatibility with Feetech and Other Backends

The EtherCAT RTCore path should not destroy the multi-backend architecture.

### 14.1 Backend abstraction is still a core design goal

From `actuator_interface.py`, all backends still share the same high-level logical-joint interface:

- `initialize()`
- `shutdown()`
- `safe_power_down()`
- `safe_power_up()`
- `set_joint_positions()`
- `get_joint_positions()`
- `prepare_sync_write_commands()`
- `sync_write()`
- and related calibration/telemetry methods

This shared contract is what allows:

- Feetech serial backends,
- simulation backends,
- and EtherCAT RTCore

to coexist under one controller.

### 14.2 Servo driver remains a compatibility layer

`servo_driver.py` is intentionally backward-compatible:

```python
# Functions in this module are being migrated to use the ActuatorBackend interface.
# Each function checks if a backend is available and uses it when possible,
# falling back to legacy servo_protocol calls otherwise.
```

That is still important. Future development should continue to:

- keep legacy Feetech behavior working,
- route new generic logic through the backend interface,
- and only specialize behavior when a backend truly needs different semantics.

### 14.3 What should stay generic vs backend-specific

Keep generic:

- controller command vocabulary,
- robot/program semantics,
- motion status shape,
- joint-level APIs,
- save/load formats,
- frontend workflows.

Keep backend-specific:

- transport details,
- hardware scaling,
- RTCore queue/jog mechanics,
- DS402 state handling,
- serial packet/register details,
- drive fault decoding.

### 14.4 Do not break Feetech compatibility while improving RTCore

A useful rule for future changes:

- if the feature is conceptually robot-control generic, implement it through `ActuatorBackend` and controller abstractions first.
- if only RTCore needs a special execution path, add it behind capability checks rather than forking the whole controller contract.

That keeps Feetech, simulation, and RTCore aligned at the user/API/controller level while still allowing RTCore to be more capable underneath.

### 14.5 Simulation backend contract rules learned from hot-switch regressions

The simulation backend is not exempt from the shared backend contract just because it is “only sim”.

The hot-switch work exposed two important contract violations that future devs must avoid reintroducing.

#### Rule 1: `prepare_sync_write_commands()` must emit backend-native low-level commands

The backend interface contract is:

- `prepare_sync_write_commands()` returns backend-ready low-level command tuples,
- `sync_write()` consumes those tuples as backend-native data.

It must **not** return logical radians in a field that `sync_write()` interprets as raw counts or another lower-level representation.

This exact mismatch previously broke SIM after home/direct-setpoint/open-loop flows:

- `prepare_sync_write_commands()` passed logical radians through,
- `sync_write()` interpreted the tuple as low-level/raw position data,
- later planning then started from corrupted simulation state.

Future rule:

- every backend, including SIM, must honor the same low-level command contract its own `sync_write()` expects.

#### Rule 2: capability advertisement must be honest

The joint-velocity lease jog architecture introduced another important rule:

- if a backend advertises joint-velocity lease jog support, it must truly implement:
  - leased velocity application,
  - watchdog timeout behavior,
  - state advancement between updates,
  - stop semantics.

It is not enough to:

- cache the most recent velocity vector,
- or claim support while silently doing nothing.

For SIM specifically, the final correct behavior is:

- integrate leased joint velocity into simulated joint/raw state over time,
- advance state on reads/updates,
- stop advancing once the watchdog deadline expires.

If a backend cannot do that, it should report no support and let the controller fall back rather than pretending to support the capability.

#### Rule 3: backend switching must preserve generic controller semantics

One of the strongest lessons from the hot-switch work is:

- controller semantics should stay stable across Feetech, SIM, and RTCore,
- while backend-specific implementation details stay behind the backend interface.

That means:

- `home`, `move`, `move_absolute`, jog sessions, and direct setpoint flows should remain controller-owned concepts,
- and backend switching should not require special-case frontend behavior.

If a future feature only works because the frontend “knows” it is SIM or “knows” it is Feetech, that is usually a sign the layering is drifting in the wrong direction.

### 14.6 `servo_driver` / `servo_protocol` backend preference rule

Another finalized hot-switch lesson:

- normal SIM mode should rely on the active backend selection already flowing through `servo_driver` and `servo_protocol`,
- not on ad hoc monkeypatch activation.

Future rule:

- preserve the backend registry as the source of truth for active backend routing,
- and avoid reintroducing one-off activation hacks that bypass normal backend selection.

## 15. Frontend and API Thinness Rules

The frontend and API should remain thin by design.

### Frontend should own

- operator workflow,
- input forms,
- state display,
- confirmation UX,
- local visualization/draft state,
- readable blocker messaging.

### API should own

- request validation,
- translation to controller commands,
- response normalization,
- HTTP concerns,
- streaming/monitor transport.

### Neither should own

- backend lifecycle sequencing,
- safe power transition logic,
- live motion ownership,
- runtime hot-switch internals,
- or RT-critical timeout/queue logic.

If you find yourself implementing drive safety state machines in the frontend or API, that is a design smell.

## 16. Feature Placement Rules for Future Devs and Agents

When adding or fixing features, use this placement checklist:

### Put it in the frontend if it is

- purely visual,
- a drafting/editor workflow,
- a user confirmation or warning,
- or a presentation of existing status.

### Put it in the API if it is

- HTTP-only validation/serialization,
- endpoint wiring,
- or controller response normalization.

### Put it in the controller if it is

- robot-policy logic,
- runtime activation,
- cross-backend orchestration,
- motion/program sequencing,
- or controller-owned safety gating.

### Put it in RTCore if it is

- per-cycle motion behavior,
- jog timeout/stop mechanics,
- queued trajectory replay,
- DS402 live command generation,
- live hold-target safety behavior,
- or timing-critical execution truth.

### Put it in the backend abstraction if it must work across backends

- joint-space command semantics,
- power transition hooks,
- calibration hooks,
- status/telemetry access,
- capability reporting.

This placement discipline is one of the main reasons the final architecture is safer and easier to reason about than the earlier versions.

## 17. Timing and Frequency Best Practices

### 13.1 Quantize planner frequency to the RT cycle

The final frequency logic correctly refuses to pretend arbitrary planner timing can run off-cycle.

From the backend:

```python
requested_step_ns = max(1, (1_000_000_000 + requested_hz - 1) // requested_hz)
cycles_per_point = max(1, (requested_step_ns + cycle_ns - 1) // cycle_ns)
step_ns = cycles_per_point * cycle_ns
effective_hz = max(1, 1_000_000_000 // step_ns)
```

Final rule:

- requested timing is advisory,
- RTCore timing is authoritative,
- effective trajectory frequency must align to integer multiples of the RT cycle.

### 13.2 Completion tolerances matter

RTCore also had to accept small final count error:

```cpp
constexpr int32_t kTrajectoryCompletionToleranceCounts = 128;
```

This reflects a realistic understanding of physical drives:

- exact count equality at the final point is not a robust completion criterion.

## 18. What Trial-and-Error Actually Taught Us

These are the highest-signal historical lessons that shaped the final design.

### 14.1 A healthy software stack can hide a dead fieldbus

Observed failure mode:

- controller up,
- API up,
- RTCore up,
- 6 logical axes exposed,
- but `Slaves: 0`, `Rx frames: 0`, `Frame loss: 100%`, raw counts all zero.

Lesson:

- always separate software liveness from bus liveness.

### 14.2 NIC binding assumptions fossilize quickly

Observed failure mode:

- repo templates were bound to an older “known good” NIC,
- live machine wiring had changed,
- stale documentation kept reinforcing the wrong assumption.

Lesson:

- config generation should be single-source and current-machine validated.

### 14.3 Slave discovery is not operational motion readiness

Observed failure mode:

- bare IgH could enumerate slaves,
- RTCore still had zero WKC and non-operational drives.

Lesson:

- separate link/discovery diagnosis from PDO/state/configuration diagnosis.

### 14.3A Wrong PDO descriptors can preserve `OP` while poisoning feedback

Observed failure mode:

- EtherCAT reached `OP`,
- startup readback and slave operational counts looked healthy,
- but all axes still appeared `NotReady` with implausible zeroed cyclic feedback,
- because the configured A6-EC `0x1B02` descriptor incorrectly included `0x203F` and shifted all later offsets by 4 bytes.

Lesson:

- do not stop at "the bus is OP"; verify that cyclic values themselves are plausible,
- when `statusword`, position, and DS402 state are uniformly impossible or flat-zero, suspect descriptor/order errors before blaming the DS402 enable state machine,
- and treat PDO descriptor accuracy as part of the safety contract.

### 14.4 STOP semantics must be backend-aware

Observed failure mode:

- safe power-down physically worked,
- but motion status still reported a completed active trajectory,
- because STOP itself had created a new one-point RTCore trajectory.

Lesson:

- generic control idioms cannot be blindly reused across backends with different execution ownership.

### 14.5 Jog stop behavior needs explicit target collapse

Observed failure mode:

- after jog stop/timeout, axes could keep chasing stale targets.

Lesson:

- for CSP-based RT jog, stopping motion must include hold-target collapse to live feedback, not just “stop sending new velocity”.

### 14.6 Very short trajectories break naive pollers

Observed failure mode:

- controller planned successfully,
- RTCore executed very quickly,
- waiter timed out because `active_traj_id` was never observed long enough.

Lesson:

- command sequence numbers and terminal status are necessary for robust completion handling.

### 14.7 Controller/runtime switching must preserve active policy

Observed failure mode:

- hot-switches risked accidentally applying unrelated pending desired runtime changes.

Lesson:

- derive runtime-switch requests from the active runtime snapshot, then change only the intended dimension.

## 19. Final Best Practices

These are the concrete best practices the repo converged on.

### 15.1 Architecture best practices

- Keep Python policy/planning separate from RT motion execution.
- Keep EtherCAT master ownership inside RTCore.
- Keep UI/API thin around runtime and power transitions.
- Use shared profile modules for DS402 and EtherCAT AL decoding.
- Generate RTCore runtime env from resolved robot/runtime policy, not hand-edited duplicated values.

### 15.2 Safety best practices

- Treat `safe_for_power_transition` as the single power-up gate.
- Always synchronize command targets to live feedback before enable.
- Do not inject legacy hold-position writes on RTCore-backed STOP.
- Keep jog as a timeout-governed RT mode, not a pseudo-trajectory.
- Fail closed if live feedback synchronization is missing.

### 15.3 Bring-up best practices

- Verify NIC binding with current live probes, not stale notes.
- Distinguish `ethercat master` discoverability from RTCore readiness.
- Require `startup_ready=1`, live WKC, and all expected slaves operational before declaring the bus ready.
- Check `/dev/EtherCAT0` ownership / `fuser` when `ecrt_request_master(0)` reports “Device or resource busy”.

### 15.4 Commissioning best practices

- Use direct-order axis mapping by default.
- Keep env override support for partial bring-up, but make it explicit.
- Implement zero capture via software offsets using live RTCore scaling, not drive EEPROM edits.
- Preserve conservative commissioning safety caps like `--max-rpm 100`.

### 15.5 Comms / execution best practices

- Use sequence-stamped command ring messages.
- Publish structured execution states from RTCore.
- Quantize scheduled motion timing to the RT cycle.
- Use command sequence aware waiters for short-trajectory robustness.

## 20. Recommended Mental Model

The final mental model for this stack is:

- The controller decides **what should happen**.
- RTCore decides **when and how motion is actually executed each cycle**.
- EtherCAT host/master configuration decides **whether deterministic fieldbus transport is possible at all**.

When debugging, always ask which layer owns the failure:

- host/NIC/master layer,
- RTCore/fieldbus/state-machine layer,
- or controller/policy/orchestration layer.

Most of the painful trial-and-error came from bugs where one layer looked healthy enough to distract from the real failing layer.

The current architecture is much stronger because those boundaries are now explicit in code, docs, service structure, and telemetry.

## 21. Concise Final Summary

The finalized GradientOS live stack is not “Python controlling EtherCAT directly.” It is:

- a Python controller that owns runtime policy, planning, and sequencing,
- a privileged RTCore daemon that owns realtime EtherCAT execution,
- and a repo-managed IgH host configuration that makes the dedicated fieldbus path deterministic and inspectable.

The most important lessons were:

- do not blur motion ownership across layers,
- do not confuse process startup with bus readiness,
- do not treat accepted commands as completed motion,
- and do not let generic legacy control patterns silently create unsafe behavior on the RTCore path.

That separation of concerns is the real final architecture, and almost every successful fix in the history moved the repo closer to it.
