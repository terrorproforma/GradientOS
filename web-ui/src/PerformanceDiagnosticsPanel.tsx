import { useMemo } from "react";

type ApiCommandPerformance = {
  count?: number;
  ok_count?: number;
  error_count?: number;
  timeout_count?: number;
  avg_round_trip_ms?: number;
  max_round_trip_ms?: number;
  last_round_trip_ms?: number;
};

type ApiUdpPerformance = {
  last_command?: string | null;
  last_result?: string | null;
  last_round_trip_ms?: number | null;
  by_command?: Record<string, ApiCommandPerformance>;
};

type RollingTimingMetric = {
  count?: number;
  avg_ms?: number;
  max_ms?: number;
  last_ms?: number;
  slow_over_5ms?: number;
  slow_over_20ms?: number;
  overrun_count?: number;
  max_overrun_ms?: number;
  last_overrun_ms?: number;
};

type ControllerCommandPerformance = {
  count?: number;
  last_dispatch_ms?: number;
  avg_dispatch_ms?: number;
  max_dispatch_ms?: number;
  last_interarrival_ms?: number | null;
  avg_interarrival_ms?: number;
  max_interarrival_ms?: number;
  interarrival_count?: number;
};

type ControllerUdpPerformance = {
  last_command?: string | null;
  last_peer?: string | null;
  last_dispatch_ms?: number | null;
  last_receive_age_s?: number | null;
  dispatch_ms?: RollingTimingMetric;
  interarrival_ms?: RollingTimingMetric;
  command_counts?: Record<string, number>;
  per_command?: Record<string, ControllerCommandPerformance>;
};

type ControllerPoseSnapshot = {
  position_m?: {
    x?: number;
    y?: number;
    z?: number;
  };
  orientation_euler_deg?: {
    roll?: number;
    pitch?: number;
    yaw?: number;
  };
  joints_deg?: number[];
};

type PoseErrorSnapshot = {
  delta_position_m?: {
    x?: number;
    y?: number;
    z?: number;
  };
  position_error_mm?: number;
  orientation_error_deg?: number;
};

type JogIkDebug = {
  captured_at?: string;
  seq?: number;
  dt_s?: number;
  linear_velocity_m_s?: number[];
  angular_velocity_deg_s?: number[];
  current_pose?: ControllerPoseSnapshot | null;
  target_pose?: ControllerPoseSnapshot | null;
  solved_pose?: ControllerPoseSnapshot | null;
  applied_pose?: ControllerPoseSnapshot | null;
  target_vs_solved?: PoseErrorSnapshot | null;
  target_vs_applied?: PoseErrorSnapshot | null;
  current_joints_deg?: number[] | null;
  ik_solution_joints_deg?: number[] | null;
  applied_joints_deg?: number[] | null;
  clamped_joint_indices?: number[];
  clamped?: boolean;
  solve_failed?: boolean;
};

type JogPerformance = {
  control_frequency_hz?: number;
  execution_policy?: string;
  rtcore_owned?: boolean;
  last_velocity_command_age_s?: number | null;
  velocity_updates?: {
    count?: number;
    gap_count?: number;
    avg_gap_ms?: number;
    max_gap_ms?: number;
    last_gap_ms?: number | null;
    zero_velocity_updates?: number;
    nonzero_velocity_updates?: number;
  };
  loop?: RollingTimingMetric;
  stages?: {
    feedback_read_ms?: RollingTimingMetric;
    ik_solve_ms?: RollingTimingMetric;
    command_send_ms?: RollingTimingMetric;
  };
  ik_debug?: JogIkDebug | null;
};

type RtcorePerformance = {
  rt_last_jitter_ns?: number | null;
  rt_max_abs_jitter_ns?: number | null;
  rt_overrun_count?: number | null;
  motion_last_update_age_ms?: number | null;
};

export type PerformanceResponse = {
  collected_at?: string;
  api_udp?: ApiUdpPerformance;
  controller?: {
    udp?: ControllerUdpPerformance;
    jog?: JogPerformance;
    pose?: ControllerPoseSnapshot;
    jog_session?: {
      session_present?: boolean;
      session_active?: boolean;
      session_id?: string | null;
      owner_id?: string | null;
      state?: string;
      deadman?: boolean;
      paused_for_motion?: boolean;
      lease_timeout_s?: number | null;
      lease_remaining_s?: number | null;
      last_update_age_s?: number | null;
      last_seq_received?: number;
      last_seq_applied?: number;
      lease_expiry_count?: number;
      pause_for_motion_count?: number;
      backend_mode?: string | null;
      backend_timeout_s?: number | null;
      last_stop_reason?: string | null;
      stale_packet_rejects?: number;
      owner_conflict_rejects?: number;
    };
    motion_state?: string;
    is_jogging?: boolean;
    last_command_age_s?: number;
    command_link_stale?: boolean;
  };
  rtcore?: RtcorePerformance | null;
};

export type DiagnosticsPoseHistorySample = {
  collected_at: string;
  motion_state?: string;
  is_jogging?: boolean;
  session_state?: string;
  api_last_command?: string | null;
  controller_last_command?: string | null;
  pose?: ControllerPoseSnapshot | null;
  ik_debug?: JogIkDebug | null;
};

function clamp01(value: number): number {
  if (!Number.isFinite(value)) {
    return 0;
  }
  return Math.max(0, Math.min(1, value));
}

function formatMs(value: number | null | undefined, digits = 2): string {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    return "--";
  }
  return `${value.toFixed(digits)} ms`;
}

function formatAgeSeconds(value: number | null | undefined, digits = 2): string {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    return "--";
  }
  return `${value.toFixed(digits)} s`;
}

function formatNsAsUs(value: number | null | undefined, digits = 2): string {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    return "--";
  }
  return `${(value / 1000).toFixed(digits)} us`;
}

function formatSigned(value: number | null | undefined, digits = 2): string {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    return "--";
  }
  const fixed = value.toFixed(digits);
  return value >= 0 ? `+${fixed}` : fixed;
}

function formatJointRange(values: number[] | undefined, startIndex: number, endIndex: number): string {
  if (!Array.isArray(values) || values.length <= startIndex) {
    return "--";
  }
  return values
    .slice(startIndex, endIndex)
    .map((value, index) => `J${startIndex + index + 1} ${formatSigned(value, 1)} deg`)
    .join(" | ");
}

function formatTimestamp(value: string | undefined): string {
  if (!value) {
    return "--";
  }
  const date = new Date(value);
  if (!Number.isFinite(date.getTime())) {
    return value;
  }
  return date.toLocaleTimeString([], {
    hour12: false,
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    fractionalSecondDigits: 3,
  });
}

function DiagnosticsMetricCard({
  title,
  children,
}: {
  title: string;
  children: React.ReactNode;
}) {
  return (
    <div className="rounded-xl border border-slate-700/60 bg-slate-950/70 p-3 text-[11px] text-slate-300 shadow-inner shadow-slate-950/40">
      <div className="mb-2 text-[10px] font-semibold uppercase tracking-[0.22em] text-cyan-200/80">
        {title}
      </div>
      <div className="space-y-1">{children}</div>
    </div>
  );
}

function DiagnosticsSignalCard({
  title,
  primaryValue,
  secondaryLabel,
  percent,
  tone = "cyan",
}: {
  title: string;
  primaryValue: string;
  secondaryLabel: string;
  percent: number;
  tone?: "cyan" | "violet" | "amber" | "emerald";
}) {
  const toneClasses: Record<string, string> = {
    cyan: "from-cyan-400/80 via-cyan-300/70 to-cyan-500/80 shadow-cyan-500/20",
    violet: "from-violet-400/80 via-violet-300/70 to-violet-500/80 shadow-violet-500/20",
    amber: "from-amber-400/80 via-amber-300/70 to-amber-500/80 shadow-amber-500/20",
    emerald: "from-emerald-400/80 via-emerald-300/70 to-emerald-500/80 shadow-emerald-500/20",
  };

  return (
    <div className="rounded-xl border border-slate-700/60 bg-slate-950/75 p-3 shadow-inner shadow-slate-950/40">
      <div className="text-[10px] font-semibold uppercase tracking-[0.22em] text-slate-400">
        {title}
      </div>
      <div className="mt-2 text-lg font-semibold text-slate-100 tabular-nums">{primaryValue}</div>
      <div className="mt-1 text-[10px] text-slate-500">{secondaryLabel}</div>
      <div className="mt-3 h-2 overflow-hidden rounded-full bg-slate-800/90">
        <div
          className={`h-full rounded-full bg-gradient-to-r ${toneClasses[tone]}`}
          style={{ width: `${clamp01(percent) * 100}%` }}
        />
      </div>
    </div>
  );
}

export function PerformanceDiagnosticsPanel({
  snapshot,
  error,
  poseHistory,
  onClearPoseHistory,
  onSavePoseHistory,
  poseHistorySaveStatus,
}: {
  snapshot: PerformanceResponse | null;
  error: string | null;
  poseHistory: DiagnosticsPoseHistorySample[];
  onClearPoseHistory: () => void;
  onSavePoseHistory: () => void;
  poseHistorySaveStatus: string | null;
}) {
  const snapshotAgeSeconds = useMemo(() => {
    if (!snapshot?.collected_at) {
      return null;
    }
    const collectedAtMs = Date.parse(snapshot.collected_at);
    if (!Number.isFinite(collectedAtMs)) {
      return null;
    }
    return Math.max(0, (Date.now() - collectedAtMs) / 1000);
  }, [snapshot]);

  const apiUdp = snapshot?.api_udp;
  const controller = snapshot?.controller;
  const controllerUdp = controller?.udp;
  const jog = controller?.jog;
  const pose = controller?.pose;
  const jogSession = controller?.jog_session;
  const rtcore = snapshot?.rtcore;
  const hasError = Boolean(error);
  const isStale = !hasError && typeof snapshotAgeSeconds === "number" && snapshotAgeSeconds > 2.0;
  const statusTone = hasError ? "border-amber-400/35 bg-amber-400/10 text-amber-100" : isStale
    ? "border-amber-400/25 bg-amber-400/10 text-amber-100"
    : snapshot
      ? "border-cyan-400/25 bg-cyan-400/10 text-cyan-100"
      : "border-slate-700/60 bg-slate-900/60 text-slate-300";
  const statusLabel = hasError
    ? `offline ${formatAgeSeconds(snapshotAgeSeconds)}`
    : isStale
      ? `stale ${formatAgeSeconds(snapshotAgeSeconds)}`
      : snapshot
        ? `live ${formatAgeSeconds(snapshotAgeSeconds)}`
        : "waiting";

  const moveApi = apiUdp?.by_command?.MOVE_LINE_RELATIVE;
  const waitApi = apiUdp?.by_command?.WAIT_FOR_IDLE;
  const jogApi = apiUdp?.by_command?.JOG_SESSION_UPDATE ?? apiUdp?.by_command?.JOG_SESSION_START;
  const moveController = controllerUdp?.per_command?.MOVE_LINE_RELATIVE;
  const jogController = controllerUdp?.per_command?.JOG_SESSION_UPDATE ?? controllerUdp?.per_command?.JOG_SESSION_START;
  const jogLoop = jog?.loop;
  const jogFeedback = jog?.stages?.feedback_read_ms;
  const jogIk = jog?.stages?.ik_solve_ms;
  const jogSend = jog?.stages?.command_send_ms;
  const ikDebug = jog?.ik_debug;
  const firstHistorySample = poseHistory.length > 0 ? poseHistory[0] : null;
  const lastHistorySample = poseHistory.length > 0 ? poseHistory[poseHistory.length - 1] : null;
  const historyDeltaX =
    typeof firstHistorySample?.pose?.position_m?.x === "number" &&
    typeof lastHistorySample?.pose?.position_m?.x === "number"
      ? lastHistorySample.pose.position_m.x - firstHistorySample.pose.position_m.x
      : null;
  const historyDeltaY =
    typeof firstHistorySample?.pose?.position_m?.y === "number" &&
    typeof lastHistorySample?.pose?.position_m?.y === "number"
      ? lastHistorySample.pose.position_m.y - firstHistorySample.pose.position_m.y
      : null;
  const historyDeltaZ =
    typeof firstHistorySample?.pose?.position_m?.z === "number" &&
    typeof lastHistorySample?.pose?.position_m?.z === "number"
      ? lastHistorySample.pose.position_m.z - firstHistorySample.pose.position_m.z
      : null;
  const historyDeltaRoll =
    typeof firstHistorySample?.pose?.orientation_euler_deg?.roll === "number" &&
    typeof lastHistorySample?.pose?.orientation_euler_deg?.roll === "number"
      ? lastHistorySample.pose.orientation_euler_deg.roll - firstHistorySample.pose.orientation_euler_deg.roll
      : null;
  const historyDeltaPitch =
    typeof firstHistorySample?.pose?.orientation_euler_deg?.pitch === "number" &&
    typeof lastHistorySample?.pose?.orientation_euler_deg?.pitch === "number"
      ? lastHistorySample.pose.orientation_euler_deg.pitch - firstHistorySample.pose.orientation_euler_deg.pitch
      : null;
  const historyDeltaYaw =
    typeof firstHistorySample?.pose?.orientation_euler_deg?.yaw === "number" &&
    typeof lastHistorySample?.pose?.orientation_euler_deg?.yaw === "number"
      ? lastHistorySample.pose.orientation_euler_deg.yaw - firstHistorySample.pose.orientation_euler_deg.yaw
      : null;

  const signalCards = [
    {
      title: "Jog RTT",
      primaryValue: formatMs(jogApi?.avg_round_trip_ms),
      secondaryLabel: `max ${formatMs(jogApi?.max_round_trip_ms)} over UI/API path`,
      percent: clamp01((jogApi?.avg_round_trip_ms ?? 0) / 25),
      tone: "cyan" as const,
    },
    {
      title: "Move ACK",
      primaryValue: formatMs(moveApi?.avg_round_trip_ms),
      secondaryLabel: "includes planning before ACK returns",
      percent: clamp01((moveApi?.avg_round_trip_ms ?? 0) / 1000),
      tone: "amber" as const,
    },
    {
      title: "Controller Dispatch",
      primaryValue: formatMs(controllerUdp?.dispatch_ms?.avg_ms),
      secondaryLabel: `jog max ${formatMs(jogController?.max_dispatch_ms)} | move max ${formatMs(moveController?.max_dispatch_ms)}`,
      percent: clamp01((controllerUdp?.dispatch_ms?.avg_ms ?? 0) / 25),
      tone: "violet" as const,
    },
    {
      title: "RT Jitter",
      primaryValue: formatNsAsUs(rtcore?.rt_last_jitter_ns),
      secondaryLabel: `max ${formatNsAsUs(rtcore?.rt_max_abs_jitter_ns)} | overruns ${rtcore?.rt_overrun_count ?? 0}`,
      percent: clamp01(((rtcore?.rt_last_jitter_ns ?? 0) / 1000) / 50),
      tone: "emerald" as const,
    },
  ];

  return (
    <section className="rounded-2xl border border-cyan-500/20 bg-slate-950/85 p-4 shadow-2xl shadow-slate-950/50 backdrop-blur">
      <div className="flex flex-col gap-3 border-b border-slate-800/60 pb-3 sm:flex-row sm:items-center sm:justify-between">
        <div>
          <div className="text-[10px] font-semibold uppercase tracking-[0.28em] text-cyan-200/75">
            Timing Diagnostics
          </div>
          <div className="mt-1 text-sm font-semibold text-slate-100">
            End-to-end timing below the live charts
          </div>
          <div className="mt-1 text-xs leading-relaxed text-slate-400">
            Captured automatically while the UI is issuing commands. Use this to separate UI/API latency from
            controller planning time and RTCore execution health.
          </div>
        </div>
        <div className="flex items-center gap-2">
          <span className={`rounded-full border px-2.5 py-1 text-[10px] font-semibold uppercase tracking-[0.18em] ${statusTone}`}>
            {statusLabel}
          </span>
        </div>
      </div>

      {error && snapshot ? (
        <div className="mt-3 rounded-xl border border-amber-500/25 bg-amber-500/10 px-3 py-2 text-xs text-amber-100">
          Diagnostics polling is offline. Showing the last successful sample from {formatAgeSeconds(snapshotAgeSeconds)} ago.
          {" "}Last error: {error}
        </div>
      ) : null}
      {error && !snapshot ? (
        <div className="mt-3 rounded-xl border border-rose-500/25 bg-rose-500/10 px-3 py-2 text-xs text-rose-100">
          {error}
        </div>
      ) : null}
      <div className="mt-4 grid gap-3 lg:grid-cols-4">
        {signalCards.map((card) => (
          <DiagnosticsSignalCard
            key={card.title}
            title={card.title}
            primaryValue={card.primaryValue}
            secondaryLabel={card.secondaryLabel}
            percent={card.percent}
            tone={card.tone}
          />
        ))}
      </div>

      <div className="mt-4 grid gap-3 xl:grid-cols-2">
        <DiagnosticsMetricCard title="UI to API timing">
          <div className="tabular-nums">Jog session RTT avg/max: {formatMs(jogApi?.avg_round_trip_ms)} / {formatMs(jogApi?.max_round_trip_ms)}</div>
          <div className="tabular-nums">Move request RTT avg/max: {formatMs(moveApi?.avg_round_trip_ms)} / {formatMs(moveApi?.max_round_trip_ms)}</div>
          <div className="tabular-nums">Wait-for-idle RTT avg/max: {formatMs(waitApi?.avg_round_trip_ms)} / {formatMs(waitApi?.max_round_trip_ms)}</div>
          <div className="tabular-nums">Controller session: {jogSession?.state ?? "idle"} | lease {formatAgeSeconds(jogSession?.lease_remaining_s)} | update age {formatAgeSeconds(jogSession?.last_update_age_s)}</div>
          <div className="pt-1 text-[10px] leading-relaxed text-slate-500">
            `MOVE_LINE_RELATIVE` includes planning before the ACK returns. `WAIT_FOR_IDLE` includes the full motion time.
          </div>
        </DiagnosticsMetricCard>

        <DiagnosticsMetricCard title="Controller UDP dispatch">
          <div className="tabular-nums">Last command: {controllerUdp?.last_command ?? "--"} | dispatch {formatMs(controllerUdp?.last_dispatch_ms)}</div>
          <div className="tabular-nums">Global dispatch avg/max: {formatMs(controllerUdp?.dispatch_ms?.avg_ms)} / {formatMs(controllerUdp?.dispatch_ms?.max_ms)}</div>
          <div className="tabular-nums">Global interarrival last/avg: {formatMs(controllerUdp?.interarrival_ms?.last_ms)} / {formatMs(controllerUdp?.interarrival_ms?.avg_ms)}</div>
          <div className="tabular-nums">Move dispatch avg/max: {formatMs(moveController?.avg_dispatch_ms)} / {formatMs(moveController?.max_dispatch_ms)}</div>
          <div className="tabular-nums">Jog dispatch avg/max: {formatMs(jogController?.avg_dispatch_ms)} / {formatMs(jogController?.max_dispatch_ms)}</div>
          <div className="tabular-nums">Slow dispatch counts &gt;5 ms / &gt;20 ms: {controllerUdp?.dispatch_ms?.slow_over_5ms ?? 0} / {controllerUdp?.dispatch_ms?.slow_over_20ms ?? 0}</div>
          <div className="tabular-nums">Command-link stale: {controller?.command_link_stale ? "yes" : "no"} | last receive age {formatAgeSeconds(controllerUdp?.last_receive_age_s)}</div>
        </DiagnosticsMetricCard>

        <DiagnosticsMetricCard title="Jog loop internals">
          <div className="tabular-nums">Loop avg/max: {formatMs(jogLoop?.avg_ms)} / {formatMs(jogLoop?.max_ms)}</div>
          <div className="tabular-nums">Loop overshoot max: {formatMs(jogLoop?.max_overrun_ms)} | overrun count {jogLoop?.overrun_count ?? 0}</div>
          <div className="tabular-nums">Velocity gap avg/max: {formatMs(jog?.velocity_updates?.avg_gap_ms)} / {formatMs(jog?.velocity_updates?.max_gap_ms)}</div>
          <div className="tabular-nums">Feedback read avg/max: {formatMs(jogFeedback?.avg_ms)} / {formatMs(jogFeedback?.max_ms)}</div>
          <div className="tabular-nums">IK avg/max: {formatMs(jogIk?.avg_ms)} / {formatMs(jogIk?.max_ms)}</div>
          <div className="tabular-nums">Command send avg/max: {formatMs(jogSend?.avg_ms)} / {formatMs(jogSend?.max_ms)}</div>
          <div className="tabular-nums">Jog command age: {formatAgeSeconds(jog?.last_velocity_command_age_s)}</div>
          <div className="tabular-nums">Lease expiries: {jogSession?.lease_expiry_count ?? 0} | stale seq rejects {jogSession?.stale_packet_rejects ?? 0}</div>
        </DiagnosticsMetricCard>

        <DiagnosticsMetricCard title="RTCore health">
          <div className="tabular-nums">RT jitter current/max: {formatNsAsUs(rtcore?.rt_last_jitter_ns)} / {formatNsAsUs(rtcore?.rt_max_abs_jitter_ns)}</div>
          <div className="tabular-nums">RT overruns: {rtcore?.rt_overrun_count ?? 0} | motion update age {formatMs(rtcore?.motion_last_update_age_ms)}</div>
          <div className="tabular-nums">Controller motion state: {(controller?.motion_state ?? "--").toUpperCase()}</div>
          <div className="tabular-nums">Controller jogging: {controller?.is_jogging ? "yes" : "no"} | exec policy {jogSession?.backend_mode ?? jog?.execution_policy ?? "--"}</div>
          <div className="tabular-nums">Owner conflicts: {jogSession?.owner_conflict_rejects ?? 0} | last stop {jogSession?.last_stop_reason ?? "--"}</div>
        </DiagnosticsMetricCard>

        <DiagnosticsMetricCard title="Controller pose snapshot">
          <div className="tabular-nums">
            XYZ (m): x {formatSigned(pose?.position_m?.x, 4)} | y {formatSigned(pose?.position_m?.y, 4)} | z {formatSigned(pose?.position_m?.z, 4)}
          </div>
          <div className="tabular-nums">
            RPY (deg): r {formatSigned(pose?.orientation_euler_deg?.roll, 2)} | p {formatSigned(pose?.orientation_euler_deg?.pitch, 2)} | y {formatSigned(pose?.orientation_euler_deg?.yaw, 2)}
          </div>
          <div className="tabular-nums">Joints 1-3: {formatJointRange(pose?.joints_deg, 0, 3)}</div>
          <div className="tabular-nums">Joints 4-6: {formatJointRange(pose?.joints_deg, 3, 6)}</div>
          <div className="pt-1 text-[10px] leading-relaxed text-slate-500">
            Sampled from controller `GET_POSITION` alongside the timing snapshot so the last successful sample remains available after a run stops.
          </div>
        </DiagnosticsMetricCard>

        <DiagnosticsMetricCard title="Jog IK readout">
          {ikDebug ? (
            <>
              <div className="tabular-nums">
                Seq {ikDebug.seq ?? "--"} | dt {typeof ikDebug.dt_s === "number" ? `${(ikDebug.dt_s * 1000).toFixed(1)} ms` : "--"}
              </div>
              <div className="tabular-nums">
                Target XYZ (m): x {formatSigned(ikDebug.target_pose?.position_m?.x, 4)} | y {formatSigned(ikDebug.target_pose?.position_m?.y, 4)} | z {formatSigned(ikDebug.target_pose?.position_m?.z, 4)}
              </div>
              <div className="tabular-nums">
                Solved err: {formatSigned(ikDebug.target_vs_solved?.position_error_mm, 2)} mm | {formatSigned(ikDebug.target_vs_solved?.orientation_error_deg, 3)} deg
              </div>
              <div className="tabular-nums">
                Applied err: {formatSigned(ikDebug.target_vs_applied?.position_error_mm, 2)} mm | {formatSigned(ikDebug.target_vs_applied?.orientation_error_deg, 3)} deg
              </div>
              <div className="tabular-nums">
                Clamp: {ikDebug.clamped ? `yes (${(ikDebug.clamped_joint_indices ?? []).map((value) => `J${value + 1}`).join(", ")})` : "no"} | solve failed {ikDebug.solve_failed ? "yes" : "no"}
              </div>
              <div className="tabular-nums">Current joints: {formatJointRange(ikDebug.current_joints_deg ?? undefined, 0, 6)}</div>
              <div className="tabular-nums">Solved joints: {formatJointRange(ikDebug.ik_solution_joints_deg ?? undefined, 0, 6)}</div>
              <div className="tabular-nums">Applied joints: {formatJointRange(ikDebug.applied_joints_deg ?? undefined, 0, 6)}</div>
            </>
          ) : (
            <div className="text-slate-500">No jog IK sample captured yet.</div>
          )}
        </DiagnosticsMetricCard>
      </div>

      <div className="mt-4 rounded-xl border border-slate-700/60 bg-slate-950/70 p-3 shadow-inner shadow-slate-950/40">
        <div className="flex flex-col gap-3 border-b border-slate-800/60 pb-3 sm:flex-row sm:items-start sm:justify-between">
          <div>
            <div className="text-[10px] font-semibold uppercase tracking-[0.22em] text-cyan-200/80">
              Pose History
            </div>
            <div className="mt-1 text-xs leading-relaxed text-slate-400">
              Full captured pose/joint timeline from diagnostics polling. It now saves locally on the Pi so you can inspect runs later without browser export.
            </div>
            {poseHistorySaveStatus ? (
              <div className="mt-2 text-[10px] font-medium tracking-[0.08em] text-cyan-200/80">{poseHistorySaveStatus}</div>
            ) : null}
          </div>
          <div className="flex flex-wrap gap-2">
            <button
              type="button"
              onClick={onSavePoseHistory}
              disabled={poseHistory.length === 0}
              className={`rounded-full border px-3 py-1.5 text-[10px] font-semibold uppercase tracking-[0.18em] ${
                poseHistory.length === 0
                  ? "border-slate-700/60 bg-slate-900/50 text-slate-500"
                  : "border-cyan-500/40 bg-cyan-500/10 text-cyan-100 hover:border-cyan-300/70"
              }`}
            >
              Save Local JSON
            </button>
            <button
              type="button"
              onClick={onClearPoseHistory}
              disabled={poseHistory.length === 0}
              className={`rounded-full border px-3 py-1.5 text-[10px] font-semibold uppercase tracking-[0.18em] ${
                poseHistory.length === 0
                  ? "border-slate-700/60 bg-slate-900/50 text-slate-500"
                  : "border-slate-600/70 bg-slate-900/70 text-slate-200 hover:border-slate-400/80 hover:text-white"
              }`}
            >
              Clear History
            </button>
          </div>
        </div>

        <div className="mt-3 grid gap-3 xl:grid-cols-2">
          <DiagnosticsMetricCard title="History summary">
            <div className="tabular-nums">Samples captured: {poseHistory.length}</div>
            <div className="tabular-nums">Start: {formatTimestamp(firstHistorySample?.collected_at)}</div>
            <div className="tabular-nums">End: {formatTimestamp(lastHistorySample?.collected_at)}</div>
            <div className="tabular-nums">
              Delta XYZ (m): x {formatSigned(historyDeltaX, 4)} | y {formatSigned(historyDeltaY, 4)} | z {formatSigned(historyDeltaZ, 4)}
            </div>
            <div className="tabular-nums">
              Delta RPY (deg): r {formatSigned(historyDeltaRoll, 2)} | p {formatSigned(historyDeltaPitch, 2)} | y {formatSigned(historyDeltaYaw, 2)}
            </div>
            <div className="tabular-nums">
              Last state: {(lastHistorySample?.motion_state ?? "--").toUpperCase()} | jogging {lastHistorySample?.is_jogging ? "yes" : "no"} | session {lastHistorySample?.session_state ?? "--"}
            </div>
          </DiagnosticsMetricCard>

          <DiagnosticsMetricCard title="Latest captured joints">
            <div className="tabular-nums">Joints 1-3: {formatJointRange(lastHistorySample?.pose?.joints_deg, 0, 3)}</div>
            <div className="tabular-nums">Joints 4-6: {formatJointRange(lastHistorySample?.pose?.joints_deg, 3, 6)}</div>
            <div className="tabular-nums">API last command: {lastHistorySample?.api_last_command ?? "--"}</div>
            <div className="tabular-nums">Controller last command: {lastHistorySample?.controller_last_command ?? "--"}</div>
          </DiagnosticsMetricCard>
        </div>

        <div className="mt-3 overflow-hidden rounded-xl border border-slate-800/70">
          <div className="max-h-[28rem] overflow-auto">
            <table className="min-w-full text-left text-[11px] text-slate-300">
              <thead className="sticky top-0 bg-slate-950/95 text-[10px] uppercase tracking-[0.18em] text-slate-400 backdrop-blur">
                <tr>
                  <th className="px-3 py-2">Time</th>
                  <th className="px-3 py-2">State</th>
                  <th className="px-3 py-2">XYZ (m)</th>
                  <th className="px-3 py-2">RPY (deg)</th>
                  <th className="px-3 py-2">J1-3</th>
                  <th className="px-3 py-2">J4-6</th>
                </tr>
              </thead>
              <tbody>
                {poseHistory.length === 0 ? (
                  <tr>
                    <td colSpan={6} className="px-3 py-4 text-center text-slate-500">
                      No pose history captured yet.
                    </td>
                  </tr>
                ) : (
                  poseHistory.map((sample, index) => (
                    <tr key={`${sample.collected_at}:${index}`} className="border-t border-slate-800/70 align-top">
                      <td className="px-3 py-2 font-medium text-slate-200">{formatTimestamp(sample.collected_at)}</td>
                      <td className="px-3 py-2">
                        <div>{(sample.motion_state ?? "--").toUpperCase()}</div>
                        <div className="text-[10px] text-slate-500">
                          jog {sample.is_jogging ? "yes" : "no"} | {sample.session_state ?? "--"}
                        </div>
                      </td>
                      <td className="px-3 py-2 tabular-nums">
                        x {formatSigned(sample.pose?.position_m?.x, 4)}
                        <br />
                        y {formatSigned(sample.pose?.position_m?.y, 4)}
                        <br />
                        z {formatSigned(sample.pose?.position_m?.z, 4)}
                      </td>
                      <td className="px-3 py-2 tabular-nums">
                        r {formatSigned(sample.pose?.orientation_euler_deg?.roll, 2)}
                        <br />
                        p {formatSigned(sample.pose?.orientation_euler_deg?.pitch, 2)}
                        <br />
                        y {formatSigned(sample.pose?.orientation_euler_deg?.yaw, 2)}
                      </td>
                      <td className="px-3 py-2 tabular-nums">{formatJointRange(sample.pose?.joints_deg, 0, 3)}</td>
                      <td className="px-3 py-2 tabular-nums">{formatJointRange(sample.pose?.joints_deg, 3, 6)}</td>
                    </tr>
                  ))
                )}
              </tbody>
            </table>
          </div>
        </div>
      </div>
    </section>
  );
}
