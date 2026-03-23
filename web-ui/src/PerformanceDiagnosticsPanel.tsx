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
}: {
  snapshot: PerformanceResponse | null;
  error: string | null;
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
      </div>
    </section>
  );
}
