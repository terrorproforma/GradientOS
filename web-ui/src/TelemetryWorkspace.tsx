import { useCallback, useEffect, useRef, useState } from "react";
import type { TelemetryEvent } from "./TelemetryCharts";
import { TelemetryCharts } from "./TelemetryCharts";
import { PerformanceDiagnosticsPanel } from "./PerformanceDiagnosticsPanel";
import type { DiagnosticsPoseHistorySample, PerformanceResponse } from "./PerformanceDiagnosticsPanel";
import { useOptionalLiveState } from "./liveState";

type TelemetryTab = "charts" | "diagnostics";
const ACTIVE_PERFORMANCE_POLL_MS = 250;
const IDLE_PERFORMANCE_POLL_MS = 1000;
const POSE_HISTORY_LIMIT = 1200;

function isMotionActive(snapshot: PerformanceResponse | null): boolean {
  if (!snapshot) {
    return false;
  }
  const controller = snapshot.controller;
  if (!controller) {
    return false;
  }
  if (controller.is_jogging) {
    return true;
  }
  const state = String(controller.motion_state ?? "").toLowerCase();
  if (state && !["idle", "ready", "complete", "completed", "stopped"].includes(state)) {
    return true;
  }
  return false;
}

function buildPoseHistorySample(snapshot: PerformanceResponse): DiagnosticsPoseHistorySample | null {
  const pose = snapshot.controller?.pose;
  if (!pose) {
    return null;
  }
  return {
    collected_at: snapshot.collected_at,
    motion_state: snapshot.controller?.motion_state ?? undefined,
    is_jogging: snapshot.controller?.is_jogging ?? false,
    session_state: snapshot.controller?.jog_session?.state ?? undefined,
    api_last_command: snapshot.api_udp?.last_command ?? null,
    controller_last_command: snapshot.controller?.udp?.last_command ?? null,
    pose,
    ik_debug: snapshot.controller?.jog?.ik_debug ?? null,
  };
}

function appendPoseHistorySample(
  history: DiagnosticsPoseHistorySample[],
  sample: DiagnosticsPoseHistorySample | null,
): DiagnosticsPoseHistorySample[] {
  if (!sample) {
    return history;
  }
  const last = history.length > 0 ? history[history.length - 1] : null;
  if (
    last &&
    last.collected_at === sample.collected_at &&
    JSON.stringify(last.pose) === JSON.stringify(sample.pose) &&
    JSON.stringify(last.ik_debug) === JSON.stringify(sample.ik_debug) &&
    last.motion_state === sample.motion_state &&
    last.is_jogging === sample.is_jogging &&
    last.session_state === sample.session_state
  ) {
    return history;
  }
  const next = [...history, sample];
  return next.length > POSE_HISTORY_LIMIT ? next.slice(-POSE_HISTORY_LIMIT) : next;
}

async function readErrorMessage(res: Response): Promise<string> {
  const contentType = res.headers.get("content-type") ?? "";
  if (contentType.includes("application/json")) {
    try {
      const payload = (await res.json()) as { detail?: unknown; message?: unknown };
      if (payload && typeof payload === "object") {
        if (
          payload.detail &&
          typeof payload.detail === "object" &&
          typeof (payload.detail as { message?: unknown }).message === "string"
        ) {
          return String((payload.detail as { message: string }).message);
        }
        if (typeof payload.detail === "string") {
          return payload.detail;
        }
        if (typeof payload.message === "string") {
          return payload.message;
        }
        return JSON.stringify(payload);
      }
    } catch {
      // Fall through to plain text handling.
    }
  }
  try {
    const text = (await res.text()).trim();
    if (text) {
      return text;
    }
  } catch {
    // Ignore read failures and use the HTTP status line.
  }
  return `${res.status} ${res.statusText}`;
}

export function TelemetryWorkspace({
  latest,
  apiHost,
}: {
  latest?: TelemetryEvent | null;
  apiHost?: string;
}) {
  const liveState = useOptionalLiveState();
  const resolvedLatest = latest ?? liveState?.latest ?? null;
  const resolvedApiHost = apiHost ?? liveState?.apiHost ?? "";
  const [activeTab, setActiveTab] = useState<TelemetryTab>("charts");
  const [diagnosticsVisible, setDiagnosticsVisible] = useState<boolean>(true);
  const [diagnosticsSnapshot, setDiagnosticsSnapshot] = useState<PerformanceResponse | null>(null);
  const [diagnosticsError, setDiagnosticsError] = useState<string | null>(null);
  const [poseHistory, setPoseHistory] = useState<DiagnosticsPoseHistorySample[]>([]);
  const [poseHistorySaveStatus, setPoseHistorySaveStatus] = useState<string | null>(null);
  const lastAutoSavedStopAtRef = useRef<string | null>(null);
  const diagnosticsPollingEnabled = diagnosticsVisible;

  useEffect(() => {
    if (!diagnosticsVisible && activeTab === "diagnostics") {
      setActiveTab("charts");
    }
  }, [activeTab, diagnosticsVisible]);

  useEffect(() => {
    if (!diagnosticsPollingEnabled) {
      return;
    }
    if (!resolvedApiHost) {
      setDiagnosticsError("Live API host unavailable.");
      return;
    }
    let disposed = false;
    let inFlight = false;
    let timer: number | null = null;

    const scheduleNext = (delayMs: number) => {
      if (disposed) {
        return;
      }
      timer = window.setTimeout(() => {
        void tick();
      }, delayMs);
    };

    const tick = async () => {
      if (disposed || inFlight) {
        return;
      }
      inFlight = true;
      try {
        const res = await fetch(`${resolvedApiHost}/debug/performance`, {
          method: "GET",
          headers: { Accept: "application/json" },
        });
        if (!res.ok) {
          throw new Error(await readErrorMessage(res));
        }
        const payload = (await res.json()) as PerformanceResponse;
        if (!disposed) {
          setDiagnosticsSnapshot(payload);
          setDiagnosticsError(null);
          setPoseHistory((current) => appendPoseHistorySample(current, buildPoseHistorySample(payload)));
        }
        scheduleNext(isMotionActive(payload) ? ACTIVE_PERFORMANCE_POLL_MS : IDLE_PERFORMANCE_POLL_MS);
      } catch (err) {
        if (!disposed) {
          setDiagnosticsError((err as Error)?.message || "Failed to load timing diagnostics.");
        }
        scheduleNext(IDLE_PERFORMANCE_POLL_MS);
      } finally {
        inFlight = false;
      }
    };
    void tick();
    return () => {
      disposed = true;
      if (timer !== null) {
        window.clearTimeout(timer);
      }
    };
  }, [diagnosticsPollingEnabled, resolvedApiHost]);

  const persistPoseHistory = useCallback(
    async (history: DiagnosticsPoseHistorySample[], source: "manual" | "auto-stop") => {
      if (history.length === 0) {
        return;
      }
      if (!resolvedApiHost) {
        setPoseHistorySaveStatus("Local save unavailable: API host missing.");
        return;
      }
      try {
        const res = await fetch(`${resolvedApiHost}/debug/pose-history`, {
          method: "POST",
          headers: {
            Accept: "application/json",
            "Content-Type": "application/json",
          },
          body: JSON.stringify({
            exported_at: new Date().toISOString(),
            source,
            sample_count: history.length,
            pose_history: history,
          }),
        });
        if (!res.ok) {
          throw new Error(await readErrorMessage(res));
        }
        const payload = (await res.json()) as { path?: unknown; sample_count?: unknown };
        setPoseHistorySaveStatus(
          `Saved locally: ${String(payload.path ?? "--")} (${String(payload.sample_count ?? history.length)} samples)`,
        );
      } catch (err) {
        setPoseHistorySaveStatus((err as Error)?.message ? `Local save failed: ${(err as Error).message}` : "Local save failed.");
      }
    },
    [resolvedApiHost],
  );

  useEffect(() => {
    const last = poseHistory.length > 0 ? poseHistory[poseHistory.length - 1] : null;
    const previous = poseHistory.length > 1 ? poseHistory[poseHistory.length - 2] : null;
    const runJustStopped =
      last?.session_state === "stopped" && Boolean(previous?.is_jogging || previous?.session_state === "active");
    if (!runJustStopped || !last) {
      return;
    }
    if (lastAutoSavedStopAtRef.current === last.collected_at) {
      return;
    }
    lastAutoSavedStopAtRef.current = last.collected_at;
    void persistPoseHistory(poseHistory, "auto-stop");
  }, [persistPoseHistory, poseHistory]);

  const tabs: Array<{ id: TelemetryTab; label: string }> = diagnosticsVisible
    ? [
        { id: "charts", label: "Live Charts" },
        { id: "diagnostics", label: "Diagnostics" },
      ]
    : [{ id: "charts", label: "Live Charts" }];

  return (
    <div className="space-y-3">
      <div className="rounded-2xl border border-slate-700/60 bg-slate-950/80 p-2 shadow-xl shadow-slate-950/40">
        <div className="flex flex-col gap-2 sm:flex-row sm:items-center sm:justify-between">
          <div className="flex flex-wrap items-center gap-2">
            {tabs.map((tab) => {
              const isActive = activeTab === tab.id;
              return (
                <button
                  key={tab.id}
                  type="button"
                  onClick={() => setActiveTab(tab.id)}
                  className={`rounded-full px-3 py-1.5 text-[10px] font-semibold uppercase tracking-[0.22em] transition ${
                    isActive
                      ? "border border-cyan-400/45 bg-cyan-400/12 text-cyan-100 shadow-lg shadow-cyan-500/10"
                      : "border border-slate-700/70 bg-slate-900/70 text-slate-300 hover:border-slate-500/80 hover:text-slate-100"
                  }`}
                >
                  {tab.label}
                </button>
              );
            })}
          </div>
          <button
            type="button"
            onClick={() => setDiagnosticsVisible((current) => !current)}
            className="rounded-full border border-slate-700/70 bg-slate-900/70 px-3 py-1.5 text-[10px] font-semibold uppercase tracking-[0.18em] text-slate-200 transition hover:border-slate-500/80 hover:text-white"
          >
            {diagnosticsVisible ? "Hide Diagnostics Tab" : "Show Diagnostics Tab"}
          </button>
        </div>
      </div>

      {activeTab === "charts" ? (
        <TelemetryCharts latest={resolvedLatest} />
      ) : (
        <PerformanceDiagnosticsPanel
          snapshot={diagnosticsSnapshot}
          error={diagnosticsError}
          poseHistory={poseHistory}
          onClearPoseHistory={() => {
            setPoseHistory([]);
            setPoseHistorySaveStatus(null);
            lastAutoSavedStopAtRef.current = null;
          }}
          onSavePoseHistory={() => {
            void persistPoseHistory(poseHistory, "manual");
          }}
          poseHistorySaveStatus={poseHistorySaveStatus}
        />
      )}
    </div>
  );
}
