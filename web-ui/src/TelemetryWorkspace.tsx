import { useEffect, useState } from "react";
import type { TelemetryEvent } from "./TelemetryCharts";
import { TelemetryCharts } from "./TelemetryCharts";
import { PerformanceDiagnosticsPanel } from "./PerformanceDiagnosticsPanel";
import type { PerformanceResponse } from "./PerformanceDiagnosticsPanel";

type TelemetryTab = "charts" | "diagnostics";
const PERFORMANCE_POLL_MS = 500;

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
  latest: TelemetryEvent | null;
  apiHost: string;
}) {
  const [activeTab, setActiveTab] = useState<TelemetryTab>("charts");
  const [diagnosticsVisible, setDiagnosticsVisible] = useState<boolean>(true);
  const [diagnosticsSnapshot, setDiagnosticsSnapshot] = useState<PerformanceResponse | null>(null);
  const [diagnosticsError, setDiagnosticsError] = useState<string | null>(null);

  useEffect(() => {
    if (!diagnosticsVisible && activeTab === "diagnostics") {
      setActiveTab("charts");
    }
  }, [activeTab, diagnosticsVisible]);

  useEffect(() => {
    let disposed = false;
    let inFlight = false;
    const tick = async () => {
      if (disposed || inFlight) {
        return;
      }
      inFlight = true;
      try {
        const res = await fetch(`${apiHost}/debug/performance`, {
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
        }
      } catch (err) {
        if (!disposed) {
          setDiagnosticsError((err as Error)?.message || "Failed to load timing diagnostics.");
        }
      } finally {
        inFlight = false;
      }
    };
    void tick();
    const timer = window.setInterval(() => {
      void tick();
    }, PERFORMANCE_POLL_MS);
    return () => {
      disposed = true;
      window.clearInterval(timer);
    };
  }, [apiHost]);

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
        <TelemetryCharts latest={latest} />
      ) : (
        <PerformanceDiagnosticsPanel snapshot={diagnosticsSnapshot} error={diagnosticsError} />
      )}
    </div>
  );
}
