import { useEffect, useState } from "react";
import type { TelemetryEvent } from "./TelemetryCharts";
import { TelemetryCharts } from "./TelemetryCharts";
import { PerformanceDiagnosticsPanel } from "./PerformanceDiagnosticsPanel";

type TelemetryTab = "charts" | "diagnostics";

export function TelemetryWorkspace({
  latest,
  apiHost,
}: {
  latest: TelemetryEvent | null;
  apiHost: string;
}) {
  const [activeTab, setActiveTab] = useState<TelemetryTab>("charts");
  const [diagnosticsVisible, setDiagnosticsVisible] = useState<boolean>(true);

  useEffect(() => {
    if (!diagnosticsVisible && activeTab === "diagnostics") {
      setActiveTab("charts");
    }
  }, [activeTab, diagnosticsVisible]);

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
        <PerformanceDiagnosticsPanel apiHost={apiHost} />
      )}
    </div>
  );
}
