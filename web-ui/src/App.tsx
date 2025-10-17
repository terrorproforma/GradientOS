import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  Camera,
  CameraOff,
  Home,
  Octagon,
  Play,
  Plug,
  RefreshCcw,
  Route,
  Settings,
  Trash2,
  Unplug,
  Undo2,
  X,
} from "lucide-react";
import { resolveDefaultApiHost, resolveDefaultVisionHost } from "./useEndpoint";
import { ArmVisualizer, type ArmVisualizerHandle } from "./ArmVisualizer";
import {
  encodePointsForApi,
  previewFromPlannerPayload,
  previewFromTrajectoryDetail,
  type Point3,
  type PreviewPlan,
} from "./previewUtils";

type TelemetryEvent = {
  timestamp: number;
  raw: string;
  joints?: number[];
  gripper?: number;
};

function normaliseApiHost(input: string): string {
  const trimmed = input.trim();
  if (!trimmed) {
    return resolveDefaultApiHost();
  }
  return trimmed.replace(/\/+$/, "");
}

function normaliseVisionHost(input: string): string {
  const trimmed = input.trim();
  if (!trimmed) {
    return resolveDefaultVisionHost();
  }
  return trimmed.replace(/\/+$/, "");
}

function TelemetryPanel({ latest }: { latest: TelemetryEvent | null }) {
  return (
    <div className="pointer-events-auto w-full max-w-xs rounded-xl border border-slate-700/60 bg-slate-900/80 p-4 shadow-lg shadow-slate-900/50 backdrop-blur-lg">
      {latest ? (
        <div className="flex flex-col gap-3 text-sm">
          <div className="flex items-center justify-between text-xs text-slate-300/80">
            <span className="font-medium text-slate-100">Received</span>
            <span>{new Date(latest.timestamp).toLocaleTimeString()}</span>
          </div>
          {latest.joints && latest.joints.length > 0 && (
            <div className="flex flex-col gap-2">
              <span className="text-xs font-semibold uppercase tracking-[0.2em] text-cyan-300/80">
                Joints (deg)
              </span>
              <ul className="grid grid-cols-2 gap-x-4 gap-y-1 text-sm text-slate-100/90">
                {latest.joints.map((value, index) => {
                  const degrees = value * (180 / Math.PI);
                  return (
                    <li key={index}>
                      <span className="text-slate-400">J{index + 1}:</span>{" "}
                      {degrees.toFixed(1)}
                    </li>
                  );
                })}
              </ul>
            </div>
          )}
          {typeof latest.gripper === "number" && (
            <div className="text-sm text-slate-100/90">
              <span className="font-semibold text-cyan-200">Gripper</span>{" "}
              {latest.gripper.toFixed(3)}
            </div>
          )}
        </div>
      ) : (
        <p className="text-sm text-slate-300/80">
          No telemetry yet. Connect to the API and start streaming.
        </p>
      )}
    </div>
  );
}

type TrajectoryPanelProps = {
  isPlanning: boolean;
  isPlanLoading: boolean;
  isRunning: boolean;
  preview: PreviewPlan | null;
  plannerPoints: Point3[];
  savedTrajectories: string[];
  selectedTrajectory: string;
  isTrajectoryListLoading: boolean;
  isLoadingSavedTrajectory: boolean;
  onPlanToggle: () => void;
  onRun: () => void;
  onClear: () => void;
  onRefreshTrajectories: () => void;
  onSelectTrajectory: (value: string) => void;
  onLoadTrajectory: () => void;
  onUndoPoint: () => void;
};

function TrajectoryPanel({
  isPlanning,
  isPlanLoading,
  isRunning,
  preview,
  plannerPoints,
  savedTrajectories,
  selectedTrajectory,
  isTrajectoryListLoading,
  isLoadingSavedTrajectory,
  onPlanToggle,
  onRun,
  onClear,
  onRefreshTrajectories,
  onSelectTrajectory,
  onLoadTrajectory,
  onUndoPoint,
}: TrajectoryPanelProps) {
  const waypointList = preview?.waypoints ?? plannerPoints;
  const waypointCount = waypointList.length;
  const hasSavedTrajectories = savedTrajectories.length > 0;
  const lastPoint =
    waypointList.length > 0 ? waypointList[waypointList.length - 1] : null;

  return (
    <div className="pointer-events-auto w-full max-w-xs rounded-xl border border-slate-700/60 bg-slate-900/80 p-4 shadow-lg shadow-slate-900/50 backdrop-blur-lg">
      <div className="mb-3 flex items-center justify-between">
        <span className="text-xs font-semibold uppercase tracking-[0.25em] text-cyan-200/80">
          Trajectory
        </span>
        <div className="flex items-center gap-2">
          <button
            type="button"
            onClick={onPlanToggle}
            disabled={isPlanLoading || isRunning}
            className={`rounded-full border border-slate-600/50 p-2 transition ${
              isPlanning
                ? "bg-cyan-500/80 text-slate-950 shadow-inner shadow-cyan-400/40"
                : "bg-slate-900/60 text-slate-200 hover:border-slate-400 hover:text-slate-100"
            } ${isPlanLoading || isRunning ? "opacity-60" : ""}`}
            aria-label="Select trajectory target"
          >
            <Route size={18} strokeWidth={2} />
          </button>
          <button
            type="button"
            onClick={onUndoPoint}
            disabled={plannerPoints.length === 0 || isPlanLoading || isRunning}
            className={`rounded-full border border-slate-600/50 bg-slate-900/60 p-2 text-slate-200 transition hover:border-slate-400 hover:text-slate-100 ${
              plannerPoints.length === 0 || isPlanLoading || isRunning
                ? "opacity-60"
                : ""
            }`}
            aria-label="Remove last waypoint"
          >
            <Undo2 size={18} strokeWidth={2} />
          </button>
          <button
            type="button"
            onClick={onRun}
            disabled={!preview || isPlanLoading || isRunning}
            className={`rounded-full border border-slate-600/50 bg-slate-900/60 p-2 text-slate-200 transition hover:border-slate-400 hover:text-slate-100 ${
              (!preview || isPlanLoading || isRunning) ? "opacity-60" : ""
            }`}
            aria-label="Execute planned trajectory"
          >
            <Play size={18} strokeWidth={2} />
          </button>
          <button
            type="button"
            onClick={onClear}
            disabled={(!preview && !isPlanning) || isPlanLoading}
            className={`rounded-full border border-slate-600/50 bg-slate-900/60 p-2 text-slate-200 transition hover:border-slate-400 hover:text-slate-100 ${
              ((!preview && !isPlanning) || isPlanLoading) ? "opacity-60" : ""
            }`}
            aria-label="Clear planned trajectory"
          >
            <Trash2 size={18} strokeWidth={2} />
          </button>
        </div>
      </div>
      <div className="text-sm text-slate-100/90">
        {isPlanLoading ? (
          <p>Planning preview trajectory…</p>
        ) : isRunning ? (
          <p>Executing trajectory…</p>
        ) : isPlanning ? (
          <p>
            Shift-click in the workspace to add waypoints. Use undo to remove
            the last point.
          </p>
        ) : preview ? (
          <div className="flex flex-col gap-2">
            <div className="text-xs text-slate-300/80">
              Loaded:{" "}
              <span className="font-semibold text-slate-100">{preview.name}</span>
            </div>
            <div className="text-xs text-slate-300/80">
              Waypoints:{" "}
              <span className="font-semibold text-slate-100">{waypointCount}</span>
            </div>
            {lastPoint && (
              <div className="text-xs text-slate-300/80">
                Last point (m):{" "}
                <span className="font-semibold text-slate-100">
                  {lastPoint.x.toFixed(3)}, {lastPoint.y.toFixed(3)},{" "}
                  {lastPoint.z.toFixed(3)}
                </span>
              </div>
            )}
          </div>
        ) : (
          <p>No preview loaded yet.</p>
        )}
      </div>
      <div className="mt-4 border-t border-slate-700/50 pt-3">
        <div className="mb-2 flex items-center justify-between">
          <span className="text-xs font-semibold uppercase tracking-[0.25em] text-cyan-200/80">
            Saved Trajectories
          </span>
          <button
            type="button"
            onClick={onRefreshTrajectories}
            disabled={isTrajectoryListLoading}
            className={`rounded-full border border-slate-600/50 bg-slate-900/60 p-2 text-slate-200 transition hover:border-slate-400 hover:text-slate-100 ${
              isTrajectoryListLoading ? "cursor-wait opacity-60" : ""
            }`}
            aria-label="Refresh saved trajectories"
          >
            <RefreshCcw
              size={16}
              strokeWidth={2}
              className={isTrajectoryListLoading ? "animate-spin" : ""}
            />
          </button>
        </div>
        {isTrajectoryListLoading ? (
          <p className="text-xs text-slate-300/80">Loading trajectories…</p>
        ) : hasSavedTrajectories ? (
          <div className="flex items-center gap-2">
            <select
              className="flex-1 rounded-lg border border-slate-600/70 bg-slate-950/60 px-3 py-2 text-sm text-slate-100 focus:border-cyan-400/60 focus:outline-none focus:ring-2 focus:ring-cyan-500/30"
              value={selectedTrajectory}
              onChange={(event) => onSelectTrajectory(event.target.value)}
            >
              {savedTrajectories.map((name) => (
                <option key={name} value={name}>
                  {name}
                </option>
              ))}
            </select>
            <button
              type="button"
              onClick={onLoadTrajectory}
              disabled={!selectedTrajectory || isLoadingSavedTrajectory}
              className={`rounded-lg border border-slate-600/60 bg-slate-900/60 px-3 py-2 text-sm font-semibold text-slate-100 transition hover:border-slate-400 hover:text-slate-50 ${
                (!selectedTrajectory || isLoadingSavedTrajectory)
                  ? "cursor-not-allowed opacity-60"
                  : ""
              }`}
              aria-label="Load selected trajectory"
            >
              {isLoadingSavedTrajectory ? "Loading…" : "Load"}
            </button>
          </div>
        ) : (
          <p className="text-xs text-slate-300/80">
            No saved trajectories available.
          </p>
        )}
      </div>
    </div>
  );
}

type SettingsDialogProps = {
  isOpen: boolean;
  apiHost: string;
  visionHost: string;
  showBoundingBox: boolean;
  onHostChange: (value: string) => void;
  onVisionHostChange: (value: string) => void;
  onShowBoundingBoxChange: (value: boolean) => void;
  onClose: () => void;
};

function SettingsDialog({
  isOpen,
  apiHost,
  visionHost,
  showBoundingBox,
  onHostChange,
  onVisionHostChange,
  onShowBoundingBoxChange,
  onClose,
}: SettingsDialogProps) {
  useEffect(() => {
    if (!isOpen) {
      return;
    }
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        onClose();
      }
    };
    window.addEventListener("keydown", handleKeyDown);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
    };
  }, [isOpen, onClose]);

  if (!isOpen) {
    return null;
  }

  return (
    <div
      role="dialog"
      aria-modal="true"
      className="fixed inset-0 z-50 flex items-center justify-center bg-slate-950/80 p-4 backdrop-blur"
      onClick={(event) => {
        if (event.target === event.currentTarget) {
          onClose();
        }
      }}
    >
      <div className="w-full max-w-md rounded-2xl border border-slate-700/60 bg-slate-900/90 p-6 shadow-2xl shadow-slate-950/60">
        <div className="mb-4 flex items-center justify-between">
          <h2 className="text-lg font-semibold text-cyan-200">Settings</h2>
          <button
            type="button"
            onClick={onClose}
            className="rounded-full border border-slate-600/60 p-1 text-slate-300 transition hover:border-slate-400 hover:text-slate-100"
            aria-label="Close settings"
          >
            <X size={16} strokeWidth={2} />
          </button>
        </div>
        <div className="flex flex-col gap-2">
          <label className="text-sm font-medium text-slate-200/90">
            Gradient API Host
            <input
              className="mt-1 w-full rounded-lg border border-slate-600/70 bg-slate-950/60 px-4 py-2 text-base text-slate-100 placeholder:text-slate-400 focus:border-cyan-400/60 focus:outline-none focus:ring-2 focus:ring-cyan-500/30"
              type="text"
              value={apiHost}
              onChange={(event) => onHostChange(event.target.value)}
              placeholder="http://localhost:8000"
              autoComplete="off"
            />
          </label>
          <p className="text-xs text-slate-400/90">
            Provide the base URL for the telemetry API. Changes apply
            immediately and persist for the next connection attempt.
          </p>
          <label className="mt-4 text-sm font-medium text-slate-200/90">
            Gradient Vision Host
            <input
              className="mt-1 w-full rounded-lg border border-slate-600/70 bg-slate-950/60 px-4 py-2 text-base text-slate-100 placeholder:text-slate-400 focus:border-cyan-400/60 focus:outline-none focus:ring-2 focus:ring-cyan-500/30"
              type="text"
              value={visionHost}
              onChange={(event) => onVisionHostChange(event.target.value)}
              placeholder="http://localhost:8080"
              autoComplete="off"
            />
          </label>
          <p className="text-xs text-slate-400/90">
            MJPEG endpoint for the camera overlay. Leave blank to default to
            the current origin on port 8080.
          </p>
          <label className="mt-4 flex items-center gap-3 text-sm font-medium text-slate-200/90">
            <input
              type="checkbox"
              className="h-4 w-4 rounded border border-slate-600 bg-slate-900 text-cyan-400 focus:outline-none focus:ring-2 focus:ring-cyan-500/40"
              checked={showBoundingBox}
              onChange={(event) => onShowBoundingBoxChange(event.target.checked)}
            />
            Show arm bounding box
          </label>
        </div>
      </div>
    </div>
  );
}

export default function App() {
  const [apiHost, setApiHost] = useState(() => resolveDefaultApiHost());
  const [visionHost, setVisionHost] = useState(() => resolveDefaultVisionHost());
  const [isConnected, setIsConnected] = useState(false);
  const [latest, setLatest] = useState<TelemetryEvent | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [visionError, setVisionError] = useState<string | null>(null);
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const [hasAttemptedAutoConnect, setHasAttemptedAutoConnect] = useState(false);
  const [isVisionActive, setIsVisionActive] = useState(false);
  const [showBoundingBox, setShowBoundingBox] = useState(true);
  const [isStopping, setIsStopping] = useState(false);
  const [previewPlan, setPreviewPlan] = useState<PreviewPlan | null>(null);
  const [plannerPoints, setPlannerPoints] = useState<Point3[]>([]);
  const [isPlanning, setIsPlanning] = useState(false);
  const [isPlanLoading, setIsPlanLoading] = useState(false);
  const [isRunningPreview, setIsRunningPreview] = useState(false);
  const [savedTrajectories, setSavedTrajectories] = useState<string[]>([]);
  const [isTrajectoryListLoading, setIsTrajectoryListLoading] = useState(false);
  const [selectedTrajectory, setSelectedTrajectory] = useState("");
  const [isLoadingSavedTrajectory, setIsLoadingSavedTrajectory] = useState(false);
  const [isHoming, setIsHoming] = useState(false);
  const visualizerRef = useRef<ArmVisualizerHandle | null>(null);
  const normalizedApiHost = useMemo(() => normaliseApiHost(apiHost), [apiHost]);
  const normalisedVisionHost = useMemo(
    () => normaliseVisionHost(visionHost),
    [visionHost],
  );
  const visionStreamUrl = useMemo(
    () => `${normalisedVisionHost}/stream.mjpg`,
    [normalisedVisionHost],
  );
  const visualWaypoints =
    plannerPoints.length > 0
      ? plannerPoints
      : previewPlan?.waypoints ?? [];
  const visualPathPoints =
    previewPlan?.pathPoints && previewPlan.pathPoints.length > 0
      ? previewPlan.pathPoints
      : visualWaypoints;
  const toggleButtonClasses = useMemo(
    () =>
      isConnected
        ? "rounded-full bg-gradient-to-r from-rose-500 to-rose-600 p-2 text-white shadow-md shadow-rose-500/30 transition hover:brightness-110"
        : "rounded-full bg-gradient-to-r from-cyan-400 to-blue-500 p-2 text-slate-950 shadow-md shadow-blue-500/30 transition hover:brightness-110",
    [isConnected],
  );
  const cameraButtonClasses = useMemo(
    () =>
      isVisionActive
        ? "rounded-full bg-gradient-to-r from-rose-500 to-rose-600 p-2 text-white shadow-md shadow-rose-500/30 transition hover:brightness-110"
        : "rounded-full bg-gradient-to-r from-cyan-400 to-blue-500 p-2 text-slate-950 shadow-md shadow-blue-500/30 transition hover:brightness-110",
    [isVisionActive],
  );
  const eventSourceRef = useRef<EventSource | null>(null);
  const trajectoryRefreshInFlight = useRef(false);

  const disconnect = useCallback(() => {
    eventSourceRef.current?.close();
    eventSourceRef.current = null;
    setIsConnected(false);
    setLatest(null);
    setIsVisionActive(false);
    setVisionError(null);
    setPreviewPlan(null);
    setPlannerPoints([]);
    setIsPlanning(false);
    setIsPlanLoading(false);
    setIsRunningPreview(false);
    setSavedTrajectories([]);
    setIsTrajectoryListLoading(false);
    setSelectedTrajectory("");
    setIsLoadingSavedTrajectory(false);
    setIsHoming(false);
  }, []);

  const handleMessage = useCallback((payload: string) => {
    let joints: number[] | undefined;
    let gripper: number | undefined;

    try {
      const parsed = JSON.parse(payload);
      if (Array.isArray(parsed?.joints)) {
        joints = parsed.joints
          .map((value: unknown) =>
            typeof value === "number" ? value : Number(value),
          )
          .filter((value) => Number.isFinite(value));
      }
      if (typeof parsed?.gripper === "number") {
        gripper = parsed.gripper;
      }
    } catch {
      // fall back to raw payload only
    }

    const next: TelemetryEvent = {
      timestamp: Date.now(),
      raw: payload,
      joints,
      gripper,
    };
    setLatest(next);
  }, []);

  const connect = useCallback(() => {
    if (isConnected) {
      return;
    }
    const host = normalizedApiHost;
    const url = `${host}/monitor`;
    setError(null);

    try {
      const es = new EventSource(url);
      eventSourceRef.current = es;

      es.onopen = () => {
        setIsConnected(true);
        setLatest(null);
      };

      es.onerror = () => {
        setError(
          "Connection lost. Ensure the API is reachable and CORS allows this origin.",
        );
        disconnect();
      };

      es.onmessage = (evt) => {
        handleMessage(evt.data);
      };
    } catch (err) {
      setError((err as Error).message);
      disconnect();
    }
  }, [disconnect, handleMessage, isConnected, normalizedApiHost]);

  const toggleConnection = useCallback(() => {
    if (isConnected) {
      disconnect();
      return;
    }
    connect();
  }, [connect, disconnect, isConnected]);

  const toggleVision = useCallback(() => {
    if (isVisionActive) {
      setIsVisionActive(false);
      setVisionError(null);
      return;
    }
    setVisionError(null);
    setIsVisionActive(true);
  }, [isVisionActive]);

  const handleResetView = useCallback(() => {
    visualizerRef.current?.resetView();
  }, []);

  const requestPlannerPreview = useCallback(
    async (points: Point3[]) => {
      if (points.length === 0) {
        setPreviewPlan(null);
        setPlannerPoints([]);
        return;
      }
      setIsPlanLoading(true);
      setError(null);
      try {
        const response = await fetch(`${normalizedApiHost}/trajectory/plan-points`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ points: encodePointsForApi(points) }),
        });
        if (!response.ok) {
          const message = await response.text();
          throw new Error(message || `Plan request failed (${response.status})`);
        }
        const data = await response.json();
        const { plan, waypoints } = previewFromPlannerPayload(data);
        setPreviewPlan(plan);
        setPlannerPoints(waypoints);
      } catch (err) {
        setError(`Failed to plan trajectory: ${(err as Error).message}`);
      } finally {
        setIsPlanLoading(false);
      }
    },
    [normalizedApiHost],
  );

  const handlePlanToggle = useCallback(() => {
    if (isPlanLoading || isRunningPreview) {
      return;
    }
    setError(null);
    setIsPlanning((current) => {
      const next = !current;
      if (next) {
        setPlannerPoints((existing) =>
          existing.length > 0
            ? existing
            : previewPlan?.waypoints ?? [],
        );
      }
      return next;
    });
  }, [isPlanLoading, isRunningPreview, previewPlan]);

  const handlePointSelected = useCallback(
    async (point: Point3) => {
      if (!isPlanning || isPlanLoading) {
        return;
      }
      const nextPoints = [...plannerPoints, point];
      setPlannerPoints(nextPoints);
      await requestPlannerPreview(nextPoints);
    },
    [isPlanning, isPlanLoading, plannerPoints, requestPlannerPreview],
  );

  const handleUndoPoint = useCallback(async () => {
    if (plannerPoints.length === 0 || isPlanLoading) {
      return;
    }
    const nextPoints = plannerPoints.slice(0, -1);
    setPlannerPoints(nextPoints);
    if (nextPoints.length === 0) {
      setPreviewPlan(null);
      return;
    }
    await requestPlannerPreview(nextPoints);
  }, [plannerPoints, isPlanLoading, requestPlannerPreview]);

  const handleRunPreview = useCallback(async () => {
    if (!previewPlan || isPlanLoading || isRunningPreview) {
      return;
    }
    setIsRunningPreview(true);
    setError(null);
    try {
      const response = await fetch(`${normalizedApiHost}/trajectory/run`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name: previewPlan.name, use_cache: true }),
      });
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || `Run request failed (${response.status})`);
      }
    } catch (err) {
      setError(`Failed to execute trajectory: ${(err as Error).message}`);
    } finally {
      setIsRunningPreview(false);
    }
  }, [previewPlan, isPlanLoading, isRunningPreview, normalizedApiHost]);

  const handleClearPreview = useCallback(() => {
    setError(null);
    setPreviewPlan(null);
    setPlannerPoints([]);
    setIsPlanning(false);
  }, []);

  const refreshTrajectoryList = useCallback(async () => {
    if (trajectoryRefreshInFlight.current) {
      return;
    }
    trajectoryRefreshInFlight.current = true;
    setError(null);
    setIsTrajectoryListLoading(true);
    try {
      const response = await fetch(`${normalizedApiHost}/trajectory/list`);
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || `List request failed (${response.status})`);
      }
      const data = (await response.json()) as { trajectories?: unknown };
      const names = Array.isArray(data.trajectories)
        ? data.trajectories
            .map((value) =>
              typeof value === "string" ? value.trim() : "",
            )
            .filter((value): value is string => value.length > 0)
        : [];
      setSavedTrajectories(names);
      setSelectedTrajectory((current) =>
        current && names.includes(current) ? current : names[0] ?? "",
      );
    } catch (err) {
      setError(`Failed to load trajectories: ${(err as Error).message}`);
      setSavedTrajectories([]);
      setSelectedTrajectory("");
    } finally {
      setIsTrajectoryListLoading(false);
      trajectoryRefreshInFlight.current = false;
    }
  }, [normalizedApiHost]);

  const handleSelectTrajectory = useCallback((value: string) => {
    setSelectedTrajectory(value);
  }, []);

  const handleLoadTrajectory = useCallback(async () => {
    if (!selectedTrajectory || isLoadingSavedTrajectory) {
      return;
    }
    setError(null);
    setIsLoadingSavedTrajectory(true);
    try {
      const response = await fetch(
        `${normalizedApiHost}/trajectory/detail/${encodeURIComponent(selectedTrajectory)}`,
      );
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || `Load request failed (${response.status})`);
      }
      const data = await response.json();
      const plan = previewFromTrajectoryDetail(
        typeof data?.name === "string" ? data.name : selectedTrajectory,
        data?.trajectory ?? { moves: [] },
      );
      setPreviewPlan(plan);
      setPlannerPoints(plan.waypoints);
      setIsPlanning(false);
    } catch (err) {
      setError(`Failed to load trajectory: ${(err as Error).message}`);
    } finally {
      setIsLoadingSavedTrajectory(false);
    }
  }, [selectedTrajectory, isLoadingSavedTrajectory, normalizedApiHost]);

  const issueStop = useCallback(async () => {
    if (isStopping) {
      return;
    }
    const stopEndpoint = `${normalizedApiHost}/control/stop`;
    setIsStopping(true);
    try {
      const response = await fetch(stopEndpoint, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
      });
      if (!response.ok) {
        let detail = `${response.status} ${response.statusText}`;
        try {
          const parsed = await response.json();
          if (typeof parsed?.detail === "string") {
            detail = parsed.detail;
          }
        } catch {
          // ignore parse error, keep default detail
        }
        throw new Error(detail);
      }
    } catch (err) {
      setError(
        `Failed to send STOP command: ${(err as Error).message ?? "Unknown error"}`,
      );
    } finally {
      setIsStopping(false);
    }
  }, [isStopping, normalizedApiHost]);

  const handleHome = useCallback(async () => {
    if (isHoming) {
      return;
    }
    setError(null);
    setIsHoming(true);
    try {
      const response = await fetch(`${normalizedApiHost}/control/home`, {
        method: "POST",
      });
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || `Home request failed (${response.status})`);
      }
    } catch (err) {
      setError(`Failed to home: ${(err as Error).message ?? "Unknown error"}`);
    } finally {
      setIsHoming(false);
    }
  }, [isHoming, normalizedApiHost]);

  useEffect(() => {
    if (!hasAttemptedAutoConnect) {
      setHasAttemptedAutoConnect(true);
      connect();
    }
  }, [connect, hasAttemptedAutoConnect]);

  useEffect(() => {
    if (!isConnected) {
      return;
    }
    refreshTrajectoryList();
  }, [isConnected, refreshTrajectoryList]);

  useEffect(() => {
    return () => {
      disconnect();
    };
  }, [disconnect]);

  useEffect(() => {
    if (isConnected) {
      setIsVisionActive(true);
    } else {
      setIsVisionActive(false);
    }
  }, [isConnected]);

  useEffect(() => {
    setVisionError(null);
  }, [normalisedVisionHost]);

  const streamingLabel = useMemo(
    () => (isConnected ? "Streaming" : "Disconnected"),
    [isConnected],
  );

  return (
    <div className="flex min-h-screen flex-col bg-gradient-to-b from-slate-900/80 via-slate-950 to-black text-slate-100">
      <header className="relative flex flex-col gap-4 border-b border-slate-800/40 bg-slate-950/60 px-6 py-6 shadow-inner shadow-slate-900/40 backdrop-blur">
        <div className="flex flex-col gap-4 sm:flex-row sm:items-start sm:justify-between">
          <div className="flex flex-col">
            <div className="flex justify-end text-2xl font-semibold tracking-tight text-cyan-300 sm:text-3xl w-full">
              Control Center
            </div>
            <div className="flex justify-end text-md tracking-tight text-cyan-300 sm:text-md w-full">
              Gradient Robotics
            </div>
          </div>
          <div className="flex items-center gap-3">
            <span
              className={`inline-flex items-center gap-2 rounded-full px-3 py-1 text-xs font-medium ${
                isConnected
                  ? "bg-emerald-500/15 text-emerald-200 ring-1 ring-inset ring-emerald-400/30"
                  : "bg-rose-500/10 text-rose-200 ring-1 ring-inset ring-rose-400/20"
              }`}
            >
              <span
                className={`h-2 w-2 rounded-full ${
                  isConnected ? "bg-emerald-400" : "bg-rose-400"
                }`}
              />
              {streamingLabel}
            </span>
            <button
              type="button"
              onClick={toggleConnection}
              className={toggleButtonClasses}
              aria-label={isConnected ? "Disconnect from robot" : "Connect to robot"}
            >
              {isConnected ? (
                <Unplug size={18} strokeWidth={2} />
              ) : (
                <Plug size={18} strokeWidth={2} />
              )}
            </button>
            <button
              type="button"
              onClick={handleHome}
              disabled={!isConnected || isHoming}
              className={`rounded-full border border-slate-600/60 bg-slate-900/60 p-2 text-slate-300 transition hover:border-slate-400 hover:text-slate-100 ${
                (!isConnected || isHoming) ? "opacity-60 cursor-not-allowed" : ""
              }`}
              aria-label="Move arm to home position"
            >
              <Home size={18} strokeWidth={2} />
            </button>
            <button
              type="button"
              onClick={issueStop}
              disabled={isStopping}
              className={`rounded-full bg-gradient-to-r from-rose-600 to-rose-700 p-2 text-white shadow-md shadow-rose-500/40 transition ${
                isStopping ? "cursor-wait opacity-60" : "hover:brightness-110"
              }`}
              aria-label="Send emergency stop"
            >
              <Octagon size={18} strokeWidth={2.5} />
            </button>
            <button
              type="button"
              onClick={toggleVision}
              className={cameraButtonClasses}
              aria-label={isVisionActive ? "Disable camera overlay" : "Enable camera overlay"}
            >
              {isVisionActive ? (
                <Camera size={18} strokeWidth={2} />
              ) : (
                <CameraOff size={18} strokeWidth={2} />
              )}
            </button>
            <button
              type="button"
              onClick={handleResetView}
              className="rounded-full border border-slate-600/60 bg-slate-900/60 p-2 text-slate-300 transition hover:border-slate-400 hover:text-slate-100"
              aria-label="Reset arm view"
            >
              <RefreshCcw size={18} strokeWidth={2} />
            </button>
            <button
              type="button"
              onClick={() => setIsSettingsOpen(true)}
              className="rounded-full border border-slate-600/60 bg-slate-900/60 p-2 text-slate-300 transition hover:border-slate-400 hover:text-slate-100"
              aria-label="Open settings"
            >
              <Settings size={18} strokeWidth={2} />
            </button>
          </div>
        </div>
        {error && (
          <div className="rounded-lg border border-rose-500/40 bg-rose-500/10 px-4 py-3 text-sm text-rose-200">
            {error}
          </div>
        )}
        {visionError && (
          <div className="rounded-lg border border-amber-500/40 bg-amber-500/10 px-4 py-3 text-sm text-amber-200">
            {visionError}
          </div>
        )}
      </header>
      <main className="relative flex-1 overflow-hidden">
        <ArmVisualizer
          ref={visualizerRef}
          joints={latest?.joints}
          showBoundingBox={showBoundingBox}
          selectionMode={isPlanning && !isPlanLoading}
          onPointSelected={handlePointSelected}
          pathPoints={visualPathPoints}
          waypoints={visualWaypoints}
        />
        {isVisionActive && !visionError && (
          <div className="pointer-events-auto absolute left-6 top-6 z-10 flex max-w-sm flex-col gap-2">
            <div className="overflow-hidden rounded-xl border border-slate-700/60 bg-slate-950/80 shadow-lg shadow-slate-950/40 backdrop-blur">
              <img
                src={visionStreamUrl}
                alt="Gradient Vision stream"
                className="block max-h-64 w-full object-cover"
                onLoad={() => setVisionError(null)}
                onError={() => {
                  setVisionError(
                    "Unable to load the vision stream. Ensure Gradient Vision is running and accessible.",
                  );
                  setIsVisionActive(false);
                }}
              />
            </div>
          </div>
        )}
        <div className="pointer-events-none absolute right-6 top-6 z-10 flex flex-col gap-3">
          <TelemetryPanel latest={latest} />
          <TrajectoryPanel
            isPlanning={isPlanning}
            isPlanLoading={isPlanLoading}
            isRunning={isRunningPreview}
            preview={previewPlan}
            plannerPoints={plannerPoints}
            savedTrajectories={savedTrajectories}
            selectedTrajectory={selectedTrajectory}
            isTrajectoryListLoading={isTrajectoryListLoading}
            isLoadingSavedTrajectory={isLoadingSavedTrajectory}
            onPlanToggle={handlePlanToggle}
            onRun={handleRunPreview}
            onClear={handleClearPreview}
            onRefreshTrajectories={refreshTrajectoryList}
            onSelectTrajectory={handleSelectTrajectory}
            onLoadTrajectory={handleLoadTrajectory}
            onUndoPoint={handleUndoPoint}
          />
        </div>
      </main>
      <SettingsDialog
        isOpen={isSettingsOpen}
        apiHost={apiHost}
        visionHost={visionHost}
        showBoundingBox={showBoundingBox}
        onHostChange={setApiHost}
        onVisionHostChange={setVisionHost}
        onShowBoundingBoxChange={setShowBoundingBox}
        onClose={() => setIsSettingsOpen(false)}
      />
    </div>
  );
}
