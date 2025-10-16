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
  X,
} from "lucide-react";
import { resolveDefaultApiHost, resolveDefaultVisionHost } from "./useEndpoint";
import { ArmVisualizer, type ArmVisualizerHandle } from "./ArmVisualizer";

type TelemetryEvent = {
  timestamp: number;
  raw: string;
  joints?: number[];
  gripper?: number;
};

type TrajectoryPreview = {
  target: { x: number; y: number; z: number };
  velocity: number;
  acceleration: number;
  closed_loop: boolean;
  joints_rad: number[][];
  cartesian_m?: number[][];
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
  preview: TrajectoryPreview | null;
  onPlan: () => void;
  onRun: () => void;
  onClear: () => void;
};

function TrajectoryPanel({
  isPlanning,
  isPlanLoading,
  isRunning,
  preview,
  onPlan,
  onRun,
  onClear,
}: TrajectoryPanelProps) {
  const waypoints = preview?.joints_rad?.length ?? 0;
  const target = preview?.target;

  return (
    <div className="pointer-events-auto w-full max-w-xs rounded-xl border border-slate-700/60 bg-slate-900/80 p-4 shadow-lg shadow-slate-900/50 backdrop-blur-lg">
      <div className="mb-3 flex items-center justify-between">
        <span className="text-xs font-semibold uppercase tracking-[0.25em] text-cyan-200/80">
          Trajectory
        </span>
        <div className="flex items-center gap-2">
          <button
            type="button"
            onClick={onPlan}
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
          <p>Select a point in the workspace to preview a trajectory.</p>
        ) : preview ? (
          <div className="flex flex-col gap-2">
            {target && (
              <div className="text-xs text-slate-300/80">
                Target (m):{" "}
                <span className="font-semibold text-slate-100">
                  {target.x.toFixed(3)}, {target.y.toFixed(3)}, {target.z.toFixed(3)}
                </span>
              </div>
            )}
            <div className="text-xs text-slate-300/80">
              Waypoints:{" "}
              <span className="font-semibold text-slate-100">{waypoints}</span>
            </div>
          </div>
        ) : (
          <p>No preview planned yet.</p>
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
  const [previewTrajectory, setPreviewTrajectory] = useState<TrajectoryPreview | null>(null);
  const [previewPath, setPreviewPath] = useState<Array<{ x: number; y: number; z: number }>>([]);
  const [isPlanning, setIsPlanning] = useState(false);
  const [isPlanLoading, setIsPlanLoading] = useState(false);
  const [isRunningPreview, setIsRunningPreview] = useState(false);
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

  const disconnect = useCallback(() => {
    eventSourceRef.current?.close();
    eventSourceRef.current = null;
    setIsConnected(false);
    setLatest(null);
    setIsVisionActive(false);
    setVisionError(null);
    setPreviewTrajectory(null);
    setPreviewPath([]);
    setIsPlanning(false);
    setIsPlanLoading(false);
    setIsRunningPreview(false);
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

  const handlePlanButton = useCallback(async () => {
    if (isPlanLoading || isRunningPreview) {
      return;
    }
    setError(null);
    if (isPlanning) {
      setIsPlanning(false);
      return;
    }
    setPreviewTrajectory(null);
    setPreviewPath([]);
    setIsPlanning(true);
    try {
      await fetch(`${normalizedApiHost}/trajectory/clear-preview`, {
        method: "POST",
      });
    } catch (err) {
      // Clearing preview is best-effort; surface warning but keep planning enabled.
      setError(
        `Unable to clear previous preview: ${(err as Error).message ?? "Unknown error"}`,
      );
    }
  }, [isPlanLoading, isPlanning, isRunningPreview, normalizedApiHost]);

  const handlePointSelected = useCallback(
    async (point: { x: number; y: number; z: number }) => {
      if (!isPlanning || isPlanLoading) {
        return;
      }
      setIsPlanning(false);
      setIsPlanLoading(true);
      setError(null);
    try {
      setPreviewPath([{ x: point.x, y: point.y, z: point.z }]);
      const response = await fetch(`${normalizedApiHost}/trajectory/preview`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          x: point.x,
          y: point.y,
          z: point.z,
        }),
      });
      if (!response.ok) {
        const message = await response.text();
        throw new Error(
          message || `Preview request failed (${response.status})`,
        );
      }
      const data = (await response.json()) as TrajectoryPreview;
      setPreviewTrajectory(data);
      const pathPoints =
        Array.isArray(data.cartesian_m) && data.cartesian_m.length > 0
          ? (data.cartesian_m
              .map((coords) =>
                Array.isArray(coords) && coords.length >= 3
                  ? {
                      x: Number(coords[0]),
                      y: Number(coords[1]),
                      z: Number(coords[2]),
                    }
                  : null,
              )
              .filter(Boolean) as Array<{ x: number; y: number; z: number }>)
          : [];
      setPreviewPath(
        pathPoints.length > 0
          ? pathPoints
          : [{ x: point.x, y: point.y, z: point.z }],
      );
    } catch (err) {
      setError(`Failed to plan trajectory: ${(err as Error).message}`);
      setPreviewTrajectory(null);
      setPreviewPath([]);
    } finally {
      setIsPlanLoading(false);
    }
    },
    [isPlanning, isPlanLoading, normalizedApiHost],
  );

  const handleRunPreview = useCallback(async () => {
    if (!previewTrajectory || isPlanLoading || isRunningPreview) {
      return;
    }
    setIsRunningPreview(true);
    setError(null);
    try {
      const response = await fetch(
        `${normalizedApiHost}/trajectory/execute-preview`,
        {
          method: "POST",
        },
      );
      if (!response.ok) {
        const message = await response.text();
        throw new Error(
          message || `Execute request failed (${response.status})`,
        );
      }
      setPreviewTrajectory(null);
      setPreviewPath([]);
    } catch (err) {
      setError(`Failed to execute trajectory: ${(err as Error).message}`);
    } finally {
      setIsRunningPreview(false);
    }
  }, [
    previewTrajectory,
    isPlanLoading,
    isRunningPreview,
    normalizedApiHost,
  ]);

  const handleClearPreview = useCallback(async () => {
    setError(null);
    setPreviewTrajectory(null);
    setPreviewPath([]);
    setIsPlanning(false);
    try {
      const response = await fetch(`${normalizedApiHost}/trajectory/clear-preview`, {
        method: "POST",
      });
      if (!response.ok) {
        const message = await response.text();
        throw new Error(
          message || `Clear request failed (${response.status})`,
        );
      }
    } catch (err) {
      setError(`Failed to clear preview: ${(err as Error).message}`);
    } finally {
      setIsPlanLoading(false);
    }
  }, [normalizedApiHost]);

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
          previewPath={previewPath}
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
            preview={previewTrajectory}
            onPlan={handlePlanButton}
            onRun={handleRunPreview}
            onClear={handleClearPreview}
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
