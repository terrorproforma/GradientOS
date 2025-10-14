import { FormEvent, useCallback, useEffect, useMemo, useRef, useState } from "react";
import { resolveDefaultApiHost } from "./useEndpoint";

type TelemetryEvent = {
  id: number;
  timestamp: number;
  raw: string;
  joints?: number[];
  gripper?: number;
};

const HISTORY_LIMIT = 200;

function normaliseHost(input: string): string {
  const trimmed = input.trim();
  if (!trimmed) {
    return resolveDefaultApiHost();
  }
  return trimmed.replace(/\/+$/, "");
}

export default function App() {
  const [apiHost, setApiHost] = useState(() => resolveDefaultApiHost());
  const [isConnected, setIsConnected] = useState(false);
  const [events, setEvents] = useState<TelemetryEvent[]>([]);
  const [error, setError] = useState<string | null>(null);
  const eventSourceRef = useRef<EventSource | null>(null);
  const eventIdRef = useRef(0);

  const disconnect = useCallback(() => {
    eventSourceRef.current?.close();
    eventSourceRef.current = null;
    setIsConnected(false);
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

    eventIdRef.current += 1;
    const next: TelemetryEvent = {
      id: eventIdRef.current,
      timestamp: Date.now(),
      raw: payload,
      joints,
      gripper,
    };
    setEvents((previous) => [next, ...previous].slice(0, HISTORY_LIMIT));
  }, []);

  const connect = useCallback(() => {
    if (isConnected) {
      return;
    }
    const host = normaliseHost(apiHost);
    const url = `${host}/monitor`;
    setError(null);

    try {
      const es = new EventSource(url);
      eventSourceRef.current = es;

      es.onopen = () => {
        setIsConnected(true);
        setEvents([]);
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
  }, [apiHost, disconnect, handleMessage, isConnected]);

  useEffect(() => {
    return () => {
      disconnect();
    };
  }, [disconnect]);

  const latest = useMemo(() => events[0], [events]);

  const handleSubmit = (event: FormEvent) => {
    event.preventDefault();
    connect();
  };

  return (
    <div className="min-h-screen bg-gradient-to-b from-slate-900/80 via-slate-950 to-black px-4 py-8 text-slate-100">
      <div className="mx-auto flex w-full max-w-6xl flex-col gap-6">
        <header className="flex flex-col gap-4 rounded-xl border border-slate-700/40 bg-slate-900/70 p-6 shadow-lg shadow-slate-900/40 backdrop-blur">
          <div className="flex flex-col gap-2 sm:flex-row sm:items-center sm:justify-between">
            <div>
              <h1 className="text-2xl font-semibold tracking-tight text-cyan-300 sm:text-3xl">
                GradientOS Monitor
              </h1>
              <p className="text-sm text-slate-300/80">
                Subscribe to the controller telemetry stream via Server-Sent Events.
              </p>
            </div>
            <span
              className={`inline-flex items-center gap-2 self-start rounded-full px-3 py-1 text-xs font-medium ${
                isConnected
                  ? "bg-emerald-500/20 text-emerald-200 ring-1 ring-inset ring-emerald-400/30"
                  : "bg-rose-500/10 text-rose-200 ring-1 ring-inset ring-rose-400/20"
              }`}
            >
              <span
                className={`h-2 w-2 rounded-full ${
                  isConnected ? "bg-emerald-400" : "bg-rose-400"
                }`}
              />
              {isConnected ? "Streaming" : "Disconnected"}
            </span>
          </div>

          <form
            className="flex flex-col gap-3 sm:flex-row sm:items-center"
            onSubmit={handleSubmit}
          >
            <label className="flex flex-1 flex-col gap-1 text-sm font-medium text-slate-200/90">
              Gradient API Host
              <input
                className="w-full rounded-lg border border-slate-600/70 bg-slate-950/60 px-4 py-2 text-base text-slate-100 placeholder:text-slate-400 focus:border-cyan-400/60 focus:outline-none focus:ring-2 focus:ring-cyan-500/30"
                type="text"
                value={apiHost}
                onChange={(event) => setApiHost(event.target.value)}
                placeholder="http://localhost:8000"
                autoComplete="off"
              />
            </label>
            <div className="flex gap-2">
              <button
                type="submit"
                disabled={isConnected}
                className="rounded-lg bg-gradient-to-r from-cyan-400 to-blue-500 px-5 py-2 font-semibold text-slate-950 shadow-md shadow-blue-500/30 transition hover:brightness-110 disabled:cursor-not-allowed disabled:bg-slate-700/70 disabled:text-slate-300 disabled:shadow-none"
              >
                Connect
              </button>
              <button
                type="button"
                onClick={disconnect}
                disabled={!isConnected}
                className="rounded-lg border border-slate-600/60 px-5 py-2 font-semibold text-slate-200 transition hover:border-slate-400 disabled:cursor-not-allowed disabled:border-slate-700 disabled:text-slate-500"
              >
                Disconnect
              </button>
            </div>
          </form>

          {error && (
            <div className="rounded-lg border border-rose-500/40 bg-rose-500/10 px-4 py-3 text-sm text-rose-200">
              {error}
            </div>
          )}
        </header>

        <main className="grid gap-6 lg:grid-cols-[minmax(0,1fr)_minmax(0,1.2fr)]">
          <section className="flex flex-col gap-4 rounded-xl border border-slate-700/40 bg-slate-900/70 p-6 shadow-lg shadow-slate-900/40 backdrop-blur">
            <div>
              <h2 className="text-xs font-semibold uppercase tracking-[0.3em] text-cyan-300/80">
                Latest Telemetry
              </h2>
            </div>
            {latest ? (
              <div className="flex flex-col gap-4 rounded-lg border border-slate-700/50 bg-slate-950/70 p-5 shadow-inner shadow-slate-900/40">
                <div className="flex flex-wrap items-center gap-2 text-sm text-slate-300">
                  <span className="font-medium text-slate-200">Received</span>
                  <span>
                    {new Date(latest.timestamp).toLocaleTimeString()}
                  </span>
                </div>
                {latest.joints && latest.joints.length > 0 && (
                  <div>
                    <h3 className="text-sm font-semibold text-cyan-200">
                      Joint Angles (rad)
                    </h3>
                    <ul className="mt-2 grid grid-cols-2 gap-x-6 gap-y-1 text-sm text-slate-200/90 sm:grid-cols-3">
                      {latest.joints.map((value, index) => (
                        <li key={index}>
                          <span className="text-slate-400">J{index + 1}:</span>{" "}
                          {value.toFixed(3)}
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
                {typeof latest.gripper === "number" && (
                  <div className="text-sm text-slate-200/90">
                    <span className="font-semibold text-cyan-200">
                      Gripper
                    </span>{" "}
                    {latest.gripper.toFixed(3)}
                  </div>
                )}
                <details className="rounded-lg bg-slate-900/60 p-3 text-xs text-slate-300/80">
                  <summary className="cursor-pointer text-slate-200 underline-offset-4 hover:text-cyan-200">
                    Raw payload
                  </summary>
                  <pre className="mt-2 overflow-x-auto whitespace-pre-wrap break-words font-mono text-[0.7rem] leading-relaxed text-slate-300">
                    {latest.raw}
                  </pre>
                </details>
              </div>
            ) : (
              <p className="rounded-lg border border-dashed border-slate-700/60 bg-slate-950/40 p-6 text-sm text-slate-400">
                No telemetry received yet. Connect to begin streaming.
              </p>
            )}
          </section>

          <section className="flex min-h-[320px] flex-col gap-4 rounded-xl border border-slate-700/40 bg-slate-900/70 p-6 shadow-lg shadow-slate-900/40 backdrop-blur">
            <div className="flex items-center justify-between">
              <h2 className="text-xs font-semibold uppercase tracking-[0.3em] text-cyan-300/80">
                Event Log
              </h2>
              <span className="text-xs text-slate-400">
                {events.length} events
              </span>
            </div>
            <div className="flex-1 overflow-y-auto rounded-lg border border-slate-800/60 bg-slate-950/60 p-4">
              {events.length === 0 ? (
                <p className="text-sm text-slate-400">
                  Waiting for events…
                </p>
              ) : (
                <ul className="flex flex-col gap-3">
                  {events.map((event) => (
                    <li
                      key={event.id}
                      className="rounded-lg border border-slate-800/60 bg-slate-900/70 p-3"
                    >
                      <header className="flex items-center justify-between text-xs text-slate-400">
                        <span>{new Date(event.timestamp).toLocaleTimeString()}</span>
                        {event.joints && (
                          <span className="rounded-full bg-slate-800/70 px-2 py-0.5 text-[0.65rem] uppercase tracking-wide text-slate-200">
                            Joints
                          </span>
                        )}
                      </header>
                      <pre className="mt-2 overflow-x-auto whitespace-pre-wrap break-words font-mono text-[0.7rem] leading-relaxed text-slate-200/90">
                        {event.raw}
                      </pre>
                    </li>
                  ))}
                </ul>
              )}
            </div>
          </section>
        </main>
      </div>
    </div>
  );
}
