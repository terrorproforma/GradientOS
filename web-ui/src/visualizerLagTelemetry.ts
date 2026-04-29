export type VisualizerLagSource = "monitor" | "fallback" | "prop";

export type VisualizerLagCompletedSample = {
  id: number;
  source: VisualizerLagSource;
  apiSequence?: number;
  apiSequenceGap?: number;
  sourceAgeMs?: number;
  sourceToApiMs?: number;
  sourceDeltaMs?: number;
  apiToBrowserMs?: number;
  interarrivalMs?: number;
  parseToPushMs?: number;
  receiveToVisibleMs?: number;
  pushToVisibleMs?: number;
  frameIntervalMs?: number;
  frameWorkMs?: number;
  payloadBytes?: number;
  jointCount?: number;
  usedDisplayJoints?: boolean;
  completedAtMs: number;
};

export type VisualizerLagSnapshot = {
  received: number;
  pushed: number;
  applied: number;
  superseded: number;
  dropped: number;
  pending: number;
  latest?: VisualizerLagCompletedSample;
  average?: VisualizerLagCompletedSample;
  max?: VisualizerLagCompletedSample;
  diagnosis: string;
  trace?: {
    sourceHz?: number;
    browserReceiveHz?: number;
    visibleHz?: number;
    lastPublishedAtMs?: number;
  };
};

type PendingSample = {
  id: number;
  source: VisualizerLagSource;
  apiSequence?: number;
  apiSequenceGap?: number;
  sourceAgeMs?: number;
  sourceToApiMs?: number;
  sourceDeltaMs?: number;
  apiToBrowserMs?: number;
  interarrivalMs?: number;
  browserReceivePerfMs: number;
  pushPerfMs?: number;
  payloadBytes?: number;
  jointCount?: number;
  usedDisplayJoints?: boolean;
};

const HISTORY_LIMIT = 120;
const PENDING_LIMIT = 240;
const LAG_WARN_MS = 250;
const CADENCE_WARN_MS = 75;
const FRAME_WORK_WARN_MS = 24;
const TRACE_HEARTBEAT_MS = 5000;
const GLOBAL_PUBLISH_INTERVAL_MS = 500;

let nextId = 1;
let lastReceivePerfMs: number | null = null;
let lastSourceTimeSec: number | null = null;
let lastApiSequence: number | null = null;
let lastConsoleLogPerfMs = 0;
let lastTraceHeartbeatPerfMs = 0;
let lastGlobalPublishPerfMs = 0;
let counters = {
  received: 0,
  pushed: 0,
  applied: 0,
  superseded: 0,
  dropped: 0,
};
const pending = new Map<number, PendingSample>();
const completed: VisualizerLagCompletedSample[] = [];

function nowPerfMs(): number {
  return typeof performance !== "undefined" && typeof performance.now === "function"
    ? performance.now()
    : Date.now();
}

function finiteOrUndefined(value: unknown): number | undefined {
  return typeof value === "number" && Number.isFinite(value) ? value : undefined;
}

function computeAverage(samples: VisualizerLagCompletedSample[]): VisualizerLagCompletedSample | undefined {
  if (samples.length === 0) {
    return undefined;
  }
  const average = (key: keyof VisualizerLagCompletedSample) => {
    const values = samples
      .map((sample) => finiteOrUndefined(sample[key]))
      .filter((value): value is number => typeof value === "number");
    if (values.length === 0) {
      return undefined;
    }
    return values.reduce((sum, value) => sum + value, 0) / values.length;
  };
  return {
    id: samples[samples.length - 1].id,
    source: samples[samples.length - 1].source,
    sourceAgeMs: average("sourceAgeMs"),
    sourceToApiMs: average("sourceToApiMs"),
    sourceDeltaMs: average("sourceDeltaMs"),
    apiToBrowserMs: average("apiToBrowserMs"),
    apiSequenceGap: average("apiSequenceGap"),
    interarrivalMs: average("interarrivalMs"),
    parseToPushMs: average("parseToPushMs"),
    receiveToVisibleMs: average("receiveToVisibleMs"),
    pushToVisibleMs: average("pushToVisibleMs"),
    frameIntervalMs: average("frameIntervalMs"),
    frameWorkMs: average("frameWorkMs"),
    payloadBytes: average("payloadBytes"),
    jointCount: average("jointCount"),
    completedAtMs: Date.now(),
  };
}

function computeMax(samples: VisualizerLagCompletedSample[]): VisualizerLagCompletedSample | undefined {
  if (samples.length === 0) {
    return undefined;
  }
  const maxValue = (key: keyof VisualizerLagCompletedSample) => {
    const values = samples
      .map((sample) => finiteOrUndefined(sample[key]))
      .filter((value): value is number => typeof value === "number");
    return values.length > 0 ? Math.max(...values) : undefined;
  };
  return {
    id: samples[samples.length - 1].id,
    source: samples[samples.length - 1].source,
    sourceAgeMs: maxValue("sourceAgeMs"),
    sourceToApiMs: maxValue("sourceToApiMs"),
    sourceDeltaMs: maxValue("sourceDeltaMs"),
    apiToBrowserMs: maxValue("apiToBrowserMs"),
    apiSequenceGap: maxValue("apiSequenceGap"),
    interarrivalMs: maxValue("interarrivalMs"),
    parseToPushMs: maxValue("parseToPushMs"),
    receiveToVisibleMs: maxValue("receiveToVisibleMs"),
    pushToVisibleMs: maxValue("pushToVisibleMs"),
    frameIntervalMs: maxValue("frameIntervalMs"),
    frameWorkMs: maxValue("frameWorkMs"),
    payloadBytes: maxValue("payloadBytes"),
    jointCount: maxValue("jointCount"),
    completedAtMs: Date.now(),
  };
}

function diagnose(latest?: VisualizerLagCompletedSample): string {
  if (!latest) {
    return "waiting for rendered samples";
  }
  const sourceAge = latest.sourceAgeMs ?? 0;
  const apiToBrowser = latest.apiToBrowserMs ?? 0;
  const sourceDelta = latest.sourceDeltaMs ?? 0;
  const browserInterarrival = latest.interarrivalMs ?? 0;
  const parseToPush = latest.parseToPushMs ?? 0;
  const pushToVisible = latest.pushToVisibleMs ?? 0;
  const receiveToVisible = latest.receiveToVisibleMs ?? 0;
  const frameInterval = latest.frameIntervalMs ?? 0;
  const frameWork = latest.frameWorkMs ?? 0;

  if (sourceAge >= 1000) {
    return "controller/backend sample is already old before the browser sees it";
  }
  if (sourceDelta >= CADENCE_WARN_MS) {
    return "controller monitor cadence is below the live visual target";
  }
  if (apiToBrowser >= 500) {
    return "API/SSE delivery is backing up between server and browser";
  }
  if (browserInterarrival >= CADENCE_WARN_MS) {
    return "browser is receiving monitor packets below the live visual target";
  }
  if (parseToPush >= CADENCE_WARN_MS) {
    return "browser-side JSON parsing/sample handling is delaying visual push";
  }
  if (pushToVisible >= 250 || frameInterval >= 250) {
    return "browser render loop/main thread is delaying visual updates";
  }
  if (frameWork >= FRAME_WORK_WARN_MS) {
    return "3D render work is consuming too much of the frame budget";
  }
  if (receiveToVisible >= 250) {
    return "browser-side parsing/scheduling is delaying visual updates";
  }
  return "no large lag in instrumented visual path";
}

function appendCompleted(sample: VisualizerLagCompletedSample): void {
  completed.push(sample);
  if (completed.length > HISTORY_LIMIT) {
    completed.splice(0, completed.length - HISTORY_LIMIT);
  }
}

function hzFromMs(intervalMs: number | undefined): number | undefined {
  return typeof intervalMs === "number" && intervalMs > 0
    ? 1000 / intervalMs
    : undefined;
}

function publishDebugGlobal(force = false): void {
  const now = nowPerfMs();
  if (!force && now - lastGlobalPublishPerfMs < GLOBAL_PUBLISH_INTERVAL_MS) {
    return;
  }
  lastGlobalPublishPerfMs = now;
  const globalTarget = typeof globalThis === "object"
    ? globalThis as Record<string, unknown>
    : null;
  if (!globalTarget) {
    return;
  }
  globalTarget.__GRADIENT_3D_LAG_TRACE__ = {
    snapshot: getVisualizerLagSnapshot(),
    recent: completed.slice(-20),
    pending: Array.from(pending.values()).slice(-20),
  };
}

function maybeLog(snapshot: VisualizerLagSnapshot): void {
  const latest = snapshot.latest;
  if (!latest) {
    return;
  }
  const largest = Math.max(
    latest.sourceAgeMs ?? 0,
    latest.apiToBrowserMs ?? 0,
    latest.receiveToVisibleMs ?? 0,
    latest.pushToVisibleMs ?? 0,
    latest.frameIntervalMs ?? 0,
  );
  const cadenceLargest = Math.max(
    latest.sourceDeltaMs ?? 0,
    latest.interarrivalMs ?? 0,
    latest.parseToPushMs ?? 0,
  );
  const frameWork = latest.frameWorkMs ?? 0;
  const now = nowPerfMs();
  const shouldWarn =
    largest >= LAG_WARN_MS ||
    cadenceLargest >= CADENCE_WARN_MS ||
    frameWork >= FRAME_WORK_WARN_MS;
  const shouldHeartbeat = now - lastTraceHeartbeatPerfMs >= TRACE_HEARTBEAT_MS;
  if (!shouldWarn && !shouldHeartbeat) {
    return;
  }
  if (shouldWarn && now - lastConsoleLogPerfMs < 1000) {
    return;
  }
  if (shouldWarn) {
    lastConsoleLogPerfMs = now;
  } else {
    lastTraceHeartbeatPerfMs = now;
  }
  // Keep this intentionally structured so live tests can copy one console line
  // and immediately see which segment owns the delay.
  const logPayload = {
    diagnosis: snapshot.diagnosis,
    latest,
    average: snapshot.average,
    max: snapshot.max,
    trace: snapshot.trace,
    counters: {
      received: snapshot.received,
      pushed: snapshot.pushed,
      applied: snapshot.applied,
      superseded: snapshot.superseded,
      dropped: snapshot.dropped,
      pending: snapshot.pending,
    },
  };
  if (shouldWarn) {
    console.warn("[GradientOS 3D lag]", logPayload);
  } else {
    console.info("[GradientOS 3D trace]", logPayload);
  }
}

export function recordVisualizerLagReceived(options: {
  source: VisualizerLagSource;
  sourceTimeSec?: number;
  apiReceivedTimeSec?: number;
  apiSequence?: number;
  browserReceiveWallMs: number;
  browserReceivePerfMs: number;
  payloadBytes?: number;
  jointCount?: number;
  usedDisplayJoints?: boolean;
}): number {
  const id = nextId;
  nextId += 1;
  counters.received += 1;
  const interarrivalMs =
    lastReceivePerfMs === null ? undefined : options.browserReceivePerfMs - lastReceivePerfMs;
  lastReceivePerfMs = options.browserReceivePerfMs;
  const sourceDeltaMs =
    typeof options.sourceTimeSec === "number" && lastSourceTimeSec !== null
      ? (options.sourceTimeSec - lastSourceTimeSec) * 1000
      : undefined;
  if (typeof options.sourceTimeSec === "number") {
    lastSourceTimeSec = options.sourceTimeSec;
  }
  const apiSequenceGap =
    typeof options.apiSequence === "number" && lastApiSequence !== null
      ? options.apiSequence - lastApiSequence
      : undefined;
  if (typeof options.apiSequence === "number") {
    lastApiSequence = options.apiSequence;
  }
  pending.set(id, {
    id,
    source: options.source,
    apiSequence: options.apiSequence,
    apiSequenceGap,
    sourceAgeMs:
      typeof options.sourceTimeSec === "number"
        ? options.browserReceiveWallMs - options.sourceTimeSec * 1000
        : undefined,
    sourceToApiMs:
      typeof options.sourceTimeSec === "number" && typeof options.apiReceivedTimeSec === "number"
        ? (options.apiReceivedTimeSec - options.sourceTimeSec) * 1000
        : undefined,
    sourceDeltaMs,
    apiToBrowserMs:
      typeof options.apiReceivedTimeSec === "number"
        ? options.browserReceiveWallMs - options.apiReceivedTimeSec * 1000
        : undefined,
    interarrivalMs,
    browserReceivePerfMs: options.browserReceivePerfMs,
    payloadBytes: options.payloadBytes,
    jointCount: options.jointCount,
    usedDisplayJoints: options.usedDisplayJoints,
  });
  if (pending.size > PENDING_LIMIT) {
    const firstKey = pending.keys().next().value as number | undefined;
    if (typeof firstKey === "number") {
      pending.delete(firstKey);
      counters.dropped += 1;
    }
  }
  publishDebugGlobal();
  return id;
}

export function recordVisualizerLagDropped(id: number | null | undefined): void {
  if (typeof id !== "number") {
    return;
  }
  if (pending.delete(id)) {
    counters.dropped += 1;
  }
  publishDebugGlobal();
}

export function recordVisualizerLagPushed(id: number | null | undefined): number | undefined {
  if (typeof id !== "number") {
    return undefined;
  }
  const sample = pending.get(id);
  if (!sample) {
    return undefined;
  }
  sample.pushPerfMs = nowPerfMs();
  counters.pushed += 1;
  publishDebugGlobal();
  return id;
}

export function recordVisualizerLagSuperseded(id: number | null | undefined): void {
  if (typeof id !== "number") {
    return;
  }
  if (pending.delete(id)) {
    counters.superseded += 1;
  }
  publishDebugGlobal();
}

export function recordVisualizerLagApplied(
  id: number | null | undefined,
  frame: {
    frameTimeMs: number;
    frameIntervalMs?: number;
    frameWorkMs?: number;
  },
): void {
  if (typeof id !== "number") {
    return;
  }
  const sample = pending.get(id);
  if (!sample) {
    return;
  }
  pending.delete(id);
  counters.applied += 1;
  const visiblePerfMs = nowPerfMs();
  const completedSample: VisualizerLagCompletedSample = {
    id: sample.id,
    source: sample.source,
    apiSequence: sample.apiSequence,
    apiSequenceGap: sample.apiSequenceGap,
    sourceAgeMs: sample.sourceAgeMs,
    sourceToApiMs: sample.sourceToApiMs,
    sourceDeltaMs: sample.sourceDeltaMs,
    apiToBrowserMs: sample.apiToBrowserMs,
    interarrivalMs: sample.interarrivalMs,
    parseToPushMs:
      typeof sample.pushPerfMs === "number"
        ? sample.pushPerfMs - sample.browserReceivePerfMs
        : undefined,
    receiveToVisibleMs: visiblePerfMs - sample.browserReceivePerfMs,
    pushToVisibleMs:
      typeof sample.pushPerfMs === "number"
        ? visiblePerfMs - sample.pushPerfMs
        : undefined,
    frameIntervalMs: frame.frameIntervalMs,
    frameWorkMs: frame.frameWorkMs,
    payloadBytes: sample.payloadBytes,
    jointCount: sample.jointCount,
    usedDisplayJoints: sample.usedDisplayJoints,
    completedAtMs: Date.now(),
  };
  appendCompleted(completedSample);
  publishDebugGlobal(true);
  maybeLog(getVisualizerLagSnapshot());
}

export function getVisualizerLagSnapshot(): VisualizerLagSnapshot {
  const latest = completed.length > 0 ? completed[completed.length - 1] : undefined;
  const average = computeAverage(completed);
  const snapshot = {
    ...counters,
    pending: pending.size,
    latest,
    average,
    max: computeMax(completed),
    diagnosis: diagnose(latest),
    trace: {
      sourceHz: hzFromMs(average?.sourceDeltaMs),
      browserReceiveHz: hzFromMs(average?.interarrivalMs),
      visibleHz: hzFromMs(average?.frameIntervalMs),
      lastPublishedAtMs: Date.now(),
    },
  };
  return snapshot;
}
