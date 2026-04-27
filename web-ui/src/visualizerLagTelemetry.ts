export type VisualizerLagSource = "monitor" | "fallback" | "prop";

export type VisualizerLagCompletedSample = {
  id: number;
  source: VisualizerLagSource;
  apiSequence?: number;
  sourceAgeMs?: number;
  apiToBrowserMs?: number;
  interarrivalMs?: number;
  parseToPushMs?: number;
  receiveToVisibleMs?: number;
  pushToVisibleMs?: number;
  frameIntervalMs?: number;
  frameWorkMs?: number;
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
};

type PendingSample = {
  id: number;
  source: VisualizerLagSource;
  apiSequence?: number;
  sourceAgeMs?: number;
  apiToBrowserMs?: number;
  interarrivalMs?: number;
  browserReceivePerfMs: number;
  pushPerfMs?: number;
};

const HISTORY_LIMIT = 120;
const PENDING_LIMIT = 240;
const LAG_WARN_MS = 250;

let nextId = 1;
let lastReceivePerfMs: number | null = null;
let lastConsoleLogPerfMs = 0;
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
    apiToBrowserMs: average("apiToBrowserMs"),
    interarrivalMs: average("interarrivalMs"),
    parseToPushMs: average("parseToPushMs"),
    receiveToVisibleMs: average("receiveToVisibleMs"),
    pushToVisibleMs: average("pushToVisibleMs"),
    frameIntervalMs: average("frameIntervalMs"),
    frameWorkMs: average("frameWorkMs"),
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
    apiToBrowserMs: maxValue("apiToBrowserMs"),
    interarrivalMs: maxValue("interarrivalMs"),
    parseToPushMs: maxValue("parseToPushMs"),
    receiveToVisibleMs: maxValue("receiveToVisibleMs"),
    pushToVisibleMs: maxValue("pushToVisibleMs"),
    frameIntervalMs: maxValue("frameIntervalMs"),
    frameWorkMs: maxValue("frameWorkMs"),
    completedAtMs: Date.now(),
  };
}

function diagnose(latest?: VisualizerLagCompletedSample): string {
  if (!latest) {
    return "waiting for rendered samples";
  }
  const sourceAge = latest.sourceAgeMs ?? 0;
  const apiToBrowser = latest.apiToBrowserMs ?? 0;
  const pushToVisible = latest.pushToVisibleMs ?? 0;
  const receiveToVisible = latest.receiveToVisibleMs ?? 0;
  const frameInterval = latest.frameIntervalMs ?? 0;

  if (sourceAge >= 1000) {
    return "controller/backend sample is already old before the browser sees it";
  }
  if (apiToBrowser >= 500) {
    return "API/SSE delivery is backing up between server and browser";
  }
  if (pushToVisible >= 250 || frameInterval >= 250) {
    return "browser render loop/main thread is delaying visual updates";
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
  const now = nowPerfMs();
  if (largest < LAG_WARN_MS || now - lastConsoleLogPerfMs < 1000) {
    return;
  }
  lastConsoleLogPerfMs = now;
  // Keep this intentionally structured so live tests can copy one console line
  // and immediately see which segment owns the delay.
  console.warn("[GradientOS 3D lag]", {
    diagnosis: snapshot.diagnosis,
    latest,
    average: snapshot.average,
    max: snapshot.max,
    counters: {
      received: snapshot.received,
      pushed: snapshot.pushed,
      applied: snapshot.applied,
      superseded: snapshot.superseded,
      dropped: snapshot.dropped,
      pending: snapshot.pending,
    },
  });
}

export function recordVisualizerLagReceived(options: {
  source: VisualizerLagSource;
  sourceTimeSec?: number;
  apiReceivedTimeSec?: number;
  apiSequence?: number;
  browserReceiveWallMs: number;
  browserReceivePerfMs: number;
}): number {
  const id = nextId;
  nextId += 1;
  counters.received += 1;
  const interarrivalMs =
    lastReceivePerfMs === null ? undefined : options.browserReceivePerfMs - lastReceivePerfMs;
  lastReceivePerfMs = options.browserReceivePerfMs;
  pending.set(id, {
    id,
    source: options.source,
    apiSequence: options.apiSequence,
    sourceAgeMs:
      typeof options.sourceTimeSec === "number"
        ? options.browserReceiveWallMs - options.sourceTimeSec * 1000
        : undefined,
    apiToBrowserMs:
      typeof options.apiReceivedTimeSec === "number"
        ? options.browserReceiveWallMs - options.apiReceivedTimeSec * 1000
        : undefined,
    interarrivalMs,
    browserReceivePerfMs: options.browserReceivePerfMs,
  });
  if (pending.size > PENDING_LIMIT) {
    const firstKey = pending.keys().next().value as number | undefined;
    if (typeof firstKey === "number") {
      pending.delete(firstKey);
      counters.dropped += 1;
    }
  }
  return id;
}

export function recordVisualizerLagDropped(id: number | null | undefined): void {
  if (typeof id !== "number") {
    return;
  }
  if (pending.delete(id)) {
    counters.dropped += 1;
  }
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
  return id;
}

export function recordVisualizerLagSuperseded(id: number | null | undefined): void {
  if (typeof id !== "number") {
    return;
  }
  if (pending.delete(id)) {
    counters.superseded += 1;
  }
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
    sourceAgeMs: sample.sourceAgeMs,
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
    completedAtMs: Date.now(),
  };
  appendCompleted(completedSample);
  maybeLog(getVisualizerLagSnapshot());
}

export function getVisualizerLagSnapshot(): VisualizerLagSnapshot {
  const latest = completed.length > 0 ? completed[completed.length - 1] : undefined;
  const snapshot = {
    ...counters,
    pending: pending.size,
    latest,
    average: computeAverage(completed),
    max: computeMax(completed),
    diagnosis: diagnose(latest),
  };
  return snapshot;
}
