import {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
  type PointerEvent as ReactPointerEvent,
  type ReactNode,
} from "react";
import { createPortal } from "react-dom";
import * as THREE from "three";
import {
  Camera,
  CameraOff,
  Crosshair,
  ChevronDown,
  ChevronRight,
  FolderOpen,
  Flame,
  Home,
  Moon,
  Octagon,
  Play,
  Plug,
  RefreshCcw,
  Route,
  Save,
  Settings,
  Trash2,
  Unplug,
  Undo2,
  Wrench,
  X,
} from "lucide-react";
import { resolveDefaultApiHost, resolveDefaultVisionHost } from "./useEndpoint";
import {
  ArmVisualizer,
  type ArmVisualizerHandle,
  type StepLoadStatus,
  type TopologyEdgeOverlay,
  type StepTransform,
} from "./ArmVisualizer";
import { TelemetryCharts } from "./TelemetryCharts";
import ControlPanel from "./ControlPanel";
import {
  buildProgramTree,
  encodePointsForApi,
  previewFromPlannerPayload,
  previewFromTrajectoryDetail,
  type ProgramNode,
  type ProgramTreeViewMode,
  type Point3,
  type PreviewPlan,
} from "./previewUtils";
import { SidebarRail, type SidebarItem } from "./components/SidebarRail";
import { SidebarDrawer } from "./components/SidebarDrawer";
import { ProgramFeatureTree } from "./components/ProgramFeatureTree";

type Alert = {
  level: "error" | "warning" | "info";
  kind: string;
  message: string;
  servo_ids?: number[];
  ts?: number;
  details?: Record<string, unknown>;
};

type ServoSample = {
  voltage_v?: number;
  temp_c?: number;
  current_a?: number;
  drive_duty_per_mille?: number;
  unloading_condition?: number;
  led_alarm_condition?: number;
  unloading_bits?: string;
  led_alarm_bits?: string;
};

type TelemetryEvent = {
  timestamp: number;
  raw: string;
  joints?: number[];
  gripper?: number;
  servos?: Record<string, ServoSample>;
  alerts?: Alert[];
  weld_active?: boolean;
  weld_type?: string;
};

type Axis3 = { x: number; y: number; z: number };

type RuntimeOffset = {
  position_m: Axis3;
  rotation_deg: Axis3;
};

type ToolOffsetMm = {
  position_mm: Axis3;
  rotation_deg: Axis3;
};

type ToolMeshConfig = {
  asset_path: string;
  scale: number;
  position_mm: Axis3;
  rotation_deg: Axis3;
};

type ToolDefinition = {
  tool_id: string;
  display_name: string;
  description?: string;
  tool_type: string;
  keywords: string[];
  compatible_robot_ids: string[];
  offset: ToolOffsetMm;
  mesh?: ToolMeshConfig | null;
  weld?: Record<string, unknown>;
};

type KinematicsProfileSnapshot = {
  revision: number;
  profile: {
    profile_id: string;
    version: string;
    robot_id: string;
    robot_serial: string;
  };
  offsets: {
    base: RuntimeOffset;
    tool: RuntimeOffset;
  };
};

type RobotPolicyOption = {
  name: string;
  robot_id: string;
  display_name: string;
  version: string;
  default_servo_backend: string;
  default_ik_solver_backend: string;
};

type ActiveRuntimeConfig = {
  robot: {
    name: string;
    robot_id: string;
    display_name: string;
    version: string;
  };
  mode: {
    sim: boolean;
  };
  ik_solver: {
    effective_backend: string;
    source: string;
    robot_default_backend: string;
    override_backend?: string | null;
  };
  servo_backend: {
    effective_backend: string;
    source: string;
    robot_default_backend: string;
    override_backend?: string | null;
  };
  tool?: {
    active_tool_id: string;
    display_name: string;
    tool_type: string;
    source?: string;
    offset?: ToolOffsetMm;
    mesh?: ToolMeshConfig | null;
  };
  allow_unsafe_overrides: boolean;
};

type RuntimeConfigSnapshot = {
  active: ActiveRuntimeConfig | null;
  active_error?: string | null;
  desired: {
    robot: string;
    active_tool_id?: string | null;
    allow_unsafe_overrides?: boolean;
    overrides?: {
      ik_solver_backend?: string | null;
      servo_backend?: string | null;
    };
  };
  meta?: {
    updated_at?: string;
    updated_by?: string;
  };
  restart_required: boolean;
  runtime_config_path?: string;
};

type PersistedSettings = {
  showBoundingBox: boolean;
  collapseLiveCharts: boolean;
  collapseStepImport: boolean;
  collapseTrajectory: boolean;
  collapseWeld: boolean;
  collapseRobotControl: boolean;
  activePanel: SidebarPanelId | null;
  showProgramTree: boolean;
  programTreeViewMode: ProgramTreeViewMode;
  expandedProgramTreeNodeIds: string[];
  selectedProgramNodeId: string | null;
};

type SidebarPanelId = "step" | "trajectory" | "tools" | "weld" | "telemetry";

type TopologyModel = {
  model_id: string;
  filename?: string;
  fingerprint?: string;
  edges: Array<{
    id: string;
    part_id?: string;
    samples: Array<[number, number, number] | { x: number; y: number; z: number }>;
  }>;
};

type WeldSegmentDraft = {
  edgeId: string;
  startS: number;
  endS: number;
  weldType: string;
};

type WeldDraft = {
  modelId: string;
  weldType: string;
  weldName: string;
  segments: WeldSegmentDraft[];
  activeSegmentEdgeId: string | null;
  workAngleDeg: number;
  travelAngleDeg: number;
  spinAngleDeg: number;
  transitionClearanceMm: number;
  postAction: "none" | "return_to_start" | "lift";
};

type WeldProgramRecord = {
  name: string;
  saved_at?: string;
  step: {
    filename: string;
    step_base64: string;
    transform: StepTransform;
  };
  weld_draft?: {
    modelId?: string;
    model_id?: string;
    edgeId?: string;
    edge_id?: string;
    weldType?: string;
    weld_type?: string;
    weldName?: string;
    weld_name?: string;
    workAngleDeg?: number;
    work_angle_deg?: number;
    travelAngleDeg?: number;
    travel_angle_deg?: number;
    spinAngleDeg?: number;
    spin_angle_deg?: number;
    transitionClearanceMm?: number;
    transition_clearance_mm?: number;
    postAction?: "none" | "return_to_start" | "lift";
    post_action?: "none" | "return_to_start" | "lift";
    startS?: number;
    start_s?: number;
    endS?: number;
    end_s?: number;
    segments?: Array<{
      edgeId?: string;
      edge_id?: string;
      startS?: number;
      start_s?: number;
      endS?: number;
      end_s?: number;
      weldType?: string;
      weld_type?: string;
    }>;
    activeSegmentEdgeId?: string | null;
    active_segment_edge_id?: string | null;
  };
  editable_waypoints: Point3[];
  planned_trajectory?: PreviewPlan | null;
};

const WELD_TYPE_OPTIONS = ["fillet", "butt", "lap", "tack/spot", "custom"] as const;
const DRAWER_LABEL_CLASS = "block text-[13px] font-normal text-slate-300";
const DRAWER_INPUT_CLASS =
  "mt-1 w-full rounded border border-slate-600/70 bg-slate-950/70 px-2 py-1 text-[13px] text-slate-100 focus:border-cyan-400/60 focus:outline-none";
const DRAWER_INLINE_INPUT_CLASS =
  "rounded border border-slate-600/70 bg-slate-950/70 px-2 py-1 text-[13px] text-slate-100 focus:border-cyan-400/60 focus:outline-none";
const DRAWER_META_TEXT_CLASS = "text-[12px] text-slate-400";
const DRAWER_SECTION_TITLE_CLASS = "text-[14px] font-semibold text-slate-200";
const DRAWER_ACTION_TEXT_CLASS = "text-[13px] font-semibold";

const WELD_LABEL_CLASS = DRAWER_LABEL_CLASS;
const WELD_INPUT_CLASS = DRAWER_INPUT_CLASS;
const WELD_META_TEXT_CLASS = DRAWER_META_TEXT_CLASS;
const WELD_SECTION_TITLE_CLASS = DRAWER_SECTION_TITLE_CLASS;
const WORK_ANGLE_HELP_LIMIT_DEG = 80;
const TRAVEL_ANGLE_HELP_LIMIT_DEG = 60;

function clamp(value: number, min: number, max: number) {
  return Math.max(min, Math.min(max, value));
}

function pointFromAngle(cx: number, cy: number, radius: number, degFromUp: number) {
  const rad = (degFromUp * Math.PI) / 180;
  return {
    x: cx + Math.sin(rad) * radius,
    y: cy - Math.cos(rad) * radius,
  };
}

function angleFromPointer(
  event: ReactPointerEvent<SVGSVGElement>,
  centerX: number,
  centerY: number,
) {
  const rect = event.currentTarget.getBoundingClientRect();
  const x = event.clientX - rect.left;
  const y = event.clientY - rect.top;
  const dx = x - centerX;
  const dy = centerY - y;
  return (Math.atan2(dx, dy) * 180) / Math.PI;
}

const SETTINGS_STORAGE_KEY = "gradient-ui:settings";
// Keep initial visualizer identity stable before runtime snapshot hydration completes.
const DEFAULT_VISUALIZER_ROBOT_ID = "gradient-05";
const DEFAULT_STEP_TRANSFORM: StepTransform = {
  position: { x: 0, y: 0, z: 0 },
  rotationDeg: { x: 0, y: 0, z: 0 },
  scale: 1,
};

const DEFAULT_RUNTIME_OFFSET: RuntimeOffset = {
  position_m: { x: 0, y: 0, z: 0 },
  rotation_deg: { x: 0, y: 0, z: 0 },
};

function createDefaultMeshConfig(assetPath = "", scale = 1): ToolMeshConfig {
  return {
    asset_path: assetPath,
    scale,
    position_mm: { x: 0, y: 0, z: 0 },
    rotation_deg: { x: 0, y: 0, z: 0 },
  };
}

const DEFAULT_TOOL_DRAFT: ToolDefinition = {
  tool_id: "",
  display_name: "",
  description: "",
  tool_type: "generic",
  keywords: [],
  compatible_robot_ids: [],
  offset: {
    position_mm: { x: 0, y: 0, z: 0 },
    rotation_deg: { x: 0, y: 0, z: 0 },
  },
  mesh: null,
  weld: {},
};

function cloneRuntimeOffset(offset: RuntimeOffset): RuntimeOffset {
  return {
    position_m: { ...offset.position_m },
    rotation_deg: { ...offset.rotation_deg },
  };
}

function cloneToolDefinition(tool: ToolDefinition): ToolDefinition {
  const mesh = tool.mesh
    ? {
        asset_path: tool.mesh.asset_path,
        scale: tool.mesh.scale,
        position_mm: {
          x: Number(tool.mesh.position_mm?.x ?? 0),
          y: Number(tool.mesh.position_mm?.y ?? 0),
          z: Number(tool.mesh.position_mm?.z ?? 0),
        },
        rotation_deg: {
          x: Number(tool.mesh.rotation_deg?.x ?? 0),
          y: Number(tool.mesh.rotation_deg?.y ?? 0),
          z: Number(tool.mesh.rotation_deg?.z ?? 0),
        },
      }
    : null;
  return {
    ...tool,
    keywords: Array.isArray(tool.keywords) ? [...tool.keywords] : [],
    compatible_robot_ids: Array.isArray(tool.compatible_robot_ids)
      ? [...tool.compatible_robot_ids]
      : [],
    offset: {
      position_mm: { ...tool.offset.position_mm },
      rotation_deg: { ...tool.offset.rotation_deg },
    },
    mesh,
    weld: tool.weld ? { ...tool.weld } : {},
  };
}

function loadPersistedSettings(): PersistedSettings {
  const defaults: PersistedSettings = {
    showBoundingBox: true,
    collapseLiveCharts: false,
    collapseStepImport: false,
    collapseTrajectory: false,
    collapseWeld: false,
    collapseRobotControl: false,
    activePanel: "step",
    showProgramTree: true,
    programTreeViewMode: "chronological",
    expandedProgramTreeNodeIds: ["program_root", "setup_primary", "op_chronological", "op_weld"],
    selectedProgramNodeId: null,
  };
  if (typeof window === "undefined") {
    return defaults;
  }
  try {
    const stored = window.localStorage.getItem(SETTINGS_STORAGE_KEY);
    if (!stored) {
      return defaults;
    }
    const parsed = JSON.parse(stored);
    if (parsed && typeof parsed === "object") {
      return {
        showBoundingBox:
          typeof parsed.showBoundingBox === "boolean"
            ? parsed.showBoundingBox
            : defaults.showBoundingBox,
        collapseLiveCharts:
          typeof parsed.collapseLiveCharts === "boolean"
            ? parsed.collapseLiveCharts
            : defaults.collapseLiveCharts,
        collapseStepImport:
          typeof parsed.collapseStepImport === "boolean"
            ? parsed.collapseStepImport
            : defaults.collapseStepImport,
        collapseTrajectory:
          typeof parsed.collapseTrajectory === "boolean"
            ? parsed.collapseTrajectory
            : defaults.collapseTrajectory,
        collapseWeld:
          typeof parsed.collapseWeld === "boolean"
            ? parsed.collapseWeld
            : defaults.collapseWeld,
        collapseRobotControl:
          typeof parsed.collapseRobotControl === "boolean"
            ? parsed.collapseRobotControl
            : defaults.collapseRobotControl,
        activePanel:
          parsed.activePanel === "step" ||
          parsed.activePanel === "trajectory" ||
          parsed.activePanel === "tools" ||
          parsed.activePanel === "weld" ||
          parsed.activePanel === "telemetry" ||
          parsed.activePanel === null
            ? parsed.activePanel
            : defaults.activePanel,
        showProgramTree:
          typeof parsed.showProgramTree === "boolean"
            ? parsed.showProgramTree
            : defaults.showProgramTree,
        programTreeViewMode:
          parsed.programTreeViewMode === "chronological" ||
          parsed.programTreeViewMode === "grouped"
            ? parsed.programTreeViewMode
            : defaults.programTreeViewMode,
        expandedProgramTreeNodeIds: Array.isArray(parsed.expandedProgramTreeNodeIds)
          ? parsed.expandedProgramTreeNodeIds
              .map((entry: unknown) => (typeof entry === "string" ? entry.trim() : ""))
              .filter((entry: string) => entry.length > 0)
          : defaults.expandedProgramTreeNodeIds,
        selectedProgramNodeId:
          typeof parsed.selectedProgramNodeId === "string"
            ? parsed.selectedProgramNodeId
            : defaults.selectedProgramNodeId,
      };
    }
  } catch {
    // ignore malformed storage
  }
  return defaults;
}

function persistSettings(settings: PersistedSettings) {
  if (typeof window === "undefined") {
    return;
  }
  try {
    window.localStorage.setItem(
      SETTINGS_STORAGE_KEY,
      JSON.stringify(settings),
    );
  } catch {
    // best-effort persistence; ignore quota errors
  }
}

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

function toTopologyPoint(value: unknown): Point3 | null {
  if (Array.isArray(value) && value.length >= 3) {
    const x = Number(value[0]);
    const y = Number(value[1]);
    const z = Number(value[2]);
    if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
      return { x, y, z };
    }
  } else if (value && typeof value === "object") {
    const obj = value as Record<string, unknown>;
    const x = Number(obj.x);
    const y = Number(obj.y);
    const z = Number(obj.z);
    if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
      return { x, y, z };
    }
  }
  return null;
}

function toTopologyEdgeOverlay(model: TopologyModel | null): TopologyEdgeOverlay[] {
  if (!model || !Array.isArray(model.edges)) {
    return [];
  }
  return model.edges
    .map((edge) => {
      const points = Array.isArray(edge.samples)
        ? edge.samples
            .map((sample) => toTopologyPoint(sample))
            .filter((p): p is Point3 => p !== null)
        : [];
      return {
        id: edge.id,
        partId: edge.part_id,
        points,
      } as TopologyEdgeOverlay;
    })
    .filter((edge) => edge.points.length >= 2);
}

function computeTopologyOffset(edges: TopologyEdgeOverlay[]): Point3 | null {
  if (!Array.isArray(edges) || edges.length === 0) {
    return null;
  }
  let minX = Infinity;
  let minY = Infinity;
  let minZ = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  let sawPoint = false;

  edges.forEach((edge) => {
    edge.points.forEach((point) => {
      sawPoint = true;
      minX = Math.min(minX, point.x);
      minY = Math.min(minY, point.y);
      minZ = Math.min(minZ, point.z);
      maxX = Math.max(maxX, point.x);
      maxY = Math.max(maxY, point.y);
    });
  });

  if (!sawPoint) {
    return null;
  }
  return {
    x: (minX + maxX) * 0.5,
    y: (minY + maxY) * 0.5,
    z: minZ,
  };
}

function buildStepTransformMatrix(transform: StepTransform): THREE.Matrix4 {
  const safeScale = Number.isFinite(transform.scale) ? Math.max(1e-4, transform.scale) : 1;
  const position = new THREE.Vector3(
    transform.position.x,
    transform.position.y,
    transform.position.z,
  );
  const rotation = new THREE.Euler(
    THREE.MathUtils.degToRad(transform.rotationDeg.x),
    THREE.MathUtils.degToRad(transform.rotationDeg.y),
    THREE.MathUtils.degToRad(transform.rotationDeg.z),
    "XYZ",
  );
  const quaternion = new THREE.Quaternion().setFromEuler(rotation);
  const scale = new THREE.Vector3(safeScale, safeScale, safeScale);
  return new THREE.Matrix4().compose(position, quaternion, scale);
}

function transformTopologyPointToScene(
  point: Point3,
  topologyOffset: Point3 | null,
  stepMatrix: THREE.Matrix4,
): Point3 {
  const localX = topologyOffset ? point.x - topologyOffset.x : point.x;
  const localY = topologyOffset ? point.y - topologyOffset.y : point.y;
  const localZ = topologyOffset ? point.z - topologyOffset.z : point.z;
  const scenePoint = new THREE.Vector3(localX, localY, localZ).applyMatrix4(stepMatrix);
  return { x: scenePoint.x, y: scenePoint.y, z: scenePoint.z };
}

async function fileToBase64(file: File): Promise<string> {
  const bytes = new Uint8Array(await file.arrayBuffer());
  let binary = "";
  const chunkSize = 0x8000;
  for (let i = 0; i < bytes.length; i += chunkSize) {
    const chunk = bytes.subarray(i, i + chunkSize);
    binary += String.fromCharCode(...chunk);
  }
  return window.btoa(binary);
}

function base64ToFile(base64: string, filename: string, mimeType = "application/step"): File {
  const binary = window.atob(base64);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i += 1) {
    bytes[i] = binary.charCodeAt(i);
  }
  return new File([bytes], filename, { type: mimeType });
}

function samplePointOnPolyline(points: Point3[], s: number): Point3 | null {
  if (!Array.isArray(points) || points.length === 0) {
    return null;
  }
  if (points.length === 1) {
    return points[0];
  }
  const clamped = Math.max(0, Math.min(1, s));
  const cumulative: number[] = [0];
  for (let i = 1; i < points.length; i += 1) {
    const a = points[i - 1];
    const b = points[i];
    cumulative.push(
      cumulative[i - 1] +
        Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z),
    );
  }
  const total = cumulative[cumulative.length - 1];
  if (total <= 1e-9) {
    return points[0];
  }
  const target = clamped * total;
  let hi = 1;
  while (hi < cumulative.length && cumulative[hi] < target) {
    hi += 1;
  }
  const lo = Math.max(0, hi - 1);
  const l0 = cumulative[lo];
  const l1 = cumulative[Math.min(hi, cumulative.length - 1)];
  if (Math.abs(l1 - l0) <= 1e-9) {
    return points[Math.min(hi, points.length - 1)];
  }
  const t = (target - l0) / (l1 - l0);
  const p0 = points[lo];
  const p1 = points[Math.min(hi, points.length - 1)];
  return {
    x: p0.x + (p1.x - p0.x) * t,
    y: p0.y + (p1.y - p0.y) * t,
    z: p0.z + (p1.z - p0.z) * t,
  };
}

function sampleSegmentOnPolyline(
  points: Point3[],
  startS: number,
  endS: number,
  sampleCount = 24,
): Point3[] {
  const a = Math.max(0, Math.min(1, startS));
  const b = Math.max(0, Math.min(1, endS));
  const s0 = Math.min(a, b);
  const s1 = Math.max(a, b);
  const count = Math.max(2, sampleCount);
  const out: Point3[] = [];
  for (let i = 0; i < count; i += 1) {
    const t = i / (count - 1);
    const point = samplePointOnPolyline(points, s0 + (s1 - s0) * t);
    if (point) {
      out.push(point);
    }
  }
  return out;
}

function clamp01(value: number): number {
  if (!Number.isFinite(value)) {
    return 0;
  }
  return Math.max(0, Math.min(1, value));
}

function polylineLength(points: Point3[]): number {
  if (!Array.isArray(points) || points.length < 2) {
    return 0;
  }
  let total = 0;
  for (let i = 1; i < points.length; i += 1) {
    total += Math.hypot(
      points[i].x - points[i - 1].x,
      points[i].y - points[i - 1].y,
      points[i].z - points[i - 1].z,
    );
  }
  return total;
}

function segmentBoundsS(segment: WeldSegmentDraft): { startS: number; endS: number } {
  const start = clamp01(segment.startS);
  const end = clamp01(segment.endS);
  return {
    startS: Math.min(start, end),
    endS: Math.max(start, end),
  };
}

function mmFromSegmentS(segment: WeldSegmentDraft, edgeLengthM: number): {
  startMm: number;
  endMm: number;
} {
  const { startS, endS } = segmentBoundsS(segment);
  const lengthM = Math.max(0, Number.isFinite(edgeLengthM) ? edgeLengthM : 0);
  return {
    startMm: startS * lengthM * 1000,
    endMm: endS * lengthM * 1000,
  };
}

function normalizeWeldSegments(
  rawSegments: unknown,
  defaultWeldType: string,
  validEdgeIds?: Set<string>,
): WeldSegmentDraft[] {
  if (!Array.isArray(rawSegments)) {
    return [];
  }
  const unique = new Set<string>();
  const out: WeldSegmentDraft[] = [];
  rawSegments.forEach((entry) => {
    if (!entry || typeof entry !== "object") {
      return;
    }
    const data = entry as Record<string, unknown>;
    const edgeId = String(data.edgeId ?? data.edge_id ?? "").trim();
    if (!edgeId || unique.has(edgeId)) {
      return;
    }
    if (validEdgeIds && !validEdgeIds.has(edgeId)) {
      return;
    }
    out.push({
      edgeId,
      startS: clamp01(Number(data.startS ?? data.start_s ?? 0)),
      endS: clamp01(Number(data.endS ?? data.end_s ?? 1)),
      weldType:
        String(data.weldType ?? data.weld_type ?? defaultWeldType).trim() ||
        defaultWeldType,
    });
    unique.add(edgeId);
  });
  return out;
}

function normalizeWeldDraftRecord(
  raw: WeldProgramRecord["weld_draft"] | null | undefined,
  fallbackModelId: string,
  validEdgeIds?: Set<string>,
): WeldDraft | null {
  if (!raw || typeof raw !== "object") {
    return null;
  }
  const modelId = String(raw.modelId ?? raw.model_id ?? fallbackModelId).trim() || fallbackModelId;
  const weldType = String(raw.weldType ?? raw.weld_type ?? "fillet").trim() || "fillet";
  const weldName = String(raw.weldName ?? raw.weld_name ?? `${weldType} weld`).trim() || `${weldType} weld`;
  const workAngleDeg = Number(raw.workAngleDeg ?? raw.work_angle_deg ?? 45);
  const travelAngleDeg = Number(raw.travelAngleDeg ?? raw.travel_angle_deg ?? 0);
  const spinAngleDeg = Number(raw.spinAngleDeg ?? raw.spin_angle_deg ?? 0);
  const transitionClearanceMm = Number(
    raw.transitionClearanceMm ?? raw.transition_clearance_mm ?? 35,
  );
  const postActionRaw = String(raw.postAction ?? raw.post_action ?? "return_to_start").trim();
  const postAction: "none" | "return_to_start" | "lift" =
    postActionRaw === "none"
      ? "none"
      : postActionRaw === "lift"
        ? "lift"
        : "return_to_start";
  const segments = normalizeWeldSegments(raw.segments, weldType, validEdgeIds);
  if (segments.length === 0) {
    const edgeId = String(raw.edgeId ?? raw.edge_id ?? "").trim();
    if (
      edgeId &&
      (!validEdgeIds || validEdgeIds.has(edgeId))
    ) {
      segments.push({
        edgeId,
        startS: clamp01(Number(raw.startS ?? raw.start_s ?? 0)),
        endS: clamp01(Number(raw.endS ?? raw.end_s ?? 1)),
        weldType,
      });
    }
  }
  if (segments.length === 0) {
    return null;
  }
  const requestedActive = String(
    raw.activeSegmentEdgeId ?? raw.active_segment_edge_id ?? "",
  ).trim();
  const activeSegmentEdgeId =
    requestedActive && segments.some((segment) => segment.edgeId === requestedActive)
      ? requestedActive
      : segments[0].edgeId;
  return {
    modelId,
    weldType,
    weldName,
    segments,
    activeSegmentEdgeId,
    workAngleDeg: Number.isFinite(workAngleDeg) ? workAngleDeg : 45,
    travelAngleDeg: Number.isFinite(travelAngleDeg) ? travelAngleDeg : 0,
    spinAngleDeg: Number.isFinite(spinAngleDeg) ? spinAngleDeg : 0,
    transitionClearanceMm:
      Number.isFinite(transitionClearanceMm) && transitionClearanceMm > 0
        ? transitionClearanceMm
        : 35,
    postAction,
  };
}

type WeldPreviewSection = {
  kind: "weld" | "transition";
  weldType?: string;
  edgeId?: string;
  points: Point3[];
};

function buildWeldPreviewSections(
  draft: WeldDraft,
  topologyEdgeById: Map<string, TopologyEdgeOverlay>,
): WeldPreviewSection[] {
  const sections: WeldPreviewSection[] = [];
  const clearanceM =
    Number.isFinite(draft.transitionClearanceMm) && draft.transitionClearanceMm > 0
      ? draft.transitionClearanceMm / 1000
      : 0.035;
  draft.segments.forEach((segment) => {
    const edge = topologyEdgeById.get(segment.edgeId);
    if (!edge || edge.points.length < 2) {
      return;
    }
    const weldPoints = sampleSegmentOnPolyline(
      edge.points,
      segment.startS,
      segment.endS,
      24,
    );
    if (weldPoints.length === 0) {
      return;
    }
    const lastSection = sections[sections.length - 1];
    const previousWeldSection =
      lastSection?.kind === "weld"
        ? lastSection
        : sections.length > 1 && sections[sections.length - 2]?.kind === "weld"
          ? sections[sections.length - 2]
          : null;
    const previousEnd =
      previousWeldSection && previousWeldSection.points.length > 0
        ? previousWeldSection.points[previousWeldSection.points.length - 1]
        : null;
    const nextStart = weldPoints[0];
    const isContiguous =
      previousEnd &&
      Math.hypot(
        previousEnd.x - nextStart.x,
        previousEnd.y - nextStart.y,
        previousEnd.z - nextStart.z,
      ) < 1e-4;
    const shouldMerge =
      Boolean(previousWeldSection) &&
      isContiguous &&
      previousWeldSection?.weldType === segment.weldType;

    if (shouldMerge && previousWeldSection) {
      if (previousWeldSection.points.length > 0 && weldPoints.length > 0) {
        const first = weldPoints[0];
        const prev = previousWeldSection.points[previousWeldSection.points.length - 1];
        if (Math.hypot(prev.x - first.x, prev.y - first.y, prev.z - first.z) < 1e-6) {
          weldPoints.shift();
        }
      }
      previousWeldSection.points.push(...weldPoints);
      return;
    }

    if (previousEnd && !isContiguous) {
      const liftZ = Math.max(previousEnd.z, nextStart.z) + clearanceM;
      sections.push({
        kind: "transition",
        points: [
          { ...previousEnd },
          { x: previousEnd.x, y: previousEnd.y, z: liftZ },
          { x: nextStart.x, y: nextStart.y, z: liftZ },
          { ...nextStart },
        ],
      });
    }

    sections.push({
      kind: "weld",
      weldType: segment.weldType,
      edgeId: segment.edgeId,
      points: weldPoints,
    });
  });

  return sections;
}

function indexProgramNodes(root: ProgramNode | null): Map<string, ProgramNode> {
  const byId = new Map<string, ProgramNode>();
  if (!root) {
    return byId;
  }
  const stack: ProgramNode[] = [root];
  while (stack.length > 0) {
    const node = stack.pop()!;
    byId.set(node.id, node);
    for (let i = node.children.length - 1; i >= 0; i -= 1) {
      stack.push(node.children[i]);
    }
  }
  return byId;
}

function findWeldProgramNodeIdByEdge(
  nodeById: Map<string, ProgramNode>,
  edgeId: string | null | undefined,
): string | null {
  if (!edgeId) {
    return null;
  }
  for (const [nodeId, node] of nodeById.entries()) {
    if (node.type !== "weldSegment") {
      continue;
    }
    if (node.focus?.weldSegmentEdgeId === edgeId) {
      return nodeId;
    }
  }
  return null;
}

function TelemetryPanel({ latest }: { latest: TelemetryEvent | null }) {
  return (
    <div className="pointer-events-auto w-full">
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

type CollapsibleOverlayPanelProps = {
  title: string;
  collapsed: boolean;
  onToggle: () => void;
  children: ReactNode;
  widthClassName?: string;
};

function CollapsibleOverlayPanel({
  title,
  collapsed,
  onToggle,
  children,
  widthClassName = "w-full max-w-xs",
}: CollapsibleOverlayPanelProps) {
  return (
    <div className={`pointer-events-auto ${widthClassName}`}>
      <button
        type="button"
        onClick={onToggle}
        className="flex w-full items-center justify-between rounded-lg border border-slate-700/60 bg-slate-900/80 px-3 py-2 text-left shadow-md shadow-slate-900/30 backdrop-blur transition hover:border-slate-500/70"
      >
        <span className="text-xs font-semibold uppercase tracking-[0.22em] text-cyan-200/80">
          {title}
        </span>
        {collapsed ? (
          <ChevronRight size={16} className="text-slate-300/80" />
        ) : (
          <ChevronDown size={16} className="text-slate-300/80" />
        )}
      </button>
      {!collapsed && <div className="mt-2">{children}</div>}
    </div>
  );
}

type StepImportPanelProps = {
  stepFileName: string | null;
  stepStatus: StepLoadStatus;
  transform: StepTransform;
  onFileChange: (file: File | null) => void;
  onTransformChange: (
    group: "position" | "rotationDeg",
    axis: "x" | "y" | "z",
    value: number,
  ) => void;
  onScaleChange: (value: number) => void;
  onResetTransform: () => void;
  onClearFile: () => void;
};

function StepImportPanel({
  stepFileName,
  stepStatus,
  transform,
  onFileChange,
  onTransformChange,
  onScaleChange,
  onResetTransform,
  onClearFile,
}: StepImportPanelProps) {
  const statusTone =
    stepStatus.state === "error"
      ? "text-rose-300"
      : stepStatus.state === "loaded"
      ? "text-emerald-300"
      : stepStatus.state === "loading"
      ? "text-amber-200"
      : "text-slate-300/80";
  const parseOrKeep = (raw: string, fallback: number) => {
    const parsed = Number.parseFloat(raw);
    return Number.isFinite(parsed) ? parsed : fallback;
  };

  return (
    <div className="pointer-events-auto w-full">
      <div className="mb-3 flex items-center gap-2">
        <label className={`flex-1 cursor-pointer rounded-lg border border-slate-600/60 bg-slate-900/60 px-3 py-2 text-center ${DRAWER_ACTION_TEXT_CLASS} text-slate-100 transition hover:border-slate-400 hover:text-slate-50`}>
          Load .step/.stp
          <input
            type="file"
            accept=".step,.stp,model/step"
            className="hidden"
            onChange={(event) => {
              const file = event.target.files?.[0] ?? null;
              onFileChange(file);
            }}
          />
        </label>
        <button
          type="button"
          onClick={onClearFile}
          disabled={!stepFileName}
          className={`rounded-lg border border-slate-600/60 bg-slate-900/60 px-3 py-2 ${DRAWER_ACTION_TEXT_CLASS} text-slate-100 transition hover:border-slate-400 hover:text-slate-50 ${
            stepFileName ? "" : "cursor-not-allowed opacity-60"
          }`}
        >
          Clear
        </button>
      </div>
      <p className={`truncate ${DRAWER_META_TEXT_CLASS}`}>
        File:{" "}
        <span className="font-semibold text-slate-100">
          {stepFileName ?? "None"}
        </span>
      </p>
      <p className="mt-1 text-[12px] text-cyan-200/80">
        Frame: world (Z-up). +X red, +Y green, +Z blue.
      </p>
      <p className={`mt-1 text-[12px] ${statusTone}`}>{stepStatus.message}</p>
      <div className="mt-3 grid grid-cols-3 gap-2">
        {(["x", "y", "z"] as const).map((axis) => (
          <label
            key={`pos-${axis}`}
            className="flex flex-col gap-1 rounded-lg border border-slate-700/60 bg-slate-950/40 px-2 py-2 text-[12px] text-slate-300/90"
          >
            P{axis.toUpperCase()} (m)
            <input
              className="rounded bg-slate-900/70 px-2 py-1 text-[13px] text-slate-100 outline-none ring-1 ring-slate-700/70 focus:ring-cyan-500/50"
              type="number"
              step="0.01"
              value={transform.position[axis]}
              onChange={(event) =>
                onTransformChange(
                  "position",
                  axis,
                  parseOrKeep(event.target.value, transform.position[axis]),
                )
              }
            />
          </label>
        ))}
        {(["x", "y", "z"] as const).map((axis) => (
          <label
            key={`rot-${axis}`}
            className="flex flex-col gap-1 rounded-lg border border-slate-700/60 bg-slate-950/40 px-2 py-2 text-[12px] text-slate-300/90"
          >
            R{axis.toUpperCase()} (deg)
            <input
              className="rounded bg-slate-900/70 px-2 py-1 text-[13px] text-slate-100 outline-none ring-1 ring-slate-700/70 focus:ring-cyan-500/50"
              type="number"
              step="1"
              value={transform.rotationDeg[axis]}
              onChange={(event) =>
                onTransformChange(
                  "rotationDeg",
                  axis,
                  parseOrKeep(event.target.value, transform.rotationDeg[axis]),
                )
              }
            />
          </label>
        ))}
        <label className="col-span-3 flex items-center justify-between rounded-lg border border-slate-700/60 bg-slate-950/40 px-2 py-2 text-[13px] text-slate-300/90">
          <span>Scale</span>
          <input
            className="w-24 rounded bg-slate-900/70 px-2 py-1 text-right text-[13px] text-slate-100 outline-none ring-1 ring-slate-700/70 focus:ring-cyan-500/50"
            type="number"
            min="0.01"
            step="0.1"
            value={transform.scale}
            onChange={(event) =>
              onScaleChange(parseOrKeep(event.target.value, transform.scale))
            }
          />
        </label>
      </div>
      <button
        type="button"
        onClick={onResetTransform}
        className={`mt-3 w-full rounded-lg border border-slate-600/60 bg-slate-900/60 px-2 py-2 ${DRAWER_ACTION_TEXT_CLASS} text-slate-100 transition hover:border-slate-400 hover:text-slate-50`}
      >
        Reset Pose
      </button>
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
    <div className="pointer-events-auto w-full">
      <div className="mb-3 flex items-center justify-end">
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
      <div className="text-[13px] leading-[1.35] text-slate-100/90">
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
            <div className={DRAWER_META_TEXT_CLASS}>
              Loaded:{" "}
              <span className="font-semibold text-slate-100">{preview.name}</span>
            </div>
            <div className={DRAWER_META_TEXT_CLASS}>
              Waypoints:{" "}
              <span className="font-semibold text-slate-100">{waypointCount}</span>
            </div>
            {lastPoint && (
              <div className={DRAWER_META_TEXT_CLASS}>
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
          <span className={DRAWER_SECTION_TITLE_CLASS}>
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
          <p className={DRAWER_META_TEXT_CLASS}>Loading trajectories…</p>
        ) : hasSavedTrajectories ? (
          <div className="flex items-center gap-2">
            <select
              className={`flex-1 ${DRAWER_INLINE_INPUT_CLASS} px-3 py-2 focus:ring-2 focus:ring-cyan-500/30`}
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
              className={`rounded-lg border border-slate-600/60 bg-slate-900/60 px-3 py-2 ${DRAWER_ACTION_TEXT_CLASS} text-slate-100 transition hover:border-slate-400 hover:text-slate-50 ${
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
          <p className={DRAWER_META_TEXT_CLASS}>
            No saved trajectories available.
          </p>
        )}
      </div>
    </div>
  );
}

type WeldPanelProps = {
  isConnected: boolean;
  isTopologyLoading: boolean;
  topologyModelId: string | null;
  topologyEdgeCount: number;
  activeEdgeId: string | null;
  selectedEdges: Array<{
    edgeId: string;
    startMm: number;
    endMm: number;
    lengthMm: number;
    weldType: string;
  }>;
  weldSelectionMode: boolean;
  draft: WeldDraft | null;
  isPlanningWeld: boolean;
  isRunning: boolean;
  canRunPreview: boolean;
  weldActive: boolean;
  planningWarnings: string[];
  onToggleSelection: () => void;
  onSelectEdge: (edgeId: string) => void;
  onRemoveEdge: (edgeId: string) => void;
  onPlanFromEdge: () => void;
  onRun: () => void;
  onSetWeldType: (value: string) => void;
  onSetWeldName: (value: string) => void;
  onSetWorkAngleDeg: (value: number) => void;
  onSetTravelAngleDeg: (value: number) => void;
  onSetSpinAngleDeg: (value: number) => void;
  showEndEffectorFrame: boolean;
  onShowEndEffectorFrameChange: (value: boolean) => void;
  onSetTransitionClearanceMm: (value: number) => void;
  onSetPostAction: (value: "none" | "return_to_start" | "lift") => void;
  onSetStartS: (value: number) => void;
  onSetEndS: (value: number) => void;
  weldProgramName: string;
  onWeldProgramNameChange: (value: string) => void;
  onSaveProgram: () => void;
  isSavingProgram: boolean;
  savedPrograms: string[];
  selectedProgram: string;
  onSelectedProgramChange: (value: string) => void;
  onLoadProgram: () => void;
  isLoadingProgram: boolean;
  isProgramListLoading: boolean;
  onRefreshPrograms: () => void;
};

function WeldAngleHelpTooltip({
  workAngleDeg,
  travelAngleDeg,
}: {
  workAngleDeg: number;
  travelAngleDeg: number;
}) {
  const buttonRef = useRef<HTMLButtonElement | null>(null);
  const tooltipRef = useRef<HTMLDivElement | null>(null);
  const [isOpen, setIsOpen] = useState(false);
  const [draggingWork, setDraggingWork] = useState(false);
  const [draggingTravel, setDraggingTravel] = useState(false);
  const [tooltipPosition, setTooltipPosition] = useState({ left: 16, top: 16 });
  const [demoWorkAngle, setDemoWorkAngle] = useState(() =>
    clamp(workAngleDeg, -WORK_ANGLE_HELP_LIMIT_DEG, WORK_ANGLE_HELP_LIMIT_DEG),
  );
  const [demoTravelAngle, setDemoTravelAngle] = useState(() =>
    clamp(travelAngleDeg, -TRAVEL_ANGLE_HELP_LIMIT_DEG, TRAVEL_ANGLE_HELP_LIMIT_DEG),
  );

  const syncFromCurrentAngles = useCallback(() => {
    setDemoWorkAngle(clamp(workAngleDeg, -WORK_ANGLE_HELP_LIMIT_DEG, WORK_ANGLE_HELP_LIMIT_DEG));
    setDemoTravelAngle(
      clamp(travelAngleDeg, -TRAVEL_ANGLE_HELP_LIMIT_DEG, TRAVEL_ANGLE_HELP_LIMIT_DEG),
    );
  }, [workAngleDeg, travelAngleDeg]);

  const updateTooltipPosition = useCallback(() => {
    const button = buttonRef.current;
    if (!button) {
      return;
    }
    const rect = button.getBoundingClientRect();
    const tooltipWidth = tooltipRef.current?.offsetWidth ?? 368;
    const tooltipHeight = tooltipRef.current?.offsetHeight ?? 320;
    const margin = 12;
    const gap = 10;

    let left = rect.right + gap;
    if (left + tooltipWidth > window.innerWidth - margin) {
      left = rect.left - tooltipWidth - gap;
    }
    if (left < margin) {
      left = margin;
    }

    let top = rect.bottom + 8;
    if (top + tooltipHeight > window.innerHeight - margin) {
      top = rect.top - tooltipHeight - 8;
    }
    if (top < margin) {
      top = margin;
    }

    setTooltipPosition({ left, top });
  }, []);

  useEffect(() => {
    if (!isOpen) {
      return;
    }
    const onPointerDown = (event: PointerEvent) => {
      const target = event.target as Node;
      if (buttonRef.current?.contains(target) || tooltipRef.current?.contains(target)) {
        return;
      }
      setIsOpen(false);
    };
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        setIsOpen(false);
      }
    };
    const onViewportChange = () => updateTooltipPosition();

    window.addEventListener("pointerdown", onPointerDown);
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("resize", onViewportChange);
    window.addEventListener("scroll", onViewportChange, true);
    const raf = window.requestAnimationFrame(updateTooltipPosition);
    return () => {
      window.removeEventListener("pointerdown", onPointerDown);
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("resize", onViewportChange);
      window.removeEventListener("scroll", onViewportChange, true);
      window.cancelAnimationFrame(raf);
    };
  }, [isOpen, updateTooltipPosition]);

  const updateWorkFromPointer = useCallback((event: ReactPointerEvent<SVGSVGElement>) => {
    const angle = angleFromPointer(event, 76, 78);
    setDemoWorkAngle(clamp(angle, -WORK_ANGLE_HELP_LIMIT_DEG, WORK_ANGLE_HELP_LIMIT_DEG));
  }, []);

  const updateTravelFromPointer = useCallback((event: ReactPointerEvent<SVGSVGElement>) => {
    const angle = angleFromPointer(event, 76, 78);
    setDemoTravelAngle(clamp(angle, -TRAVEL_ANGLE_HELP_LIMIT_DEG, TRAVEL_ANGLE_HELP_LIMIT_DEG));
  }, []);

  const workTip = pointFromAngle(76, 78, 42, demoWorkAngle);
  const travelTip = pointFromAngle(76, 78, 42, demoTravelAngle);

  const tooltip =
    isOpen && typeof document !== "undefined"
      ? createPortal(
          <div
            ref={tooltipRef}
            className="fixed z-[80] w-[23rem] rounded-lg border border-slate-600/70 bg-slate-950/95 p-3 shadow-xl shadow-slate-950/70 backdrop-blur"
            style={{ left: tooltipPosition.left, top: tooltipPosition.top }}
          >
            <div className="mb-1 text-[12px] font-semibold uppercase tracking-[0.16em] text-cyan-200/90">
              Angle Definitions
            </div>
            <p className="mb-2 text-[12px] leading-5 text-slate-300/90">
              Work angle is side-to-side torch tilt across the joint. Travel angle is forward/back
              tilt along the weld direction. Drag either torch tip to preview how each angle is
              measured.
            </p>
            <div className="grid grid-cols-2 gap-2">
              <div className="rounded border border-slate-700/60 bg-slate-900/65 p-1.5">
                <div className="mb-1 text-[11px] font-semibold text-slate-200">
                  Work ({demoWorkAngle.toFixed(0)} deg)
                </div>
                <svg
                  viewBox="0 0 152 120"
                  className="h-[120px] w-full cursor-grab rounded bg-slate-950/70 active:cursor-grabbing"
                  onPointerDown={(event) => {
                    setDraggingWork(true);
                    event.currentTarget.setPointerCapture(event.pointerId);
                    updateWorkFromPointer(event);
                  }}
                  onPointerMove={(event) => {
                    if (draggingWork) {
                      updateWorkFromPointer(event);
                    }
                  }}
                  onPointerUp={(event) => {
                    setDraggingWork(false);
                    event.currentTarget.releasePointerCapture(event.pointerId);
                  }}
                  onPointerCancel={(event) => {
                    setDraggingWork(false);
                    event.currentTarget.releasePointerCapture(event.pointerId);
                  }}
                >
                  <path d="M 20 110 L 76 78 L 132 110" stroke="#334155" strokeWidth="2" fill="none" />
                  <line
                    x1="76"
                    y1="78"
                    x2="76"
                    y2="24"
                    stroke="#38bdf8"
                    strokeWidth="1.5"
                    strokeDasharray="4 3"
                  />
                  <circle cx="76" cy="78" r="42" stroke="#475569" strokeWidth="1.2" fill="none" />
                  <line
                    x1="76"
                    y1="78"
                    x2={workTip.x}
                    y2={workTip.y}
                    stroke="#f59e0b"
                    strokeWidth="3"
                    strokeLinecap="round"
                  />
                  <circle cx={workTip.x} cy={workTip.y} r="4.5" fill="#f59e0b" />
                  <text x="10" y="20" fill="#94a3b8" fontSize="9">
                    cross-section
                  </text>
                </svg>
              </div>
              <div className="rounded border border-slate-700/60 bg-slate-900/65 p-1.5">
                <div className="mb-1 text-[11px] font-semibold text-slate-200">
                  Travel ({demoTravelAngle.toFixed(0)} deg)
                </div>
                <svg
                  viewBox="0 0 152 120"
                  className="h-[120px] w-full cursor-grab rounded bg-slate-950/70 active:cursor-grabbing"
                  onPointerDown={(event) => {
                    setDraggingTravel(true);
                    event.currentTarget.setPointerCapture(event.pointerId);
                    updateTravelFromPointer(event);
                  }}
                  onPointerMove={(event) => {
                    if (draggingTravel) {
                      updateTravelFromPointer(event);
                    }
                  }}
                  onPointerUp={(event) => {
                    setDraggingTravel(false);
                    event.currentTarget.releasePointerCapture(event.pointerId);
                  }}
                  onPointerCancel={(event) => {
                    setDraggingTravel(false);
                    event.currentTarget.releasePointerCapture(event.pointerId);
                  }}
                >
                  <line x1="20" y1="96" x2="132" y2="96" stroke="#334155" strokeWidth="2" />
                  <line x1="96" y1="90" x2="124" y2="90" stroke="#22c55e" strokeWidth="2" />
                  <polygon points="124,90 116,86 116,94" fill="#22c55e" />
                  <line
                    x1="76"
                    y1="78"
                    x2="76"
                    y2="24"
                    stroke="#38bdf8"
                    strokeWidth="1.5"
                    strokeDasharray="4 3"
                  />
                  <circle cx="76" cy="78" r="42" stroke="#475569" strokeWidth="1.2" fill="none" />
                  <line
                    x1="76"
                    y1="78"
                    x2={travelTip.x}
                    y2={travelTip.y}
                    stroke="#f97316"
                    strokeWidth="3"
                    strokeLinecap="round"
                  />
                  <circle cx={travelTip.x} cy={travelTip.y} r="4.5" fill="#f97316" />
                  <text x="10" y="20" fill="#94a3b8" fontSize="9">
                    seam direction →
                  </text>
                </svg>
              </div>
            </div>
          </div>,
          document.body,
        )
      : null;

  return (
    <div className="inline-flex">
      <button
        ref={buttonRef}
        type="button"
        onClick={() => {
          if (!isOpen) {
            syncFromCurrentAngles();
            setIsOpen(true);
            return;
          }
          setIsOpen(false);
        }}
        className="inline-flex h-5 w-5 items-center justify-center rounded-full border border-cyan-500/50 bg-cyan-500/15 text-[11px] font-semibold text-cyan-100 transition hover:border-cyan-300/70 hover:bg-cyan-500/25"
        aria-label="Explain work and travel angles"
        title="Angle definition help"
      >
        ?
      </button>
      {tooltip}
    </div>
  );
}

function WeldPanel({
  isConnected,
  isTopologyLoading,
  topologyModelId,
  topologyEdgeCount,
  activeEdgeId,
  selectedEdges,
  weldSelectionMode,
  draft,
  isPlanningWeld,
  isRunning,
  canRunPreview,
  weldActive,
  planningWarnings,
  onToggleSelection,
  onSelectEdge,
  onRemoveEdge,
  onPlanFromEdge,
  onRun,
  onSetWeldType,
  onSetWeldName,
  onSetWorkAngleDeg,
  onSetTravelAngleDeg,
  onSetSpinAngleDeg,
  showEndEffectorFrame,
  onShowEndEffectorFrameChange,
  onSetTransitionClearanceMm,
  onSetPostAction,
  onSetStartS,
  onSetEndS,
  weldProgramName,
  onWeldProgramNameChange,
  onSaveProgram,
  isSavingProgram,
  savedPrograms,
  selectedProgram,
  onSelectedProgramChange,
  onLoadProgram,
  isLoadingProgram,
  isProgramListLoading,
  onRefreshPrograms,
}: WeldPanelProps) {
  const activeSegment = draft?.segments.find(
    (segment) => segment.edgeId === draft.activeSegmentEdgeId,
  );
  const activeRow = selectedEdges.find((row) => row.edgeId === activeEdgeId) ?? null;
  const canPlanFromEdge = Boolean(
    topologyModelId &&
      draft &&
      draft.segments.length > 0 &&
      !isPlanningWeld &&
      !isRunning,
  );

  return (
    <div className="pointer-events-auto w-full">
      <div className="space-y-2 text-[13px] leading-[1.35] text-slate-200/90">
        {planningWarnings.length > 0 ? (
          <div className="rounded border border-amber-500/40 bg-amber-500/10 px-2 py-2">
            <div className="mb-1 text-[12px] font-semibold uppercase tracking-[0.14em] text-amber-200">
              Orientation Fallback Warnings
            </div>
            <div className="space-y-1 text-[12px] leading-5 text-amber-100/90">
              {planningWarnings.map((warning) => (
                <div key={`planning-warning-${warning}`} className="rounded border border-amber-500/20 bg-amber-500/5 px-1.5 py-1">
                  {warning}
                </div>
              ))}
            </div>
          </div>
        ) : null}
        <div className="rounded border border-slate-700/50 bg-slate-950/50 px-2 py-2">
          <div className="mb-1 flex items-center justify-between">
            <span className="text-slate-300">Topology Model</span>
            <span className="font-semibold text-slate-100">
              {topologyModelId ? topologyModelId.slice(0, 12) : "none"}
            </span>
          </div>
          <div className={`flex items-center justify-between ${WELD_META_TEXT_CLASS}`}>
            <span>Edges</span>
            <span>{topologyEdgeCount}</span>
          </div>
          <button
            type="button"
            onClick={onToggleSelection}
            disabled={!topologyModelId || isTopologyLoading || !isConnected}
            className={`mt-2 inline-flex w-full items-center justify-center gap-1 rounded border px-2 py-1 text-[13px] font-semibold transition ${
              weldSelectionMode
                ? "border-cyan-400/50 bg-cyan-500/20 text-cyan-100"
                : "border-slate-600/60 bg-slate-900/60 text-slate-200 hover:border-slate-400"
            } ${(!topologyModelId || isTopologyLoading || !isConnected) ? "opacity-60" : ""}`}
          >
            <Crosshair size={14} />
            {weldSelectionMode ? "Edge Select Enabled" : "Enable Edge Select"}
          </button>
          <div className="mt-2 rounded border border-slate-700/50 bg-slate-950/40 p-1">
            <div className={WELD_META_TEXT_CLASS}>
              Selected Edges ({selectedEdges.length})
            </div>
            {selectedEdges.length > 0 ? (
              <div className="max-h-28 space-y-1 overflow-y-auto pr-1">
                {selectedEdges.map((segment) => {
                  const isActive = segment.edgeId === activeEdgeId;
                  return (
                    <div
                      key={`selected-edge-${segment.edgeId}`}
                      className={`flex items-center gap-1 rounded border px-1.5 py-1 ${
                        isActive
                          ? "border-cyan-400/60 bg-cyan-500/15"
                          : "border-slate-700/60 bg-slate-900/40"
                      }`}
                    >
                      <button
                        type="button"
                        className="flex-1 text-left text-[12px] text-slate-200"
                        onClick={() => onSelectEdge(segment.edgeId)}
                        title={segment.edgeId}
                      >
                        <div className="truncate">{segment.edgeId}</div>
                        <div className="text-[11px] text-slate-400">
                          {segment.startMm.toFixed(1)} - {segment.endMm.toFixed(1)} /{" "}
                          {segment.lengthMm.toFixed(1)} mm
                        </div>
                        <div className="text-[11px] text-cyan-300/80">{segment.weldType}</div>
                      </button>
                      <button
                        type="button"
                        className="rounded border border-slate-600/70 bg-slate-900/70 px-1 text-[11px] text-slate-200 hover:border-slate-400"
                        onClick={() => onRemoveEdge(segment.edgeId)}
                        aria-label={`Remove ${segment.edgeId}`}
                      >
                        x
                      </button>
                    </div>
                  );
                })}
              </div>
            ) : (
              <div className={WELD_META_TEXT_CLASS}>No edges selected yet.</div>
            )}
          </div>
        </div>

        <label className={WELD_LABEL_CLASS}>
          Weld Type
          <select
            className={WELD_INPUT_CLASS}
            value={activeSegment?.weldType ?? draft?.weldType ?? "fillet"}
            onChange={(event) => onSetWeldType(event.target.value)}
            disabled={!activeSegment}
          >
            {WELD_TYPE_OPTIONS.map((value) => (
              <option key={value} value={value}>
                {value}
              </option>
            ))}
          </select>
        </label>

        <label className={WELD_LABEL_CLASS}>
          Weld Name
          <input
            className={WELD_INPUT_CLASS}
            value={draft?.weldName ?? ""}
            onChange={(event) => onSetWeldName(event.target.value)}
            disabled={!draft}
          />
        </label>

        <div className="grid grid-cols-2 gap-2">
          <label className={WELD_LABEL_CLASS}>
            <div className="flex items-center justify-between gap-2">
              <span>Work Angle (deg)</span>
              <WeldAngleHelpTooltip
                workAngleDeg={draft?.workAngleDeg ?? 45}
                travelAngleDeg={draft?.travelAngleDeg ?? 0}
              />
            </div>
            <input
              className={WELD_INPUT_CLASS}
              type="number"
              step="1"
              value={Number((draft?.workAngleDeg ?? 45).toFixed(1))}
              onChange={(event) => onSetWorkAngleDeg(Number(event.target.value))}
              disabled={!draft}
            />
          </label>
          <label className={WELD_LABEL_CLASS}>
            Travel Angle (deg)
            <input
              className={WELD_INPUT_CLASS}
              type="number"
              step="1"
              value={Number((draft?.travelAngleDeg ?? 0).toFixed(1))}
              onChange={(event) => onSetTravelAngleDeg(Number(event.target.value))}
              disabled={!draft}
            />
          </label>
        </div>
        <div className="grid grid-cols-2 gap-2">
          <label className={WELD_LABEL_CLASS}>
            Spin Angle (deg)
            <input
              className={WELD_INPUT_CLASS}
              type="number"
              step="1"
              value={Number((draft?.spinAngleDeg ?? 0).toFixed(1))}
              onChange={(event) => onSetSpinAngleDeg(Number(event.target.value))}
              disabled={!draft}
            />
          </label>
        </div>
        <div className="text-[11px] text-slate-400">
          3D guide appears on selected weld segment (cyan=travel, blue=up, green=normal, orange=torch).
        </div>
        <label className="flex items-center gap-2 text-[11px] text-slate-300">
          <input
            type="checkbox"
            className="h-3.5 w-3.5 rounded border border-slate-600 bg-slate-900 text-cyan-400 focus:outline-none focus:ring-2 focus:ring-cyan-500/40"
            checked={showEndEffectorFrame}
            onChange={(event) => onShowEndEffectorFrameChange(event.target.checked)}
          />
          Show tool tip / EE frame
        </label>
        <div className="grid grid-cols-2 gap-2">
          <label className={WELD_LABEL_CLASS}>
            Clearance (mm)
            <input
              className={WELD_INPUT_CLASS}
              type="number"
              min={1}
              step="1"
              value={Number((draft?.transitionClearanceMm ?? 35).toFixed(1))}
              onChange={(event) => onSetTransitionClearanceMm(Number(event.target.value))}
              disabled={!draft}
            />
          </label>
          <label className={WELD_LABEL_CLASS}>
            End Action
            <select
              className={WELD_INPUT_CLASS}
              value={draft?.postAction ?? "return_to_start"}
              onChange={(event) => {
                const value = event.target.value;
                onSetPostAction(
                  value === "none" || value === "lift" ? value : "return_to_start",
                );
              }}
              disabled={!draft}
            >
              <option value="return_to_start">Return to trajectory start</option>
              <option value="lift">Lift</option>
              <option value="none">None</option>
            </select>
          </label>
        </div>

        <div className="rounded border border-slate-700/50 bg-slate-950/40 px-2 py-2">
          <div className={`mb-1 ${WELD_SECTION_TITLE_CLASS}`}>Edge Segment</div>
          <div className={WELD_META_TEXT_CLASS}>
            edge: <span className="text-slate-200">{activeEdgeId ?? "none selected"}</span>
          </div>
          <label className={`${WELD_LABEL_CLASS} mt-2`}>
            Start: {activeRow ? `${activeRow.startMm.toFixed(1)} mm` : "0.0 mm"}
            <input
              className="mt-1 w-full"
              type="range"
              min={0}
              max={1}
              step={0.001}
              value={activeSegment?.startS ?? 0}
              disabled={!activeSegment}
              onChange={(event) => onSetStartS(Number(event.target.value))}
            />
          </label>
          <label className={`${WELD_LABEL_CLASS} mt-1`}>
            End: {activeRow ? `${activeRow.endMm.toFixed(1)} mm` : "0.0 mm"}
            <input
              className="mt-1 w-full"
              type="range"
              min={0}
              max={1}
              step={0.001}
              value={activeSegment?.endS ?? 1}
              disabled={!activeSegment}
              onChange={(event) => onSetEndS(Number(event.target.value))}
            />
          </label>
          <button
            type="button"
            onClick={onPlanFromEdge}
            disabled={!canPlanFromEdge}
            className={`mt-2 w-full rounded border border-slate-600/60 bg-slate-900/60 px-2 py-1 text-[13px] font-semibold text-slate-100 transition hover:border-slate-400 ${
              !canPlanFromEdge ? "opacity-60" : ""
            }`}
          >
            {isPlanningWeld
              ? "Planning..."
              : selectedEdges.length > 1
                ? "Plan Weld From Selected Edges"
                : "Plan Weld From Edge"}
          </button>
        </div>

      </div>
      <div className="mt-3 rounded border border-slate-700/50 bg-slate-950/40 px-2 py-2">
        <div className={`mb-1 ${WELD_SECTION_TITLE_CLASS}`}>Weld Program</div>
        <label className={WELD_LABEL_CLASS}>
          Program Name
          <input
            className={WELD_INPUT_CLASS}
            value={weldProgramName}
            onChange={(event) => onWeldProgramNameChange(event.target.value)}
            placeholder="my_weld_program"
          />
        </label>
        <button
          type="button"
          onClick={onSaveProgram}
          disabled={!draft || isSavingProgram || !weldProgramName.trim()}
          className={`mt-2 inline-flex w-full items-center justify-center gap-2 rounded border border-slate-600/60 bg-slate-900/60 px-2 py-1 text-[13px] font-semibold text-slate-100 transition hover:border-slate-400 ${
            (!draft || isSavingProgram || !weldProgramName.trim()) ? "opacity-60" : ""
          }`}
        >
          <Save size={14} />
          {isSavingProgram ? "Saving..." : "Save Program"}
        </button>
        <div className="mt-2 flex items-center justify-between">
          <span className={WELD_META_TEXT_CLASS}>Saved Programs</span>
          <button
            type="button"
            onClick={onRefreshPrograms}
            className={`rounded border border-slate-600/60 bg-slate-900/60 px-1.5 py-0.5 text-[12px] text-slate-200 ${isProgramListLoading ? "opacity-60" : ""}`}
            disabled={isProgramListLoading}
          >
            {isProgramListLoading ? "..." : "Refresh"}
          </button>
        </div>
        {savedPrograms.length > 0 ? (
          <div className="mt-1 flex items-center gap-2">
            <select
              className={`flex-1 ${WELD_INPUT_CLASS.replace("mt-1 w-full ", "")}`}
              value={selectedProgram}
              onChange={(event) => onSelectedProgramChange(event.target.value)}
            >
              {savedPrograms.map((name) => (
                <option key={name} value={name}>
                  {name}
                </option>
              ))}
            </select>
            <button
              type="button"
              onClick={onLoadProgram}
              disabled={!selectedProgram || isLoadingProgram}
              className={`inline-flex items-center gap-1 rounded border border-slate-600/60 bg-slate-900/60 px-2 py-1 text-[13px] font-semibold text-slate-100 ${
                (!selectedProgram || isLoadingProgram) ? "opacity-60" : ""
              }`}
            >
              <FolderOpen size={14} />
              {isLoadingProgram ? "Loading..." : "Load"}
            </button>
          </div>
        ) : (
          <div className={`mt-1 ${WELD_META_TEXT_CLASS}`}>No saved weld programs.</div>
        )}
      </div>
      <button
        type="button"
        onClick={onRun}
        disabled={!canRunPreview || isRunning || isPlanningWeld}
        className={`mt-3 inline-flex w-full items-center justify-center gap-2 rounded border border-orange-400/40 bg-orange-500/20 px-3 py-2 text-sm font-semibold text-orange-100 transition hover:bg-orange-500/30 ${
          (!canRunPreview || isRunning || isPlanningWeld) ? "opacity-60" : ""
        }`}
      >
        <Play size={14} /> Run Weld Preview
      </button>
    </div>
  );
}

type ToolLibraryPanelProps = {
  robots: RobotPolicyOption[];
  tools: ToolDefinition[];
  filteredTools: ToolDefinition[];
  availableToolTypes: string[];
  selectedToolId: string;
  toolFilterRobotId: string;
  toolFilterType: string;
  toolFilterQuery: string;
  activeRuntimeConfig: ActiveRuntimeConfig | null;
  restartRequired: boolean;
  runtimeConfigError: string | null;
  isRuntimeConfigBusy: boolean;
  isRestartingController: boolean;
  isToolLibraryBusy: boolean;
  toolLibraryError: string | null;
  onSelectedToolIdChange: (value: string) => void;
  onToolFilterRobotIdChange: (value: string) => void;
  onToolFilterTypeChange: (value: string) => void;
  onToolFilterQueryChange: (value: string) => void;
  onRefreshTools: () => void;
  onRefreshRuntimeConfig: () => void;
  onApplyRuntimeConfig: () => void;
  onRestartController: () => void;
  onOpenToolSettings: () => void;
};

function ToolLibraryPanel({
  robots,
  tools,
  filteredTools,
  availableToolTypes,
  selectedToolId,
  toolFilterRobotId,
  toolFilterType,
  toolFilterQuery,
  activeRuntimeConfig,
  restartRequired,
  runtimeConfigError,
  isRuntimeConfigBusy,
  isRestartingController,
  isToolLibraryBusy,
  toolLibraryError,
  onSelectedToolIdChange,
  onToolFilterRobotIdChange,
  onToolFilterTypeChange,
  onToolFilterQueryChange,
  onRefreshTools,
  onRefreshRuntimeConfig,
  onApplyRuntimeConfig,
  onRestartController,
  onOpenToolSettings,
}: ToolLibraryPanelProps) {
  const selectableTools = filteredTools.length > 0 ? filteredTools : tools;
  const selectedOptionId = selectableTools.some((tool) => tool.tool_id === selectedToolId)
    ? selectedToolId
    : (selectableTools[0]?.tool_id ?? "");
  const selectedTool =
    tools.find((tool) => tool.tool_id === selectedOptionId) ??
    tools.find((tool) => tool.tool_id === selectedToolId) ??
    null;

  return (
    <div className="space-y-3">
      <div className="rounded border border-slate-700/60 bg-slate-950/40 px-3 py-2">
        <div className={DRAWER_SECTION_TITLE_CLASS}>Runtime Tool</div>
        <div className={`mt-1 ${DRAWER_META_TEXT_CLASS}`}>
          {activeRuntimeConfig?.tool?.display_name
            ? `Active ${activeRuntimeConfig.tool.display_name} (${activeRuntimeConfig.tool.active_tool_id})`
            : "Active tool unavailable (controller/API not synced yet)."}
        </div>
        <div className={`mt-1 text-[12px] ${restartRequired ? "text-amber-200" : "text-emerald-200"}`}>
          {restartRequired
            ? "Restart required to apply desired tool."
            : "Desired tool is in sync with controller runtime."}
        </div>
        {runtimeConfigError ? (
          <div className="mt-2 rounded border border-rose-500/40 bg-rose-500/10 px-2 py-1 text-xs text-rose-100">
            {runtimeConfigError}
          </div>
        ) : null}
        <div className="mt-2 flex flex-wrap gap-2">
          <button
            type="button"
            onClick={onRefreshRuntimeConfig}
            disabled={isRuntimeConfigBusy || isRestartingController}
            className={`rounded border border-slate-600/70 px-2 py-1 text-xs ${isRuntimeConfigBusy || isRestartingController ? "opacity-60" : "hover:border-slate-400 hover:text-slate-100"}`}
          >
            Refresh Runtime
          </button>
          <button
            type="button"
            onClick={onApplyRuntimeConfig}
            disabled={isRuntimeConfigBusy || isRestartingController || !selectedOptionId}
            className={`rounded bg-cyan-600 px-2 py-1 text-xs font-semibold text-white ${isRuntimeConfigBusy || isRestartingController || !selectedOptionId ? "opacity-60" : "hover:brightness-110"}`}
          >
            Stage Tool
          </button>
          <button
            type="button"
            onClick={onRestartController}
            disabled={!restartRequired || isRuntimeConfigBusy || isRestartingController}
            className={`rounded border border-orange-500/60 px-2 py-1 text-xs font-semibold text-orange-100 ${!restartRequired || isRuntimeConfigBusy || isRestartingController ? "opacity-60" : "hover:border-orange-300 hover:text-orange-50"}`}
          >
            {isRestartingController ? "Restarting..." : "Apply + Restart"}
          </button>
        </div>
      </div>

      <div className="rounded border border-slate-700/60 bg-slate-950/40 px-3 py-2">
        <div className="mb-2 flex items-center justify-between">
          <div className={DRAWER_SECTION_TITLE_CLASS}>Tool Library</div>
          <button
            type="button"
            onClick={onRefreshTools}
            disabled={isToolLibraryBusy || isRuntimeConfigBusy || isRestartingController}
            className={`rounded border border-slate-600/70 px-2 py-1 text-xs ${isToolLibraryBusy || isRuntimeConfigBusy || isRestartingController ? "opacity-60" : "hover:border-slate-400 hover:text-slate-100"}`}
          >
            Refresh
          </button>
        </div>
        <div className="grid gap-2">
          <label className={DRAWER_LABEL_CLASS}>
            Robot Filter
            <select
              className={DRAWER_INPUT_CLASS}
              value={toolFilterRobotId}
              onChange={(event) => onToolFilterRobotIdChange(event.target.value)}
              disabled={isToolLibraryBusy}
            >
              <option value="all">All robots</option>
              {robots.map((robot) => (
                <option key={`tool-filter-robot-${robot.robot_id}`} value={robot.robot_id}>
                  {robot.display_name}
                </option>
              ))}
            </select>
          </label>
          <label className={DRAWER_LABEL_CLASS}>
            Tool Type
            <select
              className={DRAWER_INPUT_CLASS}
              value={toolFilterType}
              onChange={(event) => onToolFilterTypeChange(event.target.value)}
              disabled={isToolLibraryBusy}
            >
              <option value="all">All types</option>
              {availableToolTypes.map((toolType) => (
                <option key={`tool-type-${toolType}`} value={toolType}>
                  {toolType}
                </option>
              ))}
            </select>
          </label>
          <label className={DRAWER_LABEL_CLASS}>
            Keyword
            <input
              className={DRAWER_INPUT_CLASS}
              value={toolFilterQuery}
              onChange={(event) => onToolFilterQueryChange(event.target.value)}
              disabled={isToolLibraryBusy}
              placeholder="tig, torch, custom..."
            />
          </label>
          <label className={DRAWER_LABEL_CLASS}>
            Desired Active Tool
            <select
              className={DRAWER_INPUT_CLASS}
              value={selectedOptionId}
              onChange={(event) => onSelectedToolIdChange(event.target.value)}
              disabled={isToolLibraryBusy || selectableTools.length === 0}
            >
              {selectableTools.map((tool) => (
                <option key={`tool-select-${tool.tool_id}`} value={tool.tool_id}>
                  {tool.display_name} ({tool.tool_id})
                </option>
              ))}
            </select>
          </label>
        </div>
        {toolLibraryError ? (
          <div className="mt-2 rounded border border-rose-500/40 bg-rose-500/10 px-2 py-1 text-xs text-rose-100">
            {toolLibraryError}
          </div>
        ) : null}
        <div className={`mt-2 ${DRAWER_META_TEXT_CLASS}`}>
          Drop-in format: <code>tools/library/&lt;tool_id&gt;/tool.json</code> + local STL/GLB/GLTF.
        </div>
      </div>

      {selectedTool ? (
        <div className="rounded border border-slate-700/60 bg-slate-950/40 px-3 py-2">
          <div className={DRAWER_SECTION_TITLE_CLASS}>Selected Tool Details</div>
          <div className="mt-1 text-[13px] text-slate-100">
            {selectedTool.display_name} <span className="text-slate-400">({selectedTool.tool_id})</span>
          </div>
          <div className={`mt-1 ${DRAWER_META_TEXT_CLASS}`}>
            Type: {selectedTool.tool_type} · Compat:{" "}
            {selectedTool.compatible_robot_ids.length > 0
              ? selectedTool.compatible_robot_ids.join(", ")
              : "all robots"}
          </div>
          <div className={`mt-2 ${DRAWER_META_TEXT_CLASS}`}>
            Offset mm: X {selectedTool.offset.position_mm.x.toFixed(1)}, Y {selectedTool.offset.position_mm.y.toFixed(1)}, Z {selectedTool.offset.position_mm.z.toFixed(1)}
          </div>
          <div className={DRAWER_META_TEXT_CLASS}>
            Rotation deg: X {selectedTool.offset.rotation_deg.x.toFixed(1)}, Y {selectedTool.offset.rotation_deg.y.toFixed(1)}, Z {selectedTool.offset.rotation_deg.z.toFixed(1)}
          </div>
        </div>
      ) : null}

      <button
        type="button"
        onClick={onOpenToolSettings}
        className="inline-flex w-full items-center justify-center rounded border border-slate-600/70 bg-slate-900/60 px-3 py-2 text-[13px] font-semibold text-slate-100 transition hover:border-slate-400"
      >
        Open Full Tool Editor
      </button>
    </div>
  );
}

type SettingsDialogTab = "general" | "tools" | "kinematics";

type SettingsDialogProps = {
  isOpen: boolean;
  initialTab: SettingsDialogTab;
  apiHost: string;
  visionHost: string;
  showBoundingBox: boolean;
  robots: RobotPolicyOption[];
  selectedRobotName: string;
  tools: ToolDefinition[];
  selectedToolId: string;
  toolFilterRobotId: string;
  toolFilterType: string;
  toolFilterQuery: string;
  toolDraft: ToolDefinition;
  activeRuntimeConfig: ActiveRuntimeConfig | null;
  restartRequired: boolean;
  runtimeConfigError: string | null;
  isRuntimeConfigBusy: boolean;
  isRestartingController: boolean;
  isToolLibraryBusy: boolean;
  toolLibraryError: string | null;
  kinematics: KinematicsProfileSnapshot | null;
  baseOffsetDraft: RuntimeOffset;
  toolOffsetDraft: RuntimeOffset;
  isKinematicsBusy: boolean;
  kinematicsError: string | null;
  onHostChange: (value: string) => void;
  onVisionHostChange: (value: string) => void;
  onShowBoundingBoxChange: (value: boolean) => void;
  onSelectedRobotNameChange: (value: string) => void;
  onSelectedToolIdChange: (value: string) => void;
  onToolFilterRobotIdChange: (value: string) => void;
  onToolFilterTypeChange: (value: string) => void;
  onToolFilterQueryChange: (value: string) => void;
  onToolDraftChange: (value: ToolDefinition) => void;
  onRefreshTools: () => void;
  onNewToolDraft: () => void;
  onCreateTool: () => void;
  onUpdateTool: () => void;
  onDeleteTool: () => void;
  onRefreshRuntimeConfig: () => void;
  onApplyRuntimeConfig: () => void;
  onRestartController: () => void;
  onOffsetDraftChange: (
    target: "base" | "tool",
    group: "position_m" | "rotation_deg",
    axis: "x" | "y" | "z",
    value: number,
  ) => void;
  onRefreshKinematics: () => void;
  onApplyOffsets: () => void;
  onResetOffsets: () => void;
  onClose: () => void;
};

function SettingsDialog({
  isOpen,
  initialTab,
  apiHost,
  visionHost,
  showBoundingBox,
  robots,
  selectedRobotName,
  tools,
  selectedToolId,
  toolFilterRobotId,
  toolFilterType,
  toolFilterQuery,
  toolDraft,
  activeRuntimeConfig,
  restartRequired,
  runtimeConfigError,
  isRuntimeConfigBusy,
  isRestartingController,
  isToolLibraryBusy,
  toolLibraryError,
  kinematics,
  baseOffsetDraft,
  toolOffsetDraft,
  isKinematicsBusy,
  kinematicsError,
  onHostChange,
  onVisionHostChange,
  onShowBoundingBoxChange,
  onSelectedRobotNameChange,
  onSelectedToolIdChange,
  onToolFilterRobotIdChange,
  onToolFilterTypeChange,
  onToolFilterQueryChange,
  onToolDraftChange,
  onRefreshTools,
  onNewToolDraft,
  onCreateTool,
  onUpdateTool,
  onDeleteTool,
  onRefreshRuntimeConfig,
  onApplyRuntimeConfig,
  onRestartController,
  onOffsetDraftChange,
  onRefreshKinematics,
  onApplyOffsets,
  onResetOffsets,
  onClose,
}: SettingsDialogProps) {
  const [activeTab, setActiveTab] = useState<SettingsDialogTab>(initialTab);

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

  useEffect(() => {
    if (!isOpen) {
      return;
    }
    setActiveTab(initialTab);
  }, [initialTab, isOpen]);

  if (!isOpen) {
    return null;
  }

  const axisOrder: Array<"x" | "y" | "z"> = ["x", "y", "z"];
  const selectedRobot =
    robots.find((robot) => robot.name === selectedRobotName) ??
    robots.find((robot) => robot.robot_id === selectedRobotName) ??
    null;
  const renderOffsetEditor = (
    title: string,
    target: "base" | "tool",
    value: RuntimeOffset,
  ) => (
    <div className="rounded-xl border border-slate-700/60 bg-slate-950/40 p-3">
      <div className="mb-2 text-xs font-semibold uppercase tracking-[0.2em] text-cyan-200/80">
        {title}
      </div>
      <div className="grid grid-cols-3 gap-2">
        {axisOrder.map((axis) => (
          <label key={`${target}:p:${axis}`} className="text-xs text-slate-300/90">
            Pos {axis.toUpperCase()} (m)
            <input
              type="number"
              step="0.001"
              className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
              value={value.position_m[axis]}
              onChange={(event) =>
                onOffsetDraftChange(
                  target,
                  "position_m",
                  axis,
                  Number(event.target.value),
                )
              }
            />
          </label>
        ))}
        {axisOrder.map((axis) => (
          <label key={`${target}:r:${axis}`} className="text-xs text-slate-300/90">
            Rot {axis.toUpperCase()} (deg)
            <input
              type="number"
              step="0.1"
              className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
              value={value.rotation_deg[axis]}
              onChange={(event) =>
                onOffsetDraftChange(
                  target,
                  "rotation_deg",
                  axis,
                  Number(event.target.value),
                )
              }
            />
          </label>
        ))}
      </div>
    </div>
  );

  const previewDeltaMm =
    Math.hypot(baseOffsetDraft.position_m.x, baseOffsetDraft.position_m.y, baseOffsetDraft.position_m.z) * 1000
    + Math.hypot(toolOffsetDraft.position_m.x, toolOffsetDraft.position_m.y, toolOffsetDraft.position_m.z) * 1000;
  const availableToolTypes = Array.from(
    new Set(tools.map((tool) => tool.tool_type).filter((token) => token.trim().length > 0)),
  ).sort((a, b) => a.localeCompare(b));
  const filteredTools = tools.filter((tool) => {
    if (toolFilterRobotId && toolFilterRobotId !== "all") {
      const compat = Array.isArray(tool.compatible_robot_ids) ? tool.compatible_robot_ids : [];
      if (compat.length > 0 && !compat.includes(toolFilterRobotId)) {
        return false;
      }
    }
    if (toolFilterType && toolFilterType !== "all" && tool.tool_type !== toolFilterType) {
      return false;
    }
    const query = toolFilterQuery.trim().toLowerCase();
    if (!query) {
      return true;
    }
    const haystack = [
      tool.tool_id,
      tool.display_name,
      tool.description ?? "",
      tool.tool_type,
      ...tool.keywords,
    ]
      .join(" ")
      .toLowerCase();
    return haystack.includes(query);
  });
  const toolAxisOrder: Array<"x" | "y" | "z"> = ["x", "y", "z"];

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
      <div className="flex w-full max-w-3xl max-h-[calc(100vh-3rem)] flex-col overflow-hidden rounded-2xl border border-slate-700/60 bg-slate-900/90 shadow-2xl shadow-slate-950/60">
        <div className="flex shrink-0 items-center justify-between border-b border-slate-700/50 px-6 py-4">
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
        <div className="shrink-0 border-b border-slate-700/40 px-6 py-3">
          <div className="flex flex-wrap gap-2">
            {([
              { id: "general", label: "General" },
              { id: "tools", label: "Tool Library" },
              { id: "kinematics", label: "Kinematics" },
            ] as const).map((tab) => {
              const selected = activeTab === tab.id;
              return (
                <button
                  key={`settings-tab-${tab.id}`}
                  type="button"
                  onClick={() => setActiveTab(tab.id)}
                  className={`rounded border px-3 py-1.5 text-xs font-semibold transition ${
                    selected
                      ? "border-cyan-400/70 bg-cyan-500/15 text-cyan-100"
                      : "border-slate-600/70 bg-slate-900/60 text-slate-200 hover:border-slate-400 hover:text-slate-100"
                  }`}
                >
                  {tab.label}
                </button>
              );
            })}
          </div>
        </div>
        <div className="gradient-scrollbar min-h-0 overflow-y-auto px-6 py-4">
          <div className="grid gap-4">
          <div className="flex flex-col gap-2">
            {activeTab === "general" ? (
              <>
            <label className="text-sm font-medium text-slate-200/90">
            Gradient API Host
            <input
              className="mt-1 w-full rounded-lg border border-slate-600/70 bg-slate-950/60 px-4 py-2 text-base text-slate-100 placeholder:text-slate-400 focus:border-cyan-400/60 focus:outline-none focus:ring-2 focus:ring-cyan-500/30"
              type="text"
              value={apiHost}
              onChange={(event) => onHostChange(event.target.value)}
              placeholder="http://localhost:4000"
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
            <div className="mt-4 rounded-xl border border-slate-700/60 bg-slate-950/40 p-3">
              <div className="mb-2 flex items-center justify-between">
                <div>
                  <div className="text-xs font-semibold uppercase tracking-[0.2em] text-cyan-200/80">
                    Robot Runtime Policy
                  </div>
                  <div className="text-xs text-slate-400">
                    {activeRuntimeConfig
                      ? `active ${activeRuntimeConfig.robot.display_name} · IK ${activeRuntimeConfig.ik_solver.effective_backend}`
                      : "Active controller runtime unavailable"}
                  </div>
                </div>
                <button
                  type="button"
                  onClick={onRefreshRuntimeConfig}
                  disabled={isRuntimeConfigBusy || isRestartingController}
                  className={`rounded border border-slate-600/70 px-2 py-1 text-xs ${isRuntimeConfigBusy || isRestartingController ? "opacity-60" : "hover:border-slate-400 hover:text-slate-100"}`}
                >
                  Refresh
                </button>
              </div>
              <label className="text-xs text-slate-300/90">
                Desired Robot
                <select
                  className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                  value={selectedRobotName}
                  onChange={(event) => onSelectedRobotNameChange(event.target.value)}
                  disabled={isRuntimeConfigBusy || isRestartingController || robots.length === 0}
                >
                  {robots.map((robot) => (
                    <option key={`robot-opt-${robot.name}`} value={robot.name}>
                      {robot.display_name} ({robot.name})
                    </option>
                  ))}
                </select>
              </label>
              <div className="mt-2 text-xs text-slate-300/90">
                Solver policy:{" "}
                <span className="font-semibold text-cyan-100">
                  {selectedRobot?.default_ik_solver_backend ?? "unknown"}
                </span>
                {" · "}
                Hardware backend:{" "}
                <span className="font-semibold text-cyan-100">
                  {selectedRobot?.default_servo_backend ?? "unknown"}
                </span>
              </div>
              {activeRuntimeConfig?.ik_solver?.source === "dev_override" ||
              activeRuntimeConfig?.servo_backend?.source === "dev_override" ? (
                <div className="mt-2 rounded border border-amber-500/40 bg-amber-500/10 px-2 py-1 text-xs text-amber-100">
                  Controller is running with development overrides, not strict robot policy.
                </div>
              ) : null}
              <div className="mt-2 text-xs">
                <span className={restartRequired ? "text-amber-200" : "text-emerald-200"}>
                  {restartRequired ? "Restart required to apply desired robot policy." : "Controller is in sync with desired robot policy."}
                </span>
              </div>
              {runtimeConfigError ? (
                <div className="mt-2 rounded border border-rose-500/40 bg-rose-500/10 px-2 py-1 text-xs text-rose-100">
                  {runtimeConfigError}
                </div>
              ) : null}
              <div className="mt-2 flex gap-2">
                <button
                  type="button"
                  onClick={onApplyRuntimeConfig}
                  disabled={isRuntimeConfigBusy || isRestartingController || !selectedRobotName}
                  className={`rounded bg-cyan-600 px-3 py-1.5 text-xs font-semibold text-white ${isRuntimeConfigBusy || isRestartingController || !selectedRobotName ? "opacity-60" : "hover:brightness-110"}`}
                >
                  Apply Runtime Config
                </button>
                <button
                  type="button"
                  onClick={onRestartController}
                  disabled={!restartRequired || isRuntimeConfigBusy || isRestartingController}
                  className={`rounded border border-orange-500/60 px-3 py-1.5 text-xs font-semibold text-orange-100 ${!restartRequired || isRuntimeConfigBusy || isRestartingController ? "opacity-60" : "hover:border-orange-300 hover:text-orange-50"}`}
                >
                  {isRestartingController ? "Restarting..." : "Apply + Restart Controller"}
                </button>
              </div>
            </div>
            </>
            ) : null}
            {activeTab === "tools" ? (
            <div className="rounded-xl border border-slate-700/60 bg-slate-950/40 p-3">
              <div className="mb-2 flex items-center justify-between">
                <div>
                  <div className="text-xs font-semibold uppercase tracking-[0.2em] text-cyan-200/80">
                    Tool Library
                  </div>
                  <div className="text-xs text-slate-400">
                    {activeRuntimeConfig?.tool?.display_name
                      ? `active ${activeRuntimeConfig.tool.display_name} (${activeRuntimeConfig.tool.active_tool_id})`
                      : "Select and apply an active tool profile"}
                  </div>
                </div>
                <button
                  type="button"
                  onClick={onRefreshTools}
                  disabled={isToolLibraryBusy || isRuntimeConfigBusy || isRestartingController}
                  className={`rounded border border-slate-600/70 px-2 py-1 text-xs ${isToolLibraryBusy || isRuntimeConfigBusy || isRestartingController ? "opacity-60" : "hover:border-slate-400 hover:text-slate-100"}`}
                >
                  Refresh
                </button>
              </div>
              <div className="grid gap-2 md:grid-cols-3">
                <label className="text-xs text-slate-300/90">
                  Robot Filter
                  <select
                    className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                    value={toolFilterRobotId}
                    onChange={(event) => onToolFilterRobotIdChange(event.target.value)}
                    disabled={isToolLibraryBusy}
                  >
                    <option value="all">All robots</option>
                    {robots.map((robot) => (
                      <option key={`tool-filter-robot-${robot.robot_id}`} value={robot.robot_id}>
                        {robot.display_name}
                      </option>
                    ))}
                  </select>
                </label>
                <label className="text-xs text-slate-300/90">
                  Tool Type
                  <select
                    className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                    value={toolFilterType}
                    onChange={(event) => onToolFilterTypeChange(event.target.value)}
                    disabled={isToolLibraryBusy}
                  >
                    <option value="all">All types</option>
                    {availableToolTypes.map((toolType) => (
                      <option key={`tool-type-${toolType}`} value={toolType}>
                        {toolType}
                      </option>
                    ))}
                  </select>
                </label>
                <label className="text-xs text-slate-300/90">
                  Keyword
                  <input
                    className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                    value={toolFilterQuery}
                    onChange={(event) => onToolFilterQueryChange(event.target.value)}
                    disabled={isToolLibraryBusy}
                    placeholder="tig, torch, custom..."
                  />
                </label>
              </div>
              <label className="mt-2 block text-xs text-slate-300/90">
                Desired Active Tool
                <select
                  className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                  value={selectedToolId}
                  onChange={(event) => onSelectedToolIdChange(event.target.value)}
                  disabled={isToolLibraryBusy || filteredTools.length === 0}
                >
                  {filteredTools.map((tool) => (
                    <option key={`tool-opt-${tool.tool_id}`} value={tool.tool_id}>
                      {tool.display_name} ({tool.tool_id})
                    </option>
                  ))}
                </select>
              </label>
              <div className="mt-3 grid gap-2 md:grid-cols-2">
                <label className="text-xs text-slate-300/90">
                  Tool ID
                  <input
                    className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                    value={toolDraft.tool_id}
                    onChange={(event) =>
                      onToolDraftChange({ ...toolDraft, tool_id: event.target.value })
                    }
                    disabled={isToolLibraryBusy}
                  />
                </label>
                <label className="text-xs text-slate-300/90">
                  Display Name
                  <input
                    className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                    value={toolDraft.display_name}
                    onChange={(event) =>
                      onToolDraftChange({ ...toolDraft, display_name: event.target.value })
                    }
                    disabled={isToolLibraryBusy}
                  />
                </label>
                <label className="text-xs text-slate-300/90">
                  Tool Type
                  <input
                    className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                    value={toolDraft.tool_type}
                    onChange={(event) =>
                      onToolDraftChange({ ...toolDraft, tool_type: event.target.value })
                    }
                    disabled={isToolLibraryBusy}
                  />
                </label>
                <label className="text-xs text-slate-300/90">
                  Keywords (comma separated)
                  <input
                    className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                    value={toolDraft.keywords.join(", ")}
                    onChange={(event) =>
                      onToolDraftChange({
                        ...toolDraft,
                        keywords: event.target.value
                          .split(",")
                          .map((item) => item.trim())
                          .filter((item) => item.length > 0),
                      })
                    }
                    disabled={isToolLibraryBusy}
                  />
                </label>
                <label className="text-xs text-slate-300/90 md:col-span-2">
                  Compatible Robot IDs (comma separated, empty = all)
                  <input
                    className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                    value={toolDraft.compatible_robot_ids.join(", ")}
                    onChange={(event) =>
                      onToolDraftChange({
                        ...toolDraft,
                        compatible_robot_ids: event.target.value
                          .split(",")
                          .map((item) => item.trim())
                          .filter((item) => item.length > 0),
                      })
                    }
                    disabled={isToolLibraryBusy}
                  />
                </label>
                <label className="text-xs text-slate-300/90 md:col-span-2">
                  Description
                  <input
                    className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                    value={toolDraft.description ?? ""}
                    onChange={(event) =>
                      onToolDraftChange({ ...toolDraft, description: event.target.value })
                    }
                    disabled={isToolLibraryBusy}
                  />
                </label>
                {toolAxisOrder.map((axis) => (
                  <label key={`tool-offset-mm-${axis}`} className="text-xs text-slate-300/90">
                    Offset {axis.toUpperCase()} (mm)
                    <input
                      type="number"
                      step="0.1"
                      className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                      value={toolDraft.offset.position_mm[axis]}
                      onChange={(event) =>
                        onToolDraftChange({
                          ...toolDraft,
                          offset: {
                            ...toolDraft.offset,
                            position_mm: {
                              ...toolDraft.offset.position_mm,
                              [axis]: Number(event.target.value),
                            },
                          },
                        })
                      }
                      disabled={isToolLibraryBusy}
                    />
                  </label>
                ))}
                {toolAxisOrder.map((axis) => (
                  <label key={`tool-rot-deg-${axis}`} className="text-xs text-slate-300/90">
                    Rot {axis.toUpperCase()} (deg)
                    <input
                      type="number"
                      step="0.1"
                      className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                      value={toolDraft.offset.rotation_deg[axis]}
                      onChange={(event) =>
                        onToolDraftChange({
                          ...toolDraft,
                          offset: {
                            ...toolDraft.offset,
                            rotation_deg: {
                              ...toolDraft.offset.rotation_deg,
                              [axis]: Number(event.target.value),
                            },
                          },
                        })
                      }
                      disabled={isToolLibraryBusy}
                    />
                  </label>
                ))}
                <label className="text-xs text-slate-300/90 md:col-span-2">
                  Mesh Asset Path (optional)
                  <input
                    className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                    value={toolDraft.mesh?.asset_path ?? ""}
                    onChange={(event) =>
                      onToolDraftChange({
                        ...toolDraft,
                        mesh: event.target.value.trim()
                          ? {
                              ...(toolDraft.mesh ?? createDefaultMeshConfig()),
                              asset_path: event.target.value,
                            }
                          : null,
                      })
                    }
                    disabled={isToolLibraryBusy}
                  />
                </label>
                <label className="text-xs text-slate-300/90 md:col-span-2">
                  Mesh Scale
                  <input
                    type="number"
                    step="0.001"
                    className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                    value={toolDraft.mesh?.scale ?? 1}
                    onChange={(event) =>
                      onToolDraftChange({
                        ...toolDraft,
                        mesh: {
                          ...(toolDraft.mesh ?? createDefaultMeshConfig()),
                          asset_path: toolDraft.mesh?.asset_path ?? "",
                          scale: Number(event.target.value),
                        },
                      })
                    }
                    disabled={isToolLibraryBusy}
                  />
                </label>
                {toolAxisOrder.map((axis) => (
                  <label key={`mesh-offset-mm-${axis}`} className="text-xs text-slate-300/90">
                    Mesh Offset {axis.toUpperCase()} (mm, J6 frame)
                    <input
                      type="number"
                      step="0.1"
                      className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                      value={toolDraft.mesh?.position_mm?.[axis] ?? 0}
                      onChange={(event) =>
                        onToolDraftChange({
                          ...toolDraft,
                          mesh: {
                            ...(toolDraft.mesh ?? createDefaultMeshConfig()),
                            position_mm: {
                              ...(toolDraft.mesh?.position_mm ?? { x: 0, y: 0, z: 0 }),
                              [axis]: Number(event.target.value),
                            },
                          },
                        })
                      }
                      disabled={isToolLibraryBusy}
                    />
                  </label>
                ))}
                {toolAxisOrder.map((axis) => (
                  <label key={`mesh-rot-deg-${axis}`} className="text-xs text-slate-300/90">
                    Mesh Rot {axis.toUpperCase()} (deg, J6 frame)
                    <input
                      type="number"
                      step="0.1"
                      className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                      value={toolDraft.mesh?.rotation_deg?.[axis] ?? 0}
                      onChange={(event) =>
                        onToolDraftChange({
                          ...toolDraft,
                          mesh: {
                            ...(toolDraft.mesh ?? createDefaultMeshConfig()),
                            rotation_deg: {
                              ...(toolDraft.mesh?.rotation_deg ?? { x: 0, y: 0, z: 0 }),
                              [axis]: Number(event.target.value),
                            },
                          },
                        })
                      }
                      disabled={isToolLibraryBusy}
                    />
                  </label>
                ))}
              </div>
              {toolLibraryError ? (
                <div className="mt-2 rounded border border-rose-500/40 bg-rose-500/10 px-2 py-1 text-xs text-rose-100">
                  {toolLibraryError}
                </div>
              ) : null}
              <div className="mt-2 flex flex-wrap gap-2">
                <button
                  type="button"
                  onClick={onNewToolDraft}
                  disabled={isToolLibraryBusy}
                  className={`rounded border border-slate-600/70 px-3 py-1.5 text-xs font-semibold text-slate-200 ${isToolLibraryBusy ? "opacity-60" : "hover:border-slate-400 hover:text-slate-100"}`}
                >
                  New Draft
                </button>
                <button
                  type="button"
                  onClick={onCreateTool}
                  disabled={isToolLibraryBusy}
                  className={`rounded bg-cyan-700 px-3 py-1.5 text-xs font-semibold text-white ${isToolLibraryBusy ? "opacity-60" : "hover:brightness-110"}`}
                >
                  Save New Tool
                </button>
                <button
                  type="button"
                  onClick={onUpdateTool}
                  disabled={isToolLibraryBusy || !toolDraft.tool_id}
                  className={`rounded bg-cyan-600 px-3 py-1.5 text-xs font-semibold text-white ${isToolLibraryBusy || !toolDraft.tool_id ? "opacity-60" : "hover:brightness-110"}`}
                >
                  Update Tool
                </button>
                <button
                  type="button"
                  onClick={onDeleteTool}
                  disabled={isToolLibraryBusy || !toolDraft.tool_id || toolDraft.tool_id === "identity"}
                  className={`rounded border border-rose-500/60 px-3 py-1.5 text-xs font-semibold text-rose-100 ${isToolLibraryBusy || !toolDraft.tool_id || toolDraft.tool_id === "identity" ? "opacity-60" : "hover:border-rose-300 hover:text-rose-50"}`}
                >
                  Delete Tool
                </button>
              </div>
              <div className="mt-2 text-[11px] text-slate-400">
                Active tool change is staged via runtime config and applied on controller restart for deterministic runtime behavior.
              </div>
              <div className="mt-1 text-[11px] text-slate-500">
                TCP/tool-tip uses <code>offset.*</code>. Mesh visual placement uses <code>mesh.position_mm</code> and <code>mesh.rotation_deg</code> relative to J6/flange.
              </div>
            </div>
            ) : null}
            {activeTab === "general" ? (
            <label className="mt-4 flex items-center gap-3 text-sm font-medium text-slate-200/90">
            <input
              type="checkbox"
              className="h-4 w-4 rounded border border-slate-600 bg-slate-900 text-cyan-400 focus:outline-none focus:ring-2 focus:ring-cyan-500/40"
              checked={showBoundingBox}
              onChange={(event) => onShowBoundingBoxChange(event.target.checked)}
            />
            Show arm bounding box
            </label>
            ) : null}
          </div>
          {activeTab === "kinematics" ? (
          <div className="flex flex-col gap-3">
            <div className="flex items-center justify-between rounded-xl border border-slate-700/60 bg-slate-950/40 px-3 py-2">
              <div>
                <div className="text-xs font-semibold uppercase tracking-[0.2em] text-cyan-200/80">
                  Kinematics Runtime
                </div>
                <div className="text-xs text-slate-400">
                  {kinematics
                    ? `rev ${kinematics.revision} · ${kinematics.profile.profile_id}`
                    : "No kinematics profile loaded"}
                </div>
              </div>
              <button
                type="button"
                onClick={onRefreshKinematics}
                disabled={isKinematicsBusy}
                className={`rounded border border-slate-600/70 px-2 py-1 text-xs ${isKinematicsBusy ? "opacity-60" : "hover:border-slate-400 hover:text-slate-100"}`}
              >
                Refresh
              </button>
            </div>
            {renderOffsetEditor("Base Runtime Offset", "base", baseOffsetDraft)}
            {renderOffsetEditor("Tool Runtime Offset", "tool", toolOffsetDraft)}
            <div className="rounded border border-slate-700/60 bg-slate-950/40 px-3 py-2 text-xs text-slate-300">
              Preview translational delta: <span className="font-semibold text-cyan-100">{previewDeltaMm.toFixed(2)} mm</span>
            </div>
            {kinematicsError ? (
              <div className="rounded border border-rose-500/40 bg-rose-500/10 px-3 py-2 text-xs text-rose-100">
                {kinematicsError}
              </div>
            ) : null}
            <div className="flex gap-2">
              <button
                type="button"
                onClick={onApplyOffsets}
                disabled={isKinematicsBusy || !kinematics}
                className={`rounded bg-cyan-600 px-3 py-1.5 text-xs font-semibold text-white ${isKinematicsBusy || !kinematics ? "opacity-60" : "hover:brightness-110"}`}
              >
                Apply Offsets
              </button>
              <button
                type="button"
                onClick={onResetOffsets}
                disabled={isKinematicsBusy || !kinematics}
                className={`rounded border border-slate-600/70 px-3 py-1.5 text-xs font-semibold text-slate-200 ${isKinematicsBusy || !kinematics ? "opacity-60" : "hover:border-slate-400 hover:text-slate-100"}`}
              >
                Reset Offsets
              </button>
            </div>
          </div>
          ) : null}
        </div>
      </div>
      </div>
    </div>
  );
}

function AlertsPanel({
  alerts,
  onDismiss,
}: {
  alerts: Alert[];
  onDismiss: (index: number) => void;
}) {
  if (!alerts || alerts.length === 0) {
    return null;
  }
  const colorFor = (lvl: Alert["level"]) =>
    lvl === "error"
      ? "border-rose-500/50 bg-rose-500/10 text-rose-100"
      : lvl === "warning"
      ? "border-amber-500/50 bg-amber-500/10 text-amber-100"
      : "border-cyan-500/40 bg-cyan-500/10 text-cyan-100";
  const iconFor = (lvl: Alert["level"]) =>
    lvl === "error" ? <Octagon size={16} /> : lvl === "warning" ? <Octagon size={16} /> : <Octagon size={16} />;
  return (
    <div className="pointer-events-auto flex max-w-sm flex-col gap-2">
      {alerts.slice(-4).map((a, idx) => (
        <div
          key={`${a.kind}:${a.ts ?? idx}:${idx}`}
          className={`flex items-start gap-2 rounded-lg border px-3 py-2 text-sm shadow-md ${colorFor(a.level)}`}
        >
          <div className="mt-0.5 shrink-0">{iconFor(a.level)}</div>
          <div className="flex-1">
            <div className="font-medium">{a.message}</div>
            {a.servo_ids && a.servo_ids.length > 0 && (
              <div className="text-xs opacity-75">Servos: {a.servo_ids.join(", ")}</div>
            )}
          </div>
          <button
            type="button"
            className="ml-2 rounded p-1 text-current/70 hover:text-current"
            aria-label="Dismiss alert"
            onClick={() => onDismiss(idx)}
          >
            <X size={14} />
          </button>
        </div>
      ))}
    </div>
  );
}

export default function App() {
  const [apiHost, setApiHost] = useState(() => resolveDefaultApiHost());
  const [visionHost, setVisionHost] = useState(() => resolveDefaultVisionHost());
  const [settings, setSettings] = useState<PersistedSettings>(() => loadPersistedSettings());
  const [isConnected, setIsConnected] = useState(false);
  const [latest, setLatest] = useState<TelemetryEvent | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [visionError, setVisionError] = useState<string | null>(null);
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const [settingsInitialTab, setSettingsInitialTab] = useState<SettingsDialogTab>("general");
  const [robotOptions, setRobotOptions] = useState<RobotPolicyOption[]>([]);
  const [runtimeConfigSnapshot, setRuntimeConfigSnapshot] = useState<RuntimeConfigSnapshot | null>(null);
  const [selectedRobotName, setSelectedRobotName] = useState("");
  const [toolLibrary, setToolLibrary] = useState<ToolDefinition[]>([]);
  const [selectedToolId, setSelectedToolId] = useState("identity");
  const [toolFilterRobotId, setToolFilterRobotId] = useState("all");
  const [toolFilterType, setToolFilterType] = useState("all");
  const [toolFilterQuery, setToolFilterQuery] = useState("");
  const [toolDraft, setToolDraft] = useState<ToolDefinition>(() => cloneToolDefinition(DEFAULT_TOOL_DRAFT));
  const [isRuntimeConfigBusy, setIsRuntimeConfigBusy] = useState(false);
  const [runtimeConfigError, setRuntimeConfigError] = useState<string | null>(null);
  const [isToolLibraryBusy, setIsToolLibraryBusy] = useState(false);
  const [toolLibraryError, setToolLibraryError] = useState<string | null>(null);
  const [isRestartingController, setIsRestartingController] = useState(false);
  const [kinematicsSnapshot, setKinematicsSnapshot] = useState<KinematicsProfileSnapshot | null>(null);
  const [baseOffsetDraft, setBaseOffsetDraft] = useState<RuntimeOffset>(() =>
    cloneRuntimeOffset(DEFAULT_RUNTIME_OFFSET),
  );
  const [toolOffsetDraft, setToolOffsetDraft] = useState<RuntimeOffset>(() =>
    cloneRuntimeOffset(DEFAULT_RUNTIME_OFFSET),
  );
  const [isKinematicsBusy, setIsKinematicsBusy] = useState(false);
  const [kinematicsError, setKinematicsError] = useState<string | null>(null);
  const [hasAttemptedAutoConnect, setHasAttemptedAutoConnect] = useState(false);
  const [isVisionActive, setIsVisionActive] = useState(false);
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
  const [alerts, setAlerts] = useState<Alert[]>([]);
  const [isResting, setIsResting] = useState(false);
  const [stepFile, setStepFile] = useState<File | null>(null);
  const [stepTransform, setStepTransform] = useState<StepTransform>(
    DEFAULT_STEP_TRANSFORM,
  );
  const [stepLoadStatus, setStepLoadStatus] = useState<StepLoadStatus>({
    state: "idle",
    message: "No STEP model loaded.",
  });
  const [topologyModel, setTopologyModel] = useState<TopologyModel | null>(null);
  const [isTopologyLoading, setIsTopologyLoading] = useState(false);
  const [weldSelectionMode, setWeldSelectionMode] = useState(false);
  const [weldDraft, setWeldDraft] = useState<WeldDraft | null>(null);
  const [isPlanningWeld, setIsPlanningWeld] = useState(false);
  const [weldEditableWaypoints, setWeldEditableWaypoints] = useState<Point3[]>([]);
  const [weldPreviewGhostJoints, setWeldPreviewGhostJoints] = useState<number[] | null>(null);
  const [weldPreviewCacheReady, setWeldPreviewCacheReady] = useState(false);
  const [showEndEffectorFrame, setShowEndEffectorFrame] = useState(false);
  const [weldProgramName, setWeldProgramName] = useState("weld_program");
  const [savedWeldPrograms, setSavedWeldPrograms] = useState<string[]>([]);
  const [selectedWeldProgram, setSelectedWeldProgram] = useState("");
  const [isWeldProgramListLoading, setIsWeldProgramListLoading] = useState(false);
  const [isSavingWeldProgram, setIsSavingWeldProgram] = useState(false);
  const [isLoadingWeldProgram, setIsLoadingWeldProgram] = useState(false);
  const [pendingWeldProgramRestore, setPendingWeldProgramRestore] = useState<{
    weldDraft: WeldDraft;
    editableWaypoints: Point3[];
    previewPlan: PreviewPlan | null;
  } | null>(null);
  const showBoundingBox = settings.showBoundingBox;
  const activePanel = settings.activePanel;
  const showProgramTree = settings.showProgramTree;
  const programTreeViewMode = settings.programTreeViewMode;
  const isRobotControlCollapsed = settings.collapseRobotControl;
  const expandedProgramTreeNodeIds = settings.expandedProgramTreeNodeIds;
  const selectedProgramNodeId = settings.selectedProgramNodeId;
  const visualizerRef = useRef<ArmVisualizerHandle | null>(null);
  const normalizedApiHost = useMemo(() => normaliseApiHost(apiHost), [apiHost]);
  const normalisedVisionHost = useMemo(
    () => normaliseVisionHost(visionHost),
    [visionHost],
  );

  const updateSettings = useCallback(
    (partial: Partial<PersistedSettings>) => {
      setSettings((prev) => {
        return { ...prev, ...partial };
      });
    },
    [],
  );

  const activeRobotId = runtimeConfigSnapshot?.active?.robot?.robot_id ?? null;
  const activeToolId = runtimeConfigSnapshot?.active?.tool?.active_tool_id ?? null;
  const desiredRobotId = useMemo(
    () => {
      const fromOptions = robotOptions.find((robot) => robot.name === selectedRobotName)?.robot_id;
      if (fromOptions) {
        return fromOptions;
      }
      const fallback = selectedRobotName.trim();
      return fallback.length > 0 ? fallback : null;
    },
    [robotOptions, selectedRobotName],
  );
  const visualizerRobotId = activeRobotId ?? desiredRobotId ?? DEFAULT_VISUALIZER_ROBOT_ID;
  const visualizerToolId = activeToolId ?? selectedToolId ?? null;
  const visualizerRuntimeTool = useMemo(() => {
    const runtimeTool = runtimeConfigSnapshot?.active?.tool;
    if (!runtimeTool || !runtimeTool.active_tool_id) {
      return null;
    }
    const offset = runtimeTool.offset;
    const mesh = runtimeTool.mesh;
    const normalizedMesh =
      mesh && typeof mesh.asset_path === "string" && mesh.asset_path.trim().length > 0
        ? {
            asset_path: mesh.asset_path,
            scale: Number.isFinite(mesh.scale) ? mesh.scale : 1,
            position_mm: {
              x: Number(mesh.position_mm?.x ?? 0),
              y: Number(mesh.position_mm?.y ?? 0),
              z: Number(mesh.position_mm?.z ?? 0),
            },
            rotation_deg: {
              x: Number(mesh.rotation_deg?.x ?? 0),
              y: Number(mesh.rotation_deg?.y ?? 0),
              z: Number(mesh.rotation_deg?.z ?? 0),
            },
          }
        : null;
    return {
      tool_id: runtimeTool.active_tool_id,
      display_name: runtimeTool.display_name || runtimeTool.active_tool_id,
      offset: {
        position_mm: {
          x: Number(offset?.position_mm?.x ?? 0),
          y: Number(offset?.position_mm?.y ?? 0),
          z: Number(offset?.position_mm?.z ?? 0),
        },
        rotation_deg: {
          x: Number(offset?.rotation_deg?.x ?? 0),
          y: Number(offset?.rotation_deg?.y ?? 0),
          z: Number(offset?.rotation_deg?.z ?? 0),
        },
      },
      mesh: normalizedMesh,
    };
  }, [runtimeConfigSnapshot]);
  const visualizerTool = useMemo(
    () =>
      visualizerRuntimeTool ??
      toolLibrary.find((tool) => tool.tool_id === visualizerToolId) ??
      null,
    [toolLibrary, visualizerToolId, visualizerRuntimeTool],
  );
  const availableToolTypes = useMemo(
    () =>
      Array.from(
        new Set(
          toolLibrary
            .map((tool) => tool.tool_type)
            .filter((token) => token.trim().length > 0),
        ),
      ).sort((a, b) => a.localeCompare(b)),
    [toolLibrary],
  );
  const filteredTools = useMemo(() => {
    return toolLibrary.filter((tool) => {
      if (toolFilterRobotId && toolFilterRobotId !== "all") {
        const compat = Array.isArray(tool.compatible_robot_ids)
          ? tool.compatible_robot_ids
          : [];
        if (compat.length > 0 && !compat.includes(toolFilterRobotId)) {
          return false;
        }
      }
      if (toolFilterType && toolFilterType !== "all" && tool.tool_type !== toolFilterType) {
        return false;
      }
      const query = toolFilterQuery.trim().toLowerCase();
      if (!query) {
        return true;
      }
      const haystack = [
        tool.tool_id,
        tool.display_name,
        tool.description ?? "",
        tool.tool_type,
        ...tool.keywords,
      ]
        .join(" ")
        .toLowerCase();
      return haystack.includes(query);
    });
  }, [toolFilterQuery, toolFilterRobotId, toolFilterType, toolLibrary]);

  const fetchKinematicsSnapshot = useCallback(async () => {
    const response = await fetch(`${normalizedApiHost}/kinematics/profile`);
    if (!response.ok) {
      const body = await response.text();
      throw new Error(body || `HTTP ${response.status}`);
    }
    const payload = (await response.json()) as KinematicsProfileSnapshot;
    setKinematicsSnapshot(payload);
    setBaseOffsetDraft(cloneRuntimeOffset(payload.offsets.base));
    setToolOffsetDraft(cloneRuntimeOffset(payload.offsets.tool));
    setKinematicsError(null);
    return payload;
  }, [normalizedApiHost]);

  const fetchToolLibrarySnapshot = useCallback(async () => {
    const response = await fetch(`${normalizedApiHost}/tools/library`);
    if (!response.ok) {
      const body = await response.text();
      throw new Error(body || `Tool library HTTP ${response.status}`);
    }
    const payload = (await response.json()) as { tools?: ToolDefinition[]; default_tool_id?: string };
    const tools = Array.isArray(payload.tools) ? payload.tools : [];
    setToolLibrary(tools);
    setToolLibraryError(null);
    setSelectedToolId((previous) => {
      const desired = previous.trim();
      if (desired && tools.some((tool) => tool.tool_id === desired)) {
        return desired;
      }
      if (typeof payload.default_tool_id === "string" && tools.some((tool) => tool.tool_id === payload.default_tool_id)) {
        return payload.default_tool_id;
      }
      return tools[0]?.tool_id ?? "identity";
    });
    return tools;
  }, [normalizedApiHost]);

  const fetchRuntimeConfigSnapshot = useCallback(async () => {
    const [robotsResponse, runtimeResponse] = await Promise.all([
      fetch(`${normalizedApiHost}/info/robots`),
      fetch(`${normalizedApiHost}/info/runtime-config`),
    ]);
    if (!robotsResponse.ok) {
      const body = await robotsResponse.text();
      throw new Error(body || `Robot policy HTTP ${robotsResponse.status}`);
    }
    if (!runtimeResponse.ok) {
      const body = await runtimeResponse.text();
      throw new Error(body || `Runtime config HTTP ${runtimeResponse.status}`);
    }
    const robotsPayload = (await robotsResponse.json()) as { robots?: RobotPolicyOption[] };
    const runtimePayload = (await runtimeResponse.json()) as RuntimeConfigSnapshot;
    const nextRobots = Array.isArray(robotsPayload.robots) ? robotsPayload.robots : [];
    setRobotOptions(nextRobots);
    setRuntimeConfigSnapshot(runtimePayload);
    const desiredRobot = runtimePayload?.desired?.robot;
    const desiredTool = runtimePayload?.desired?.active_tool_id;
    setSelectedRobotName((previous) => {
      if (typeof desiredRobot === "string" && desiredRobot.trim().length > 0) {
        return desiredRobot;
      }
      if (previous && previous.trim().length > 0) {
        return previous;
      }
      return nextRobots[0]?.name ?? "";
    });
    setSelectedToolId((previous) => {
      if (typeof desiredTool === "string" && desiredTool.trim().length > 0) {
        return desiredTool;
      }
      return previous || "identity";
    });
    setRuntimeConfigError(null);
    return runtimePayload;
  }, [normalizedApiHost]);

  useEffect(() => {
    if (toolLibrary.length === 0) {
      return;
    }
    const selected =
      toolLibrary.find((tool) => tool.tool_id === selectedToolId) ??
      toolLibrary.find((tool) => tool.tool_id === "identity") ??
      toolLibrary[0];
    if (!selected) {
      return;
    }
    setToolDraft((previous) => {
      if (
        previous.tool_id === selected.tool_id &&
        previous.display_name === selected.display_name &&
        previous.tool_type === selected.tool_type &&
        JSON.stringify(previous.offset) === JSON.stringify(selected.offset)
      ) {
        return previous;
      }
      return cloneToolDefinition(selected);
    });
  }, [selectedToolId, toolLibrary]);

  const handleOffsetDraftChange = useCallback(
    (
      target: "base" | "tool",
      group: "position_m" | "rotation_deg",
      axis: "x" | "y" | "z",
      value: number,
    ) => {
      const safeValue = Number.isFinite(value) ? value : 0;
      if (target === "base") {
        setBaseOffsetDraft((prev) => ({
          ...prev,
          [group]: { ...prev[group], [axis]: safeValue },
        }));
      } else {
        setToolOffsetDraft((prev) => ({
          ...prev,
          [group]: { ...prev[group], [axis]: safeValue },
        }));
      }
    },
    [],
  );

  const handleApplyRuntimeOffsets = useCallback(async () => {
    if (!kinematicsSnapshot) {
      return;
    }
    const confirmed = window.confirm(
      `Apply runtime offsets at revision ${kinematicsSnapshot.revision}?`,
    );
    if (!confirmed) {
      return;
    }
    setIsKinematicsBusy(true);
    try {
      const response = await fetch(`${normalizedApiHost}/kinematics/runtime-offsets`, {
        method: "PATCH",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          expected_revision: kinematicsSnapshot.revision,
          base: baseOffsetDraft,
          tool: toolOffsetDraft,
        }),
      });
      const payload = await response.json();
      if (!response.ok) {
        const message =
          typeof payload?.detail?.message === "string"
            ? payload.detail.message
            : response.statusText || "Failed to apply kinematics offsets.";
        throw new Error(message);
      }
      setKinematicsSnapshot(payload as KinematicsProfileSnapshot);
      setKinematicsError(null);
    } catch (err) {
      setKinematicsError((err as Error).message);
      await fetchKinematicsSnapshot().catch(() => undefined);
    } finally {
      setIsKinematicsBusy(false);
    }
  }, [
    baseOffsetDraft,
    fetchKinematicsSnapshot,
    kinematicsSnapshot,
    normalizedApiHost,
    toolOffsetDraft,
  ]);

  const handleResetRuntimeOffsets = useCallback(async () => {
    if (!kinematicsSnapshot) {
      return;
    }
    const confirmed = window.confirm("Reset runtime TCP/base offsets to zero?");
    if (!confirmed) {
      return;
    }
    setIsKinematicsBusy(true);
    try {
      const response = await fetch(`${normalizedApiHost}/kinematics/runtime-offsets/reset`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ expected_revision: kinematicsSnapshot.revision }),
      });
      const payload = await response.json();
      if (!response.ok) {
        const message =
          typeof payload?.detail?.message === "string"
            ? payload.detail.message
            : response.statusText || "Failed to reset kinematics offsets.";
        throw new Error(message);
      }
      const snapshot = payload as KinematicsProfileSnapshot;
      setKinematicsSnapshot(snapshot);
      setBaseOffsetDraft(cloneRuntimeOffset(snapshot.offsets.base));
      setToolOffsetDraft(cloneRuntimeOffset(snapshot.offsets.tool));
      setKinematicsError(null);
    } catch (err) {
      setKinematicsError((err as Error).message);
      await fetchKinematicsSnapshot().catch(() => undefined);
    } finally {
      setIsKinematicsBusy(false);
    }
  }, [fetchKinematicsSnapshot, kinematicsSnapshot, normalizedApiHost]);

  const handleNewToolDraft = useCallback(() => {
    setToolDraft(
      cloneToolDefinition({
        ...DEFAULT_TOOL_DRAFT,
        tool_id: `tool-${Date.now()}`,
        display_name: "New Tool",
      }),
    );
  }, []);

  const handleCreateTool = useCallback(async () => {
    setIsToolLibraryBusy(true);
    setToolLibraryError(null);
    try {
      const response = await fetch(`${normalizedApiHost}/tools/library`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          actor: "web-ui",
          tool: toolDraft,
        }),
      });
      const payload = await response.json();
      if (!response.ok) {
        const message =
          typeof payload?.detail === "string"
            ? payload.detail
            : response.statusText || "Failed to create tool.";
        throw new Error(message);
      }
      await fetchToolLibrarySnapshot();
      setSelectedToolId(toolDraft.tool_id);
      setToolLibraryError(null);
    } catch (err) {
      setToolLibraryError((err as Error).message);
    } finally {
      setIsToolLibraryBusy(false);
    }
  }, [fetchToolLibrarySnapshot, normalizedApiHost, toolDraft]);

  const handleUpdateTool = useCallback(async () => {
    if (!toolDraft.tool_id) {
      return;
    }
    setIsToolLibraryBusy(true);
    setToolLibraryError(null);
    try {
      const response = await fetch(
        `${normalizedApiHost}/tools/library/${encodeURIComponent(toolDraft.tool_id)}`,
        {
          method: "PATCH",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            actor: "web-ui",
            tool: toolDraft,
          }),
        },
      );
      const payload = await response.json();
      if (!response.ok) {
        const message =
          typeof payload?.detail === "string"
            ? payload.detail
            : response.statusText || "Failed to update tool.";
        throw new Error(message);
      }
      await fetchToolLibrarySnapshot();
      setSelectedToolId(toolDraft.tool_id);
      setToolLibraryError(null);
    } catch (err) {
      setToolLibraryError((err as Error).message);
    } finally {
      setIsToolLibraryBusy(false);
    }
  }, [fetchToolLibrarySnapshot, normalizedApiHost, toolDraft]);

  const handleDeleteTool = useCallback(async () => {
    if (!toolDraft.tool_id || toolDraft.tool_id === "identity") {
      return;
    }
    const confirmed = window.confirm(`Delete tool '${toolDraft.tool_id}'?`);
    if (!confirmed) {
      return;
    }
    setIsToolLibraryBusy(true);
    setToolLibraryError(null);
    try {
      const response = await fetch(
        `${normalizedApiHost}/tools/library/${encodeURIComponent(toolDraft.tool_id)}?actor=web-ui`,
        { method: "DELETE" },
      );
      const payload = await response.json();
      if (!response.ok) {
        const message =
          typeof payload?.detail === "string"
            ? payload.detail
            : response.statusText || "Failed to delete tool.";
        throw new Error(message);
      }
      await fetchToolLibrarySnapshot();
      setSelectedToolId("identity");
      setToolLibraryError(null);
    } catch (err) {
      setToolLibraryError((err as Error).message);
    } finally {
      setIsToolLibraryBusy(false);
    }
  }, [fetchToolLibrarySnapshot, normalizedApiHost, toolDraft.tool_id]);

  const handleApplyRuntimeConfig = useCallback(async () => {
    if (!selectedRobotName) {
      return;
    }
    setIsRuntimeConfigBusy(true);
    try {
      const response = await fetch(`${normalizedApiHost}/info/runtime-config`, {
        method: "PATCH",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          robot: selectedRobotName,
          active_tool_id: selectedToolId || null,
          actor: "web-ui",
        }),
      });
      const payload = await response.json();
      if (!response.ok) {
        const message =
          typeof payload?.detail === "string"
            ? payload.detail
            : response.statusText || "Failed to update runtime config.";
        throw new Error(message);
      }
      setRuntimeConfigSnapshot(payload as RuntimeConfigSnapshot);
      setRuntimeConfigError(null);
    } catch (err) {
      setRuntimeConfigError((err as Error).message);
      await fetchRuntimeConfigSnapshot().catch(() => undefined);
    } finally {
      setIsRuntimeConfigBusy(false);
    }
  }, [fetchRuntimeConfigSnapshot, normalizedApiHost, selectedRobotName, selectedToolId]);

  const handleRestartController = useCallback(async () => {
    if (!runtimeConfigSnapshot?.restart_required) {
      return;
    }
    const confirmed = window.confirm(
      "Request controller restart now? An external supervisor should restart it automatically.",
    );
    if (!confirmed) {
      return;
    }
    setIsRestartingController(true);
    setRuntimeConfigError(null);
    try {
      const response = await fetch(`${normalizedApiHost}/control/restart-controller`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ reason: "runtime-config-change" }),
      });
      const payload = await response.json();
      if (!response.ok) {
        const message =
          typeof payload?.detail === "string"
            ? payload.detail
            : response.statusText || "Failed to request controller restart.";
        throw new Error(message);
      }
      await new Promise((resolve) => window.setTimeout(resolve, 1250));
      await fetchRuntimeConfigSnapshot();
    } catch (err) {
      setRuntimeConfigError((err as Error).message);
    } finally {
      setIsRestartingController(false);
    }
  }, [fetchRuntimeConfigSnapshot, normalizedApiHost, runtimeConfigSnapshot?.restart_required]);
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
  const topologyOverlays = useMemo(
    () => toTopologyEdgeOverlay(topologyModel),
    [topologyModel],
  );
  const topologySceneOffset = useMemo(
    () => computeTopologyOffset(topologyOverlays),
    [topologyOverlays],
  );
  const stepTransformMatrix = useMemo(
    () => buildStepTransformMatrix(stepTransform),
    [stepTransform],
  );
  const topologyEdgeById = useMemo(
    () => new Map(topologyOverlays.map((edge) => [edge.id, edge])),
    [topologyOverlays],
  );
  const weldActive = Boolean(latest?.weld_active);
  const weldIndicatorPoint = useMemo(() => {
    if (!weldActive) {
      return null;
    }
    if (visualPathPoints.length > 0) {
      return visualPathPoints[visualPathPoints.length - 1];
    }
    if (visualWaypoints.length > 0) {
      return visualWaypoints[visualWaypoints.length - 1];
    }
    return null;
  }, [weldActive, visualPathPoints, visualWaypoints]);
  const activeWeldSegment = useMemo(
    () =>
      weldDraft
        ? weldDraft.segments.find(
            (segment) => segment.edgeId === weldDraft.activeSegmentEdgeId,
          ) ?? null
        : null,
    [weldDraft],
  );
  const selectedTopologyEdge = useMemo(
    () => (activeWeldSegment ? topologyEdgeById.get(activeWeldSegment.edgeId) ?? null : null),
    [activeWeldSegment, topologyEdgeById],
  );
  const selectedTopologyEdgeIds = useMemo(
    () => (weldDraft ? weldDraft.segments.map((segment) => segment.edgeId) : []),
    [weldDraft],
  );
  const weldStartPoint = useMemo(
    () =>
      selectedTopologyEdge && activeWeldSegment
        ? samplePointOnPolyline(selectedTopologyEdge.points, activeWeldSegment.startS)
        : null,
    [selectedTopologyEdge, activeWeldSegment],
  );
  const weldStopPoint = useMemo(
    () =>
      selectedTopologyEdge && activeWeldSegment
        ? samplePointOnPolyline(selectedTopologyEdge.points, activeWeldSegment.endS)
        : null,
    [selectedTopologyEdge, activeWeldSegment],
  );
  const weldSegmentPoints = useMemo(
    () =>
      selectedTopologyEdge && activeWeldSegment
        ? sampleSegmentOnPolyline(
            selectedTopologyEdge.points,
            activeWeldSegment.startS,
            activeWeldSegment.endS,
            28,
          )
        : [],
    [selectedTopologyEdge, activeWeldSegment],
  );
  const weldAnglePreview = useMemo(() => {
    if (activePanel !== "weld" || !weldDraft || !activeWeldSegment) {
      return null;
    }
    return {
      workAngleDeg: weldDraft.workAngleDeg,
      travelAngleDeg: weldDraft.travelAngleDeg,
      spinAngleDeg: weldDraft.spinAngleDeg,
    };
  }, [activePanel, weldDraft, activeWeldSegment]);
  const weldSelectedEdgeRows = useMemo(
    () =>
      weldDraft
        ? weldDraft.segments.map((segment) => {
            const edge = topologyEdgeById.get(segment.edgeId);
            const lengthM = edge ? polylineLength(edge.points) : 0;
            const mm = mmFromSegmentS(segment, lengthM);
            return {
              edgeId: segment.edgeId,
              startMm: mm.startMm,
              endMm: mm.endMm,
              lengthMm: lengthM * 1000,
              weldType: segment.weldType,
            };
          })
        : [],
    [weldDraft, topologyEdgeById],
  );
  const programTreeRoot = useMemo(
    () =>
      buildProgramTree({
        plan: previewPlan,
        weldSegments: weldDraft?.segments ?? [],
        weldType: weldDraft?.weldType,
        viewMode: programTreeViewMode,
      }),
    [previewPlan, weldDraft, programTreeViewMode],
  );
  const programNodeById = useMemo(
    () => indexProgramNodes(programTreeRoot),
    [programTreeRoot],
  );
  const selectedProgramNode = useMemo(
    () =>
      selectedProgramNodeId
        ? (programNodeById.get(selectedProgramNodeId) ?? null)
        : null,
    [selectedProgramNodeId, programNodeById],
  );
  const selectedProgramPathRange = selectedProgramNode?.focus?.pathRange ?? null;
  const selectedProgramWaypointIndices = selectedProgramNode?.focus?.waypointIndices ?? [];
  const selectedNodeFocusPoints = useMemo(() => {
    const node = selectedProgramNode;
    if (!node) {
      return [];
    }
    const out: Point3[] = [];
    const focus = node.focus;
    if (focus?.pathRange && visualPathPoints.length > 0) {
      const start = Math.max(0, Math.min(visualPathPoints.length - 1, focus.pathRange.start));
      const end = Math.max(start, Math.min(visualPathPoints.length - 1, focus.pathRange.end));
      out.push(...visualPathPoints.slice(start, end + 1));
    }
    if (Array.isArray(focus?.waypointIndices) && focus.waypointIndices.length > 0) {
      focus.waypointIndices.forEach((index) => {
        if (index >= 0 && index < visualWaypoints.length) {
          out.push(visualWaypoints[index]);
        }
      });
    }
    if (out.length === 0 && node.type === "program") {
      return visualPathPoints.length > 0 ? visualPathPoints : visualWaypoints;
    }
    return out;
  }, [selectedProgramNode, visualPathPoints, visualWaypoints]);
  const selectedProgramControlPointIndex = useMemo(() => {
    if (!selectedProgramNode?.id.startsWith("control_point_")) {
      return null;
    }
    const candidate = selectedProgramNode.focus?.waypointIndices?.[0];
    if (typeof candidate !== "number" || !Number.isInteger(candidate) || candidate < 0) {
      return null;
    }
    return candidate;
  }, [selectedProgramNode]);
  const treeEditableWaypoints = weldDraft ? weldEditableWaypoints : plannerPoints;
  const selectedProgramControlPoint = useMemo(() => {
    if (selectedProgramControlPointIndex === null) {
      return null;
    }
    if (
      selectedProgramControlPointIndex < 0 ||
      selectedProgramControlPointIndex >= treeEditableWaypoints.length
    ) {
      return null;
    }
    const point = treeEditableWaypoints[selectedProgramControlPointIndex];
    if (!point) {
      return null;
    }
    return {
      index: selectedProgramControlPointIndex,
      point,
    };
  }, [selectedProgramControlPointIndex, treeEditableWaypoints]);
  const canTreeWaypointValueEdit = !isPlanningWeld && !isRunningPreview;
  const minRemainingWaypoints = weldDraft ? 2 : 1;
  const canTreeWaypointAdd = canTreeWaypointValueEdit;
  const canTreeWaypointRemove = Boolean(
      selectedProgramControlPoint &&
      treeEditableWaypoints.length > minRemainingWaypoints &&
      canTreeWaypointValueEdit,
  );
  const canTreeWaypointApply =
    canTreeWaypointValueEdit &&
    (weldDraft ? weldEditableWaypoints.length > 1 : treeEditableWaypoints.length > 0);
  const sidebarItems = useMemo<SidebarItem[]>(
    () => [
      { id: "step", label: "STEP Import", icon: <FolderOpen size={17} />, shortcut: "1" },
      { id: "trajectory", label: "Trajectory", icon: <Route size={17} />, shortcut: "2" },
      { id: "tools", label: "Tool Library", icon: <Wrench size={17} /> },
      { id: "weld", label: "Weld", icon: <Flame size={17} />, shortcut: "3" },
      { id: "telemetry", label: "Live Charts", icon: <Camera size={17} />, shortcut: "4" },
    ],
    [],
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
  const panelSelectionOriginRef = useRef<"tree" | "weld" | null>(null);
  const trajectoryRefreshInFlight = useRef(false);
  const lastTelemetrySourceTimeRef = useRef<number | null>(null);
  const lastAcceptedJointsRef = useRef<number[] | null>(null);
  const lastAutoWeldAnglePreviewKeyRef = useRef<string>("");

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
    setTopologyModel(null);
    setIsTopologyLoading(false);
    setWeldSelectionMode(false);
    setWeldDraft(null);
    setIsPlanningWeld(false);
    setWeldEditableWaypoints([]);
    setWeldPreviewGhostJoints(null);
    setWeldPreviewCacheReady(false);
    setSavedWeldPrograms([]);
    setSelectedWeldProgram("");
    setIsWeldProgramListLoading(false);
    setIsSavingWeldProgram(false);
    setIsLoadingWeldProgram(false);
    setPendingWeldProgramRestore(null);
    lastTelemetrySourceTimeRef.current = null;
    lastAcceptedJointsRef.current = null;
  }, []);

  const handleMessage = useCallback((payload: string) => {
    let joints: number[] | undefined;
    let gripper: number | undefined;
    let servos: Record<string, ServoSample> | undefined;
    let parsedAlerts: Alert[] | undefined;
    let weldActiveValue: boolean | undefined;
    let weldTypeValue: string | undefined;
    let sourceTimeSec: number | undefined;

    try {
      const parsed = JSON.parse(payload);
      if (typeof parsed?.t === "number" && Number.isFinite(parsed.t)) {
        sourceTimeSec = parsed.t;
      } else if (typeof parsed?.t === "string") {
        const parsedTime = Number(parsed.t);
        if (Number.isFinite(parsedTime)) {
          sourceTimeSec = parsedTime;
        }
      }
      if (Array.isArray(parsed?.joints)) {
        joints = parsed.joints
          .map((value: unknown) =>
            typeof value === "number" ? value : Number(value),
          )
          .filter((value: number) => Number.isFinite(value));
      }
      if (typeof parsed?.gripper === "number") {
        gripper = parsed.gripper;
      }
      if (parsed && typeof parsed === "object" && parsed.servos && typeof parsed.servos === "object") {
        const out: Record<string, ServoSample> = {};
        for (const [k, v] of Object.entries(parsed.servos as Record<string, unknown>)) {
          if (v && typeof v === "object") {
            const s = v as Record<string, unknown>;
            const sample: ServoSample = {};
            if (typeof s.voltage_v === "number") sample.voltage_v = s.voltage_v;
            if (typeof s.temp_c === "number") sample.temp_c = s.temp_c;
            if (typeof s.current_a === "number") sample.current_a = s.current_a;
            if (typeof s.drive_duty_per_mille === "number") sample.drive_duty_per_mille = s.drive_duty_per_mille;
            if (typeof s.unloading_condition === "number") sample.unloading_condition = s.unloading_condition;
            if (typeof s.led_alarm_condition === "number") sample.led_alarm_condition = s.led_alarm_condition;
            if (typeof s.unloading_bits === "string") sample.unloading_bits = s.unloading_bits;
            if (typeof s.led_alarm_bits === "string") sample.led_alarm_bits = s.led_alarm_bits;
        const maybeEePose = maybeObj.ee_pose;
        if (maybeEePose && typeof maybeEePose === "object") {
          const eePoseObj = maybeEePose as Record<string, unknown>;
          const position = eePoseObj.position_m;
          const rotation = eePoseObj.rotation_matrix;
          if (position && typeof position === "object" && Array.isArray(rotation) && rotation.length === 3) {
            const posObj = position as Record<string, unknown>;
            const rotRows = rotation as unknown[];
            const parsedRows = rotRows.map((row) =>
              Array.isArray(row) && row.length === 3
                ? row.map((value) => Number(value))
                : null,
            );
            const validRows =
              parsedRows.length === 3 &&
              parsedRows.every(
                (row) => Array.isArray(row) && row.length === 3 && row.every((value) => Number.isFinite(value)),
              );
            const px = Number(posObj.x);
            const py = Number(posObj.y);
            const pz = Number(posObj.z);
            if (validRows && Number.isFinite(px) && Number.isFinite(py) && Number.isFinite(pz)) {
              eePoseValue = {
                position_m: { x: px, y: py, z: pz },
                rotation_matrix: parsedRows as [[number, number, number], [number, number, number], [number, number, number]],
              };
            }
          }
        }
            out[k] = sample;
          }
        }
        servos = out;
      }
      // Alerts (optional)
      if (parsed && typeof parsed === "object") {
        const maybeObj = parsed as Record<string, unknown>;
        if (Array.isArray(maybeObj.alerts)) {
          parsedAlerts = maybeObj.alerts as Alert[];
        }
        if (typeof maybeObj.weld_active === "boolean") {
          weldActiveValue = maybeObj.weld_active;
        }
        if (typeof maybeObj.weld_type === "string" && maybeObj.weld_type.trim()) {
          weldTypeValue = maybeObj.weld_type.trim();
        }
      }
    } catch {
      // fall back to raw payload only
    }

    const next: TelemetryEvent = {
      timestamp: Date.now(),
      raw: payload,
      joints,
      gripper,
      servos,
      alerts: parsedAlerts,
      weld_active: weldActiveValue,
      weld_type: weldTypeValue,
    };

    const candidateTimeSec = sourceTimeSec ?? next.timestamp / 1000;
    const lastTimeSec = lastTelemetrySourceTimeRef.current;
    if (lastTimeSec !== null && candidateTimeSec <= lastTimeSec) {
      // Drop out-of-order telemetry packets to prevent visual snap-backs.
      return;
    }
    if (Array.isArray(joints) && joints.length > 0 && lastAcceptedJointsRef.current) {
      const previous = lastAcceptedJointsRef.current;
      if (previous.length === joints.length) {
        let maxJump = 0;
        for (let i = 0; i < joints.length; i += 1) {
          const jump = Math.abs(joints[i] - previous[i]);
          if (jump > maxJump) {
            maxJump = jump;
          }
        }
        const dtSec =
          lastTimeSec !== null ? Math.max(0, candidateTimeSec - lastTimeSec) : Number.POSITIVE_INFINITY;
        // Reject single-frame spikes (commonly stale packets) that imply impossible
        // arm motion over one telemetry interval.
        if (dtSec <= 0.25 && maxJump > 0.8) {
          return;
        }
      }
    }

    lastTelemetrySourceTimeRef.current = candidateTimeSec;
    if (Array.isArray(joints) && joints.length > 0) {
      lastAcceptedJointsRef.current = joints.slice();
    }

    setLatest(next);
    // Merge alerts into state (keep last 20)
    if (Array.isArray(next.alerts) && next.alerts.length > 0) {
      setAlerts((prev) => {
        const merged = [...prev, ...next.alerts!];
        return merged.slice(-20);
      });
    }
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

  const handleStepFileChange = useCallback((file: File | null) => {
    setStepFile(file);
    setTopologyModel(null);
    setWeldSelectionMode(false);
    setWeldDraft(null);
    setWeldEditableWaypoints([]);
    setWeldPreviewGhostJoints(null);
    setWeldPreviewCacheReady(false);
    setPendingWeldProgramRestore(null);
    if (file) {
      setStepTransform(DEFAULT_STEP_TRANSFORM);
    }
  }, []);

  const handleClearStepFile = useCallback(() => {
    setStepFile(null);
    setStepLoadStatus({ state: "idle", message: "No STEP model loaded." });
    setTopologyModel(null);
    setWeldSelectionMode(false);
    setWeldDraft(null);
    setWeldEditableWaypoints([]);
    setWeldPreviewGhostJoints(null);
    setWeldPreviewCacheReady(false);
    setPendingWeldProgramRestore(null);
  }, []);

  const handleStepTransformChange = useCallback(
    (
      group: "position" | "rotationDeg",
      axis: "x" | "y" | "z",
      value: number,
    ) => {
      setStepTransform((current) => ({
        ...current,
        [group]: {
          ...current[group],
          [axis]: value,
        },
      }));
    },
    [],
  );

  const handleStepScaleChange = useCallback((value: number) => {
    setStepTransform((current) => ({
      ...current,
      scale: Number.isFinite(value) ? Math.max(0.01, value) : current.scale,
    }));
  }, []);

  const handleResetStepTransform = useCallback(() => {
    setStepTransform(DEFAULT_STEP_TRANSFORM);
  }, []);

  useEffect(() => {
    let cancelled = false;
    const run = async () => {
      if (!stepFile) {
        setTopologyModel(null);
        setWeldDraft(null);
        setWeldEditableWaypoints([]);
        setWeldPreviewGhostJoints(null);
        setWeldPreviewCacheReady(false);
        setPendingWeldProgramRestore(null);
        return;
      }
      setIsTopologyLoading(true);
      try {
        const encoded = await fileToBase64(stepFile);
        const response = await fetch(`${normalizedApiHost}/cad/topology/load-step`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            filename: stepFile.name,
            step_base64: encoded,
            sample_count: 64,
          }),
        });
        if (!response.ok) {
          const message = await response.text();
          throw new Error(message || `Topology request failed (${response.status})`);
        }
        const data = (await response.json()) as TopologyModel;
        if (cancelled) {
          return;
        }
        setTopologyModel(data);
        const validEdgeIds = new Set(
          Array.isArray(data.edges) ? data.edges.map((edge) => edge.id) : [],
        );
        if (pendingWeldProgramRestore) {
          const restored = pendingWeldProgramRestore.weldDraft;
          const fallbackEdgeId =
            Array.isArray(data.edges) && data.edges.length > 0 ? data.edges[0]?.id : "";
          const restoredSegments = restored.segments
            .filter((segment) => validEdgeIds.has(segment.edgeId))
            .map((segment) => ({
              edgeId: segment.edgeId,
              startS: clamp01(segment.startS),
              endS: clamp01(segment.endS),
              weldType:
                String(segment.weldType ?? restored.weldType ?? "fillet").trim() || "fillet",
            }));
          if (restoredSegments.length === 0 && fallbackEdgeId) {
            restoredSegments.push({
              edgeId: fallbackEdgeId,
              startS: 0,
              endS: 1,
              weldType: restored.weldType ?? "fillet",
            });
          }
          const restoredActive =
            restored.activeSegmentEdgeId &&
            restoredSegments.some((segment) => segment.edgeId === restored.activeSegmentEdgeId)
              ? restored.activeSegmentEdgeId
              : restoredSegments[0]?.edgeId ?? null;
          setWeldDraft(
            restoredSegments.length > 0
              ? {
                  ...restored,
                  modelId: data.model_id,
                  segments: restoredSegments,
                  activeSegmentEdgeId: restoredActive,
                }
              : null,
          );
          setWeldEditableWaypoints(pendingWeldProgramRestore.editableWaypoints);
          if (pendingWeldProgramRestore.previewPlan) {
            setPreviewPlan(pendingWeldProgramRestore.previewPlan);
            setPlannerPoints(pendingWeldProgramRestore.previewPlan.waypoints);
            setWeldPreviewGhostJoints(null);
            setWeldPreviewCacheReady(false);
          } else {
            // Clear any previous preview/path so loading a program without a saved plan
            // does not leave stale geometry visible in the scene.
            setPreviewPlan(null);
            setPlannerPoints([]);
            setWeldPreviewGhostJoints(null);
            setWeldPreviewCacheReady(false);
          }
          setPendingWeldProgramRestore(null);
          return;
        }
        // Fresh STEP imports start with no selected weld edges.
        setWeldDraft(null);
        setWeldPreviewGhostJoints(null);
      } catch (err) {
        if (cancelled) {
          return;
        }
        setTopologyModel(null);
        setWeldDraft(null);
        setWeldEditableWaypoints([]);
        setWeldPreviewGhostJoints(null);
        setWeldPreviewCacheReady(false);
        setPendingWeldProgramRestore(null);
        setError(`Failed to load CAD topology: ${(err as Error).message}`);
      } finally {
        if (!cancelled) {
          setIsTopologyLoading(false);
        }
      }
    };
    run();
    return () => {
      cancelled = true;
    };
  }, [stepFile, normalizedApiHost, pendingWeldProgramRestore]);

  const requestPlannerPreview = useCallback(
    async (points: Point3[]) => {
      if (points.length === 0) {
        setPreviewPlan(null);
        setPlannerPoints([]);
        setWeldPreviewGhostJoints(null);
        setWeldPreviewCacheReady(false);
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
        setWeldPreviewGhostJoints(null);
        setWeldPreviewCacheReady(false);
      } catch (err) {
        setError(`Failed to plan trajectory: ${(err as Error).message}`);
        setWeldPreviewGhostJoints(null);
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
      setWeldPreviewGhostJoints(null);
      return;
    }
    await requestPlannerPreview(nextPoints);
  }, [plannerPoints, isPlanLoading, requestPlannerPreview]);

  const handleClearPreview = useCallback(() => {
    setError(null);
    setPreviewPlan(null);
    setPlannerPoints([]);
    setWeldEditableWaypoints([]);
    setWeldPreviewGhostJoints(null);
    setWeldPreviewCacheReady(false);
    setIsPlanning(false);
  }, []);

  const requestWeldPreview = useCallback(
    async (draft: WeldDraft, waypointsOverride?: Point3[]): Promise<PreviewPlan | null> => {
      setWeldPreviewCacheReady(false);
      const primarySegment =
        draft.segments.find((segment) => segment.edgeId === draft.activeSegmentEdgeId) ??
        draft.segments[0];
      if (!primarySegment) {
        setError("Select at least one edge before planning a weld.");
        return null;
      }
      const sourceSections =
        Array.isArray(waypointsOverride) && waypointsOverride.length > 1
          ? [
              {
                kind: "weld" as const,
                weldType: primarySegment.weldType,
                edgeId: primarySegment.edgeId,
                points: waypointsOverride,
              },
            ]
          : buildWeldPreviewSections(draft, topologyEdgeById).map((section) => ({
              ...section,
              points: section.points.map((point) =>
                transformTopologyPointToScene(point, topologySceneOffset, stepTransformMatrix),
              ),
            }));
      const computedOverride = sourceSections.flatMap((section) => section.points);
      if (computedOverride.length < 2) {
        setError("Selected edge segments do not produce enough points to plan a weld.");
        return null;
      }
      setIsPlanningWeld(true);
      setError(null);
      try {
        const response = await fetch(`${normalizedApiHost}/trajectory/plan-weld`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            model_id: draft.modelId,
            edge_id: primarySegment.edgeId,
            start_s: clamp01(primarySegment.startS),
            end_s: clamp01(primarySegment.endS),
            weld_type: primarySegment.weldType || draft.weldType,
            weld_name: draft.weldName,
            sample_count: 48,
            waypoints_override: encodePointsForApi(computedOverride),
            sections: sourceSections.map((section) => ({
              kind: section.kind,
              weld_type: section.weldType,
              edge_id: section.edgeId,
              points: encodePointsForApi(section.points),
            })),
            options: {
              work_angle_deg: draft.workAngleDeg,
              travel_angle_deg: draft.travelAngleDeg,
              spin_angle_deg: draft.spinAngleDeg,
              transition_clearance_mm: draft.transitionClearanceMm,
              post_action: draft.postAction,
            },
          }),
        });
        if (!response.ok) {
          let message = "";
          try {
            const payload = (await response.json()) as { detail?: unknown };
            if (typeof payload?.detail === "string") {
              message = payload.detail;
            }
          } catch {
            message = await response.text();
          }
          throw new Error(message || `Weld planning failed (${response.status})`);
        }
        const data = await response.json();
        const { plan, waypoints } = previewFromPlannerPayload(data);
        setPreviewPlan(plan);
        setPlannerPoints(waypoints);
        setWeldEditableWaypoints(waypoints);
        const ghostPose =
          data &&
          Array.isArray((data as { weld_preview_joint_pose?: unknown }).weld_preview_joint_pose)
            ? ((data as { weld_preview_joint_pose: unknown[] }).weld_preview_joint_pose
                .map((value) => Number(value))
                .filter((value) => Number.isFinite(value)) as number[])
            : [];
        setWeldPreviewGhostJoints(ghostPose.length > 0 ? ghostPose : null);
        setWeldPreviewCacheReady(true);
        setIsPlanning(false);
        return plan;
      } catch (err) {
        const detail = (err as Error).message;
        const hint = detail.includes("Planning failed for one or more waypoints")
          ? " Try moving the STEP model closer to the robot workspace or adjusting start/end segment positions."
          : "";
        setError(`Failed to plan weld: ${detail}${hint}`);
        setWeldPreviewGhostJoints(null);
        return null;
      } finally {
        setIsPlanningWeld(false);
      }
    },
    [
      normalizedApiHost,
      topologyEdgeById,
      topologySceneOffset,
      stepTransformMatrix,
    ],
  );

  useEffect(() => {
    if (activePanel !== "weld" || !weldDraft || !activeWeldSegment || isPlanningWeld) {
      return;
    }
    const autoPreviewKey = JSON.stringify({
      modelId: weldDraft.modelId,
      edgeId: activeWeldSegment.edgeId,
      startS: clamp01(activeWeldSegment.startS),
      endS: clamp01(activeWeldSegment.endS),
      workAngleDeg: weldDraft.workAngleDeg,
      travelAngleDeg: weldDraft.travelAngleDeg,
      spinAngleDeg: weldDraft.spinAngleDeg,
    });
    if (lastAutoWeldAnglePreviewKeyRef.current === autoPreviewKey) {
      return;
    }
    const timer = window.setTimeout(() => {
      lastAutoWeldAnglePreviewKeyRef.current = autoPreviewKey;
      void requestWeldPreview(weldDraft);
    }, 180);
    return () => window.clearTimeout(timer);
  }, [activePanel, weldDraft, activeWeldSegment, isPlanningWeld, requestWeldPreview]);

  useEffect(() => {
    if (!weldDraft) {
      lastAutoWeldAnglePreviewKeyRef.current = "";
    }
  }, [weldDraft]);

  const handleRunPreview = useCallback(async () => {
    if (!previewPlan || isPlanLoading || isRunningPreview) {
      return;
    }
    setIsRunningPreview(true);
    setError(null);
    try {
      const trajectoryMeta = previewPlan.trajectory as unknown as Record<string, unknown>;
      const isWeldPreview =
        Boolean(trajectoryMeta) &&
        typeof trajectoryMeta.weld === "object" &&
        trajectoryMeta.weld !== null;
      let runName = previewPlan.name;
      let useCache = false;
      if (isWeldPreview) {
        useCache = true;
        if (!weldDraft) {
          throw new Error(
            "Weld preview is unavailable. Re-plan the weld preview before running.",
          );
        }
        // Always regenerate weld preview from the robot's current state so
        // return_to_start targets the actual pre-weld start pose for THIS run.
        const replanned = await requestWeldPreview(weldDraft);
        if (!replanned) {
          throw new Error("Failed to refresh weld preview before run.");
        }
        runName = replanned.name;
      }
      const response = await fetch(`${normalizedApiHost}/trajectory/run`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        // Weld previews need cached high-fidelity paths; endpoint-only re-planning
        // flattens the weld arc into sparse straight segments.
        body: JSON.stringify({ name: runName, use_cache: useCache }),
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
  }, [
    previewPlan,
    isPlanLoading,
    isRunningPreview,
    normalizedApiHost,
    weldDraft,
    requestWeldPreview,
  ]);

  const handleTopologyEdgeSelected = useCallback(
    (edgeId: string) => {
      panelSelectionOriginRef.current = "weld";
      setWeldDraft((current) => {
        const modelId = topologyModel?.model_id ?? current?.modelId ?? "";
        const weldType =
          current?.segments.find((segment) => segment.edgeId === current.activeSegmentEdgeId)
            ?.weldType ??
          current?.weldType ??
          "fillet";
        const existingSegments = current?.segments ?? [];
        const alreadySelected = existingSegments.some((segment) => segment.edgeId === edgeId);
        return {
          modelId,
          weldType,
          weldName: current?.weldName ?? `${weldType} weld`,
          segments: alreadySelected
            ? existingSegments
            : [...existingSegments, { edgeId, startS: 0, endS: 1, weldType }],
          activeSegmentEdgeId: edgeId,
          workAngleDeg: current?.workAngleDeg ?? 45,
          travelAngleDeg: current?.travelAngleDeg ?? 0,
          spinAngleDeg: current?.spinAngleDeg ?? 0,
          transitionClearanceMm: current?.transitionClearanceMm ?? 35,
          postAction: current?.postAction ?? "return_to_start",
        };
      });
    },
    [topologyModel],
  );

  const handlePlanWeldFromEdge = useCallback(async () => {
    if (!weldDraft) {
      return;
    }
    await requestWeldPreview(weldDraft);
  }, [requestWeldPreview, weldDraft]);

  const handleApplyWeldWaypointEdits = useCallback(async () => {
    if (!weldDraft || weldEditableWaypoints.length < 2) {
      return;
    }
    await requestWeldPreview(weldDraft, weldEditableWaypoints);
  }, [requestWeldPreview, weldDraft, weldEditableWaypoints]);

  const handleTreeWaypointChange = useCallback(
    (index: number, axis: "x" | "y" | "z", value: number) => {
      if (weldDraft) {
        setWeldEditableWaypoints((current) =>
          current.map((point, pointIndex) =>
            pointIndex === index
              ? {
                  ...point,
                  [axis]: Number.isFinite(value) ? value : point[axis],
                }
              : point,
          ),
        );
        setWeldPreviewCacheReady(false);
        return;
      }
      setPlannerPoints((current) =>
        current.map((point, pointIndex) =>
          pointIndex === index
            ? {
                ...point,
                [axis]: Number.isFinite(value) ? value : point[axis],
              }
            : point,
        ),
      );
    },
    [weldDraft],
  );

  const handleTreeAddWaypoint = useCallback(() => {
    if (weldDraft) {
      setWeldEditableWaypoints((current) => {
        if (current.length === 0) {
          return [{ x: 0, y: 0, z: 0 }, { x: 0.02, y: 0, z: 0 }];
        }
        const last = current[current.length - 1];
        return [...current, { ...last }];
      });
      setWeldPreviewCacheReady(false);
      return;
    }
    setPlannerPoints((current) => {
      if (current.length === 0) {
        return [{ x: 0, y: 0, z: 0 }];
      }
      const last = current[current.length - 1];
      return [...current, { ...last }];
    });
  }, [weldDraft]);

  const handleTreeRemoveWaypoint = useCallback((index: number) => {
    if (weldDraft) {
      setWeldEditableWaypoints((current) =>
        current.length <= 2
          ? current
          : current.filter((_, pointIndex) => pointIndex !== index),
      );
      setWeldPreviewCacheReady(false);
      return;
    }
    setPlannerPoints((current) =>
      current.length <= 1
        ? current
        : current.filter((_, pointIndex) => pointIndex !== index),
    );
  }, [weldDraft]);

  const handleApplyTreeWaypointEdits = useCallback(async () => {
    if (weldDraft) {
      await handleApplyWeldWaypointEdits();
      return;
    }
    if (treeEditableWaypoints.length === 0) {
      return;
    }
    await requestPlannerPreview(treeEditableWaypoints);
  }, [
    weldDraft,
    handleApplyWeldWaypointEdits,
    requestPlannerPreview,
    treeEditableWaypoints,
  ]);

  const refreshWeldProgramList = useCallback(async () => {
    setIsWeldProgramListLoading(true);
    try {
      const response = await fetch(`${normalizedApiHost}/weld-program/list`);
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || `Weld program list failed (${response.status})`);
      }
      const payload = (await response.json()) as { programs?: unknown };
      const names = Array.isArray(payload.programs)
        ? payload.programs
            .map((entry) => (typeof entry === "string" ? entry.trim() : ""))
            .filter((entry): entry is string => entry.length > 0)
        : [];
      setSavedWeldPrograms(names);
      setSelectedWeldProgram((current) =>
        current && names.includes(current) ? current : names[0] ?? "",
      );
    } catch (err) {
      setError(`Failed to load weld programs: ${(err as Error).message}`);
      setSavedWeldPrograms([]);
      setSelectedWeldProgram("");
    } finally {
      setIsWeldProgramListLoading(false);
    }
  }, [normalizedApiHost]);

  const handleSaveWeldProgram = useCallback(async () => {
    if (!weldDraft || !stepFile) {
      setError("Load a STEP model and select an edge before saving a weld program.");
      return;
    }
    const programName = weldProgramName.trim();
    if (!programName) {
      setError("Program name is required.");
      return;
    }
    setIsSavingWeldProgram(true);
    setError(null);
    try {
      const stepBase64 = await fileToBase64(stepFile);
      const response = await fetch(`${normalizedApiHost}/weld-program/save`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          name: programName,
          step: {
            filename: stepFile.name,
            step_base64: stepBase64,
            transform: stepTransform,
          },
          weld_draft: {
            modelId: weldDraft.modelId,
            weldType: weldDraft.weldType,
            weldName: weldDraft.weldName,
            workAngleDeg: weldDraft.workAngleDeg,
            travelAngleDeg: weldDraft.travelAngleDeg,
            spinAngleDeg: weldDraft.spinAngleDeg,
            transitionClearanceMm: weldDraft.transitionClearanceMm,
            postAction: weldDraft.postAction,
            segments: weldDraft.segments.map((segment) => ({
              edgeId: segment.edgeId,
              startS: clamp01(segment.startS),
              endS: clamp01(segment.endS),
              weldType: segment.weldType,
            })),
            activeSegmentEdgeId: weldDraft.activeSegmentEdgeId,
          },
          editable_waypoints: weldEditableWaypoints,
          planned_trajectory: previewPlan,
        }),
      });
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || `Save weld program failed (${response.status})`);
      }
      await refreshWeldProgramList();
      setSelectedWeldProgram(programName);
    } catch (err) {
      setError(`Failed to save weld program: ${(err as Error).message}`);
    } finally {
      setIsSavingWeldProgram(false);
    }
  }, [
    weldDraft,
    stepFile,
    weldProgramName,
    normalizedApiHost,
    stepTransform,
    weldEditableWaypoints,
    previewPlan,
    refreshWeldProgramList,
  ]);

  const handleLoadWeldProgram = useCallback(async () => {
    if (!selectedWeldProgram.trim() || isLoadingWeldProgram) {
      return;
    }
    setIsLoadingWeldProgram(true);
    setError(null);
    try {
      const response = await fetch(
        `${normalizedApiHost}/weld-program/${encodeURIComponent(selectedWeldProgram.trim())}`,
      );
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || `Load weld program failed (${response.status})`);
      }
      const program = (await response.json()) as WeldProgramRecord;
      if (!program?.step?.step_base64 || !program?.step?.filename) {
        throw new Error("Saved program is missing STEP payload.");
      }

      // Reset visible preview immediately for a successful program load so stale
      // path geometry from a previous plan is not shown while restore proceeds.
      setPreviewPlan(null);
      setPlannerPoints([]);
      setWeldPreviewCacheReady(false);

      const stepFileFromProgram = base64ToFile(
        program.step.step_base64,
        program.step.filename,
      );
      const restoredDraft = normalizeWeldDraftRecord(
        program.weld_draft,
        "",
      );
      if (!restoredDraft) {
        throw new Error("Saved program weld draft is invalid.");
      }
      setPendingWeldProgramRestore({
        weldDraft: restoredDraft,
        editableWaypoints: Array.isArray(program.editable_waypoints)
          ? program.editable_waypoints
          : [],
        previewPlan: program.planned_trajectory ?? null,
      });
      setStepFile(stepFileFromProgram);
      setStepTransform(program.step.transform ?? DEFAULT_STEP_TRANSFORM);
      setWeldProgramName(program.name);
    } catch (err) {
      setError(`Failed to load weld program: ${(err as Error).message}`);
    } finally {
      setIsLoadingWeldProgram(false);
    }
  }, [isLoadingWeldProgram, normalizedApiHost, selectedWeldProgram]);

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
      setWeldEditableWaypoints(plan.waypoints);
      setWeldPreviewGhostJoints(null);
      setWeldPreviewCacheReady(false);
      if (data?.trajectory?.weld && typeof data.trajectory.weld === "object") {
        const weld = data.trajectory.weld as Record<string, unknown>;
        const weldType =
          typeof weld.type === "string" && weld.type.trim() ? weld.type.trim() : "fillet";
        const modelId =
          typeof weld.model_id === "string" && weld.model_id.trim() ? weld.model_id.trim() : "";
        const edgeId =
          typeof weld.edge_id === "string" && weld.edge_id.trim() ? weld.edge_id.trim() : "";
        const options =
          weld.options && typeof weld.options === "object"
            ? (weld.options as Record<string, unknown>)
            : {};
        const workAngleDeg = Number(options.work_angle_deg ?? 45);
        const travelAngleDeg = Number(options.travel_angle_deg ?? 0);
        const spinAngleDeg = Number(
          options.spin_angle_deg ?? options.spinAngleDeg ?? 0,
        );
        const transitionClearanceMm = Number(options.transition_clearance_mm ?? 35);
        const postActionRaw =
          typeof options.post_action === "string" ? options.post_action.trim() : "return_to_start";
        const postAction: "none" | "return_to_start" | "lift" =
          postActionRaw === "none"
            ? "none"
            : postActionRaw === "lift"
              ? "lift"
              : "return_to_start";
        const segments =
          edgeId.length > 0
            ? [
                {
                  edgeId,
                  startS: typeof weld.start_s === "number" ? clamp01(weld.start_s) : 0,
                  endS: typeof weld.end_s === "number" ? clamp01(weld.end_s) : 1,
                  weldType,
                },
              ]
            : [];
        setWeldDraft(
          segments.length > 0
            ? {
                modelId,
                weldType,
                weldName:
                  typeof weld.name === "string" && weld.name.trim()
                    ? weld.name.trim()
                    : `${weldType} weld`,
                segments,
                activeSegmentEdgeId: segments[0].edgeId,
                workAngleDeg: Number.isFinite(workAngleDeg) ? workAngleDeg : 45,
                travelAngleDeg: Number.isFinite(travelAngleDeg) ? travelAngleDeg : 0,
                spinAngleDeg: Number.isFinite(spinAngleDeg) ? spinAngleDeg : 0,
                transitionClearanceMm:
                  Number.isFinite(transitionClearanceMm) && transitionClearanceMm > 0
                    ? transitionClearanceMm
                    : 35,
                postAction,
              }
            : null,
        );
      }
      setIsPlanning(false);
    } catch (err) {
      setError(`Failed to load trajectory: ${(err as Error).message}`);
      setWeldPreviewGhostJoints(null);
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

  const handleRest = useCallback(async () => {
    if (isResting) {
      return;
    }
    setError(null);
    setIsResting(true);
    try {
      const response = await fetch(`${normalizedApiHost}/control/rest`, {
        method: "POST",
      });
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || `Rest request failed (${response.status})`);
      }
    } catch (err) {
      setError(`Failed to move to rest: ${(err as Error).message ?? "Unknown error"}`);
    } finally {
      setIsResting(false);
    }
  }, [isResting, normalizedApiHost]);

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
    refreshWeldProgramList();
  }, [isConnected, refreshTrajectoryList, refreshWeldProgramList]);

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

  useEffect(() => {
    setIsRuntimeConfigBusy(true);
    setIsToolLibraryBusy(true);
    Promise.allSettled([fetchRuntimeConfigSnapshot(), fetchToolLibrarySnapshot()])
      .then((results) => {
        const runtimeResult = results[0];
        const toolResult = results[1];
        if (runtimeResult.status === "rejected") {
          setRuntimeConfigError((runtimeResult.reason as Error)?.message ?? "Failed to load runtime config.");
        }
        if (toolResult.status === "rejected") {
          setToolLibraryError((toolResult.reason as Error)?.message ?? "Failed to load tool library.");
        }
      })
      .finally(() => {
        setIsRuntimeConfigBusy(false);
        setIsToolLibraryBusy(false);
      });
  }, [fetchRuntimeConfigSnapshot, fetchToolLibrarySnapshot]);

  useEffect(() => {
    if (!isSettingsOpen) {
      return;
    }
    setIsKinematicsBusy(true);
    fetchKinematicsSnapshot()
      .catch((err) => {
        setKinematicsError((err as Error).message);
      })
      .finally(() => {
        setIsKinematicsBusy(false);
      });
  }, [fetchKinematicsSnapshot, isSettingsOpen]);

  useEffect(() => {
    if (!isSettingsOpen) {
      return;
    }
    setIsRuntimeConfigBusy(true);
    fetchRuntimeConfigSnapshot()
      .catch((err) => {
        setRuntimeConfigError((err as Error).message);
      })
      .finally(() => {
        setIsRuntimeConfigBusy(false);
      });
  }, [fetchRuntimeConfigSnapshot, isSettingsOpen]);

  useEffect(() => {
    if (!isSettingsOpen) {
      return;
    }
    setIsToolLibraryBusy(true);
    fetchToolLibrarySnapshot()
      .catch((err) => {
        setToolLibraryError((err as Error).message);
      })
      .finally(() => {
        setIsToolLibraryBusy(false);
      });
  }, [fetchToolLibrarySnapshot, isSettingsOpen]);

  // Lightweight API health probe to surface clearer 5xx reasons
  useEffect(() => {
    let cancelled = false;
    const tick = async () => {
      try {
        const r = await fetch(`${normalizedApiHost}/health`, { method: "GET" });
        if (!r.ok) {
          const txt = await r.text();
          if (!cancelled) {
            setError(`API /health ${r.status}: ${txt || r.statusText}`);
          }
        }
      } catch (e) {
        if (!cancelled) {
          setError("API unreachable. Check that gradient-api is running.");
        }
      }
    };
    const id = window.setInterval(tick, 5000);
    tick(); // immediate first probe
    return () => {
      cancelled = true;
      window.clearInterval(id);
    };
  }, [normalizedApiHost]);
  useEffect(() => {
    persistSettings(settings);
  }, [settings]);

  useEffect(() => {
    if (!selectedProgramNodeId) {
      return;
    }
    if (programNodeById.has(selectedProgramNodeId)) {
      return;
    }
    updateSettings({ selectedProgramNodeId: null });
  }, [selectedProgramNodeId, programNodeById, updateSettings]);

  useEffect(() => {
    if (panelSelectionOriginRef.current !== "tree") {
      return;
    }
    if (!selectedProgramNodeId) {
      panelSelectionOriginRef.current = null;
      return;
    }
    const segmentEdgeId = selectedProgramNode?.focus?.weldSegmentEdgeId;
    if (!segmentEdgeId) {
      panelSelectionOriginRef.current = null;
      return;
    }
    setWeldDraft((current) => {
      if (!current || current.activeSegmentEdgeId === segmentEdgeId) {
        return current;
      }
      if (!current.segments.some((segment) => segment.edgeId === segmentEdgeId)) {
        return current;
      }
      return {
        ...current,
        activeSegmentEdgeId: segmentEdgeId,
      };
    });
    panelSelectionOriginRef.current = null;
  }, [selectedProgramNodeId, selectedProgramNode]);

  useEffect(() => {
    if (panelSelectionOriginRef.current === "tree") {
      return;
    }
    const activeEdgeId = weldDraft?.activeSegmentEdgeId ?? null;
    if (!activeEdgeId) {
      return;
    }
    const selectedNodeType = selectedProgramNode?.type ?? null;
    if (selectedNodeType && selectedNodeType !== "weldSegment" && activePanel !== "weld") {
      return;
    }
    const matchingNodeId = findWeldProgramNodeIdByEdge(programNodeById, activeEdgeId);
    if (!matchingNodeId || matchingNodeId === selectedProgramNodeId) {
      return;
    }
    updateSettings({ selectedProgramNodeId: matchingNodeId });
  }, [
    weldDraft?.activeSegmentEdgeId,
    selectedProgramNode,
    selectedProgramNodeId,
    programNodeById,
    activePanel,
    updateSettings,
  ]);

  useEffect(() => {
    if (!selectedProgramNodeId || selectedNodeFocusPoints.length === 0) {
      return;
    }
    visualizerRef.current?.focusOnPoints(selectedNodeFocusPoints);
  }, [selectedProgramNodeId, selectedNodeFocusPoints]);

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      const target = event.target as HTMLElement | null;
      const tag = target?.tagName?.toLowerCase() ?? "";
      const editable =
        tag === "input" ||
        tag === "textarea" ||
        tag === "select" ||
        Boolean(target?.isContentEditable);
      if (editable || event.altKey || event.ctrlKey || event.metaKey) {
        return;
      }
      const key = event.key.toLowerCase();
      const panelByKey: Record<string, SidebarPanelId> = {
        "1": "step",
        "2": "trajectory",
        "3": "weld",
        "4": "telemetry",
        "5": "tools",
      };
      if (panelByKey[key]) {
        const nextPanel = panelByKey[key];
        updateSettings({ activePanel: activePanel === nextPanel ? null : nextPanel });
        event.preventDefault();
        return;
      }
      if (key === "t") {
        updateSettings({ showProgramTree: !showProgramTree });
        event.preventDefault();
      }
    };
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [activePanel, showProgramTree, updateSettings]);

  const handleSelectSidebarPanel = useCallback(
    (panelId: string) => {
      const panel = panelId as SidebarPanelId;
      updateSettings({ activePanel: activePanel === panel ? null : panel });
    },
    [activePanel, updateSettings],
  );
  const handleToggleProgramTreeNode = useCallback(
    (nodeId: string) => {
      const exists = expandedProgramTreeNodeIds.includes(nodeId);
      updateSettings({
        expandedProgramTreeNodeIds: exists
          ? expandedProgramTreeNodeIds.filter((id) => id !== nodeId)
          : [...expandedProgramTreeNodeIds, nodeId],
      });
    },
    [expandedProgramTreeNodeIds, updateSettings],
  );
  const handleSelectProgramTreeNode = useCallback(
    (nodeId: string) => {
      const node = programNodeById.get(nodeId);
      const nextPanel = node?.focus?.openPanel;
      panelSelectionOriginRef.current = "tree";
      updateSettings({
        selectedProgramNodeId: nodeId,
        activePanel: nextPanel ?? activePanel,
      });
    },
    [programNodeById, updateSettings, activePanel],
  );
  const handleChangeProgramTreeViewMode = useCallback(
    (value: ProgramTreeViewMode) => {
      if (value === programTreeViewMode) {
        return;
      }
      updateSettings({
        programTreeViewMode: value,
        expandedProgramTreeNodeIds:
          value === "chronological"
            ? ["program_root", "setup_primary", "op_chronological", "op_weld"]
            : ["program_root", "setup_primary", "op_motion", "op_weld"],
      });
    },
    [programTreeViewMode, updateSettings],
  );

  const streamingLabel = useMemo(
    () => (isConnected ? "Streaming" : "Disconnected"),
    [isConnected],
  );
  const headerAlert = [error, visionError].filter(Boolean).join(" • ");
  const hasHeaderAlert = headerAlert.length > 0;
  const alertTone = error ? "rose" : "amber";
  const activeDrawerWidthClass = activePanel === "telemetry"
    ? "w-[30rem] max-w-[calc(100vw-7rem)]"
    : activePanel === "tools"
      ? "w-[23rem] max-w-[calc(100vw-7rem)]"
      : "w-[20rem] max-w-[calc(100vw-7rem)]";
  const activeDrawerHeightMode: "content" | "full" = "content";
  const activeDrawerHeader = activePanel === "step"
    ? (
        <span className="text-xs font-semibold uppercase tracking-[0.25em] text-cyan-200/80">
          STEP Import
        </span>
      )
    : activePanel === "trajectory"
      ? (
          <span className="text-xs font-semibold uppercase tracking-[0.25em] text-cyan-200/80">
            Trajectory
          </span>
        )
      : activePanel === "tools"
        ? (
            <span className="text-xs font-semibold uppercase tracking-[0.25em] text-cyan-200/80">
              Tool Library
            </span>
          )
      : activePanel === "weld"
        ? (
            <div className="flex min-w-0 flex-wrap items-center gap-2">
              <span className="text-xs font-semibold uppercase tracking-[0.25em] text-orange-200/80">
                Weld Planning
              </span>
              {weldActive ? (
                <span className="inline-flex items-center gap-1 rounded-full border border-orange-400/40 bg-orange-500/20 px-2 py-0.5 text-[11px] font-semibold text-orange-100">
                  <Flame size={12} /> Weld ON
                </span>
              ) : null}
            </div>
          )
        : activePanel === "telemetry"
          ? (
              <span className="text-xs font-semibold uppercase tracking-[0.25em] text-cyan-200/80">
                Live Charts
              </span>
            )
          : null;
  const activeDrawerContent = activePanel === "step"
    ? (
        <StepImportPanel
          stepFileName={stepFile?.name ?? null}
          stepStatus={stepLoadStatus}
          transform={stepTransform}
          onFileChange={handleStepFileChange}
          onTransformChange={handleStepTransformChange}
          onScaleChange={handleStepScaleChange}
          onResetTransform={handleResetStepTransform}
          onClearFile={handleClearStepFile}
        />
      )
    : activePanel === "trajectory"
      ? (
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
        )
      : activePanel === "tools"
        ? (
            <ToolLibraryPanel
              robots={robotOptions}
              tools={toolLibrary}
              filteredTools={filteredTools}
              availableToolTypes={availableToolTypes}
              selectedToolId={selectedToolId}
              toolFilterRobotId={toolFilterRobotId}
              toolFilterType={toolFilterType}
              toolFilterQuery={toolFilterQuery}
              activeRuntimeConfig={runtimeConfigSnapshot?.active ?? null}
              restartRequired={Boolean(runtimeConfigSnapshot?.restart_required)}
              runtimeConfigError={runtimeConfigError}
              isRuntimeConfigBusy={isRuntimeConfigBusy}
              isRestartingController={isRestartingController}
              isToolLibraryBusy={isToolLibraryBusy}
              toolLibraryError={toolLibraryError}
              onSelectedToolIdChange={setSelectedToolId}
              onToolFilterRobotIdChange={setToolFilterRobotId}
              onToolFilterTypeChange={setToolFilterType}
              onToolFilterQueryChange={setToolFilterQuery}
              onRefreshTools={() => {
                setIsToolLibraryBusy(true);
                fetchToolLibrarySnapshot()
                  .catch((err) => setToolLibraryError((err as Error).message))
                  .finally(() => setIsToolLibraryBusy(false));
              }}
              onRefreshRuntimeConfig={() => {
                setIsRuntimeConfigBusy(true);
                fetchRuntimeConfigSnapshot()
                  .catch((err) => setRuntimeConfigError((err as Error).message))
                  .finally(() => setIsRuntimeConfigBusy(false));
              }}
              onApplyRuntimeConfig={handleApplyRuntimeConfig}
              onRestartController={handleRestartController}
              onOpenToolSettings={() => {
                setSettingsInitialTab("tools");
                setIsSettingsOpen(true);
              }}
            />
          )
      : activePanel === "weld"
        ? (
            <WeldPanel
              isConnected={isConnected}
              isTopologyLoading={isTopologyLoading}
              topologyModelId={topologyModel?.model_id ?? null}
              topologyEdgeCount={topologyModel?.edges?.length ?? 0}
              activeEdgeId={activeWeldSegment?.edgeId ?? null}
              selectedEdges={weldSelectedEdgeRows}
              weldSelectionMode={weldSelectionMode}
              draft={weldDraft}
              isPlanningWeld={isPlanningWeld}
              isRunning={isRunningPreview}
              canRunPreview={Boolean(previewPlan?.name)}
              weldActive={weldActive}
              planningWarnings={previewPlan?.planningWarnings ?? []}
              onToggleSelection={() => setWeldSelectionMode((value) => !value)}
              onSelectEdge={(edgeId) => {
                panelSelectionOriginRef.current = "weld";
                setWeldDraft((current) =>
                  current
                    ? {
                        ...current,
                        activeSegmentEdgeId: edgeId,
                      }
                    : current,
                );
              }}
              onRemoveEdge={(edgeId) =>
                setWeldDraft((current) => {
                  if (!current) {
                    return current;
                  }
                  const nextSegments = current.segments.filter(
                    (segment) => segment.edgeId !== edgeId,
                  );
                  if (nextSegments.length === 0) {
                    return null;
                  }
                  const nextActive = nextSegments.some(
                    (segment) => segment.edgeId === current.activeSegmentEdgeId,
                  )
                    ? current.activeSegmentEdgeId
                    : (nextSegments[0]?.edgeId ?? null);
                  if (!nextActive) {
                    return null;
                  }
                  return {
                    ...current,
                    segments: nextSegments,
                    activeSegmentEdgeId: nextActive,
                  };
                })
              }
              onPlanFromEdge={handlePlanWeldFromEdge}
              onRun={handleRunPreview}
              onSetWeldType={(value) =>
                setWeldDraft((current) =>
                  current
                    ? {
                        ...current,
                        weldType: value,
                        segments: current.segments.map((segment) =>
                          segment.edgeId === current.activeSegmentEdgeId
                            ? { ...segment, weldType: value }
                            : segment,
                        ),
                        weldName:
                          current.weldName && current.weldName.trim().length > 0
                            ? current.weldName
                            : `${value} weld`,
                      }
                    : current,
                )
              }
              onSetWeldName={(value) =>
                setWeldDraft((current) =>
                  current
                    ? {
                        ...current,
                        weldName: value,
                      }
                    : current,
                )
              }
              onSetWorkAngleDeg={(value) =>
                setWeldDraft((current) =>
                  current
                    ? {
                        ...current,
                        workAngleDeg: Number.isFinite(value) ? value : current.workAngleDeg,
                      }
                    : current,
                )
              }
              onSetTravelAngleDeg={(value) =>
                setWeldDraft((current) =>
                  current
                    ? {
                        ...current,
                        travelAngleDeg: Number.isFinite(value) ? value : current.travelAngleDeg,
                      }
                    : current,
                )
              }
              onSetSpinAngleDeg={(value) =>
                setWeldDraft((current) =>
                  current
                    ? {
                        ...current,
                        spinAngleDeg: Number.isFinite(value)
                          ? value
                          : current.spinAngleDeg,
                      }
                    : current,
                )
              }
              showEndEffectorFrame={showEndEffectorFrame}
              onShowEndEffectorFrameChange={setShowEndEffectorFrame}
              onSetTransitionClearanceMm={(value) =>
                setWeldDraft((current) =>
                  current
                    ? {
                        ...current,
                        transitionClearanceMm:
                          Number.isFinite(value) && value > 0
                            ? value
                            : current.transitionClearanceMm,
                      }
                    : current,
                )
              }
              onSetPostAction={(value) =>
                setWeldDraft((current) =>
                  current
                    ? {
                        ...current,
                        postAction: value,
                      }
                    : current,
                )
              }
              onSetStartS={(value) =>
                setWeldDraft((current) =>
                  current
                    ? {
                        ...current,
                        segments: current.segments.map((segment) =>
                          segment.edgeId === current.activeSegmentEdgeId
                            ? { ...segment, startS: clamp01(value) }
                            : segment,
                        ),
                      }
                    : current,
                )
              }
              onSetEndS={(value) =>
                setWeldDraft((current) =>
                  current
                    ? {
                        ...current,
                        segments: current.segments.map((segment) =>
                          segment.edgeId === current.activeSegmentEdgeId
                            ? { ...segment, endS: clamp01(value) }
                            : segment,
                        ),
                      }
                    : current,
                )
              }
              weldProgramName={weldProgramName}
              onWeldProgramNameChange={setWeldProgramName}
              onSaveProgram={handleSaveWeldProgram}
              isSavingProgram={isSavingWeldProgram}
              savedPrograms={savedWeldPrograms}
              selectedProgram={selectedWeldProgram}
              onSelectedProgramChange={setSelectedWeldProgram}
              onLoadProgram={handleLoadWeldProgram}
              isLoadingProgram={isLoadingWeldProgram}
              isProgramListLoading={isWeldProgramListLoading}
              onRefreshPrograms={refreshWeldProgramList}
            />
          )
        : activePanel === "telemetry"
          ? <TelemetryCharts latest={latest} />
        : null;

  return (
    <div className="flex min-h-screen flex-col bg-gradient-to-b from-slate-900/80 via-slate-950 to-black text-slate-100">
      <header className="relative flex flex-col gap-2 border-b border-slate-800/40 bg-slate-950/60 px-6 py-2.5 shadow-inner shadow-slate-900/40 backdrop-blur">
        <div className="flex flex-col gap-2 sm:flex-row sm:items-center sm:justify-between">
          <div className="flex flex-col shrink-0">
            <div className="flex justify-end text-lg font-semibold tracking-tight text-cyan-300 sm:text-xl w-full leading-tight">
              Control Center
            </div>
            <div className="flex justify-end text-xs tracking-tight text-cyan-300/90 sm:text-sm w-full leading-tight">
              Gradient Robotics
            </div>
          </div>
          <div className="flex min-w-0 flex-1 items-center sm:px-3">
            <div
              className={`flex h-7 w-full items-center overflow-hidden rounded-lg border px-2 text-[11px] sm:text-xs ${
                hasHeaderAlert
                  ? alertTone === "rose"
                    ? "border-rose-500/40 bg-rose-500/10 text-rose-200"
                    : "border-amber-500/40 bg-amber-500/10 text-amber-200"
                  : "border-transparent bg-transparent text-transparent"
              }`}
              role={hasHeaderAlert ? "alert" : undefined}
              aria-live={hasHeaderAlert ? "polite" : undefined}
              aria-hidden={!hasHeaderAlert}
              title={hasHeaderAlert ? headerAlert : undefined}
            >
              {hasHeaderAlert && <span className="w-full truncate">{headerAlert}</span>}
            </div>
          </div>
          <div className="flex items-center gap-2">
            <span
              className={`inline-flex items-center gap-1.5 rounded-full px-2.5 py-0.5 text-[11px] font-medium ${
                isConnected
                  ? "bg-emerald-500/15 text-emerald-200 ring-1 ring-inset ring-emerald-400/30"
                  : "bg-rose-500/10 text-rose-200 ring-1 ring-inset ring-rose-400/20"
              }`}
            >
              <span
                className={`h-1.5 w-1.5 rounded-full ${
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
                <Unplug size={16} strokeWidth={2} />
              ) : (
                <Plug size={16} strokeWidth={2} />
              )}
            </button>
            <button
              type="button"
              onClick={handleHome}
              disabled={!isConnected || isHoming}
              className={`rounded-full border border-slate-600/60 bg-slate-900/60 p-1.5 text-slate-300 transition hover:border-slate-400 hover:text-slate-100 ${
                (!isConnected || isHoming) ? "opacity-60 cursor-not-allowed" : ""
              }`}
              aria-label="Move arm to home position"
            >
              <Home size={16} strokeWidth={2} />
            </button>
            <button
              type="button"
              onClick={handleRest}
              disabled={!isConnected || isResting}
              className={`rounded-full border border-slate-600/60 bg-slate-900/60 p-1.5 text-slate-300 transition hover:border-slate-400 hover:text-slate-100 ${
                (!isConnected || isResting) ? "opacity-60 cursor-not-allowed" : ""
              }`}
              aria-label="Move arm to rest pose"
            >
              <Moon size={16} strokeWidth={2} />
            </button>
            <button
              type="button"
              onClick={issueStop}
              disabled={isStopping}
              className={`rounded-full bg-gradient-to-r from-rose-600 to-rose-700 p-1.5 text-white shadow-md shadow-rose-500/40 transition ${
                isStopping ? "cursor-wait opacity-60" : "hover:brightness-110"
              }`}
              aria-label="Send emergency stop"
            >
              <Octagon size={16} strokeWidth={2.5} />
            </button>
            <button
              type="button"
              onClick={toggleVision}
              className={cameraButtonClasses}
              aria-label={isVisionActive ? "Disable camera overlay" : "Enable camera overlay"}
            >
              {isVisionActive ? (
                <Camera size={16} strokeWidth={2} />
              ) : (
                <CameraOff size={16} strokeWidth={2} />
              )}
            </button>
            <button
              type="button"
              onClick={handleResetView}
              className="rounded-full border border-slate-600/60 bg-slate-900/60 p-1.5 text-slate-300 transition hover:border-slate-400 hover:text-slate-100"
              aria-label="Reset arm view"
            >
              <RefreshCcw size={16} strokeWidth={2} />
            </button>
            <button
              type="button"
              onClick={() => updateSettings({ showProgramTree: !showProgramTree })}
              className={`rounded-full border bg-slate-900/60 p-1.5 transition ${
                showProgramTree
                  ? "border-cyan-400/60 text-cyan-100"
                  : "border-slate-600/60 text-slate-300 hover:border-slate-400 hover:text-slate-100"
              }`}
              aria-label="Toggle program tree"
              title="Toggle program tree (T)"
            >
              <Route size={16} strokeWidth={2} />
            </button>
            <button
              type="button"
              onClick={() => {
                setSettingsInitialTab("general");
                setIsSettingsOpen(true);
              }}
              className="rounded-full border border-slate-600/60 bg-slate-900/60 p-1.5 text-slate-300 transition hover:border-slate-400 hover:text-slate-100"
              aria-label="Open settings"
            >
              <Settings size={16} strokeWidth={2} />
            </button>
          </div>
        </div>
      </header>
      <main className="relative flex-1 overflow-hidden">
        <ArmVisualizer
          robotId={visualizerRobotId}
          activeTool={visualizerTool}
          ref={visualizerRef}
          joints={latest?.joints}
          showBoundingBox={showBoundingBox}
          selectionMode={isPlanning && !isPlanLoading}
          onPointSelected={handlePointSelected}
          weldSelectionMode={weldSelectionMode}
          topologyEdges={topologyOverlays}
          selectedTopologyEdgeId={activeWeldSegment?.edgeId ?? null}
          selectedTopologyEdgeIds={selectedTopologyEdgeIds}
          onTopologyEdgeSelected={handleTopologyEdgeSelected}
          weldActive={weldActive}
          weldIndicatorPoint={weldIndicatorPoint}
          weldStartPoint={weldStartPoint}
          weldStopPoint={weldStopPoint}
          weldSegmentPoints={weldSegmentPoints}
          showEndEffectorFrame={showEndEffectorFrame}
          weldAnglePreview={weldAnglePreview}
          weldGhostJoints={weldPreviewGhostJoints}
          pathPoints={visualPathPoints}
          waypoints={visualWaypoints}
          highlightPathRange={selectedProgramPathRange}
          highlightWaypointIndices={selectedProgramWaypointIndices}
          stepFile={stepFile}
          stepTransform={stepTransform}
          onStepStatusChange={setStepLoadStatus}
        />
        <SidebarRail
          items={sidebarItems}
          activeItemId={activePanel}
          onSelect={handleSelectSidebarPanel}
        />
        {activePanel && activeDrawerContent ? (
          <SidebarDrawer
            onClose={() => updateSettings({ activePanel: null })}
            headerContent={activeDrawerHeader}
            widthClassName={activeDrawerWidthClass}
            heightMode={activeDrawerHeightMode}
          >
            {activeDrawerContent}
          </SidebarDrawer>
        ) : null}
        {showProgramTree ? (
          <ProgramFeatureTree
            root={programTreeRoot}
            expandedNodeIds={expandedProgramTreeNodeIds}
            selectedNodeId={selectedProgramNodeId}
            viewMode={programTreeViewMode}
            editableControlPoint={selectedProgramControlPoint}
            canEditWaypointValues={canTreeWaypointValueEdit}
            canAddWaypoint={canTreeWaypointAdd}
            canRemoveWaypoint={canTreeWaypointRemove}
            canApplyWaypointEdits={canTreeWaypointApply}
            onToggleExpand={handleToggleProgramTreeNode}
            onSelectNode={handleSelectProgramTreeNode}
            onChangeViewMode={handleChangeProgramTreeViewMode}
            onWaypointChange={handleTreeWaypointChange}
            onAddWaypoint={handleTreeAddWaypoint}
            onRemoveWaypoint={handleTreeRemoveWaypoint}
            onApplyWaypointEdits={handleApplyTreeWaypointEdits}
          />
        ) : null}
        <div className="pointer-events-none absolute left-1/2 top-4 z-20 -translate-x-1/2 transform">
          <AlertsPanel
            alerts={alerts}
            onDismiss={(idx) =>
              setAlerts((prev) => {
                const copy = [...prev];
                copy.splice(idx, 1);
                return copy;
              })
            }
          />
        </div>
        {isVisionActive && !visionError ? (
          <div className="pointer-events-auto absolute bottom-6 left-24 z-20 w-72 overflow-hidden rounded-xl border border-slate-700/60 bg-slate-950/85 shadow-lg shadow-slate-950/40 backdrop-blur">
            <img
              src={visionStreamUrl}
              alt="Gradient Vision stream"
              className="block max-h-56 w-full object-cover"
              onLoad={() => setVisionError(null)}
              onError={() => {
                setVisionError(
                  "Unable to load the vision stream. Ensure Gradient Vision is running and accessible.",
                );
                setIsVisionActive(false);
              }}
            />
          </div>
        ) : null}
        {isRobotControlCollapsed ? (
          <div className="pointer-events-auto absolute bottom-6 right-6 z-20 w-[340px] max-w-[calc(100vw-2rem)]">
            <button
              type="button"
              onClick={() => updateSettings({ collapseRobotControl: false })}
              className="flex w-full items-center justify-between rounded-lg border border-slate-700/60 bg-slate-900/80 px-3 py-2 text-left shadow-md shadow-slate-900/30 backdrop-blur transition hover:border-slate-500/70"
              aria-label="Show robot control panel"
            >
              <span className="text-xs font-semibold uppercase tracking-[0.22em] text-cyan-200/80">
                Robot Control
              </span>
              <ChevronRight size={16} className="text-slate-300/80" />
            </button>
          </div>
        ) : (
          <div className="pointer-events-auto absolute bottom-6 right-6 z-20 w-[340px] max-w-[calc(100vw-2rem)]">
            <button
              type="button"
              onClick={() => updateSettings({ collapseRobotControl: true })}
              className="absolute right-3 top-3 z-10 rounded-lg border border-slate-700/70 bg-slate-900/70 p-1 text-slate-300 transition hover:border-slate-500 hover:text-slate-100"
              aria-label="Hide robot control panel"
              title="Hide robot control panel"
            >
              <ChevronDown size={14} />
            </button>
            <ControlPanel apiHost={normalizedApiHost} onError={(m) => setError(m)} />
          </div>
        )}
      </main>
      <SettingsDialog
        isOpen={isSettingsOpen}
        initialTab={settingsInitialTab}
        apiHost={apiHost}
        visionHost={visionHost}
        showBoundingBox={showBoundingBox}
        robots={robotOptions}
        selectedRobotName={selectedRobotName}
        tools={toolLibrary}
        selectedToolId={selectedToolId}
        toolFilterRobotId={toolFilterRobotId}
        toolFilterType={toolFilterType}
        toolFilterQuery={toolFilterQuery}
        toolDraft={toolDraft}
        activeRuntimeConfig={runtimeConfigSnapshot?.active ?? null}
        restartRequired={Boolean(runtimeConfigSnapshot?.restart_required)}
        runtimeConfigError={runtimeConfigError}
        isRuntimeConfigBusy={isRuntimeConfigBusy}
        isRestartingController={isRestartingController}
        isToolLibraryBusy={isToolLibraryBusy}
        toolLibraryError={toolLibraryError}
        kinematics={kinematicsSnapshot}
        baseOffsetDraft={baseOffsetDraft}
        toolOffsetDraft={toolOffsetDraft}
        isKinematicsBusy={isKinematicsBusy}
        kinematicsError={kinematicsError}
        onHostChange={setApiHost}
        onVisionHostChange={setVisionHost}
        onShowBoundingBoxChange={(value) => updateSettings({ showBoundingBox: value })}
        onSelectedRobotNameChange={setSelectedRobotName}
        onSelectedToolIdChange={setSelectedToolId}
        onToolFilterRobotIdChange={setToolFilterRobotId}
        onToolFilterTypeChange={setToolFilterType}
        onToolFilterQueryChange={setToolFilterQuery}
        onToolDraftChange={setToolDraft}
        onRefreshTools={() => {
          setIsToolLibraryBusy(true);
          fetchToolLibrarySnapshot()
            .catch((err) => setToolLibraryError((err as Error).message))
            .finally(() => setIsToolLibraryBusy(false));
        }}
        onNewToolDraft={handleNewToolDraft}
        onCreateTool={handleCreateTool}
        onUpdateTool={handleUpdateTool}
        onDeleteTool={handleDeleteTool}
        onRefreshRuntimeConfig={() => {
          setIsRuntimeConfigBusy(true);
          fetchRuntimeConfigSnapshot()
            .catch((err) => setRuntimeConfigError((err as Error).message))
            .finally(() => setIsRuntimeConfigBusy(false));
        }}
        onApplyRuntimeConfig={handleApplyRuntimeConfig}
        onRestartController={handleRestartController}
        onOffsetDraftChange={handleOffsetDraftChange}
        onRefreshKinematics={() => {
          setIsKinematicsBusy(true);
          fetchKinematicsSnapshot()
            .catch((err) => setKinematicsError((err as Error).message))
            .finally(() => setIsKinematicsBusy(false));
        }}
        onApplyOffsets={handleApplyRuntimeOffsets}
        onResetOffsets={handleResetRuntimeOffsets}
        onClose={() => setIsSettingsOpen(false)}
      />
    </div>
  );
}
