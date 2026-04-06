// NOTE: Keep `use_cache` disabled for standard saved trajectory runs/loading.
// Preview planned-step caches are seeded from the robot's live joints at plan
// time, so replaying them later from a different pose can produce unsafe jumps.
// Only weld previews should opt back into cache-backed execution.
import {
  Suspense,
  lazy,
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
  type PointerEvent as ReactPointerEvent,
} from "react";
import { createPortal } from "react-dom";
import {
  Camera,
  CameraOff,
  Crosshair,
  FolderOpen,
  Flame,
  Home,
  Moon,
  Octagon,
  Play,
  Plus,
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
  type ArmVisualizerHandle,
  type StepLoadStatus,
  type TopologyEdgeOverlay,
  type StepTransform,
} from "./ArmVisualizer";
import { TelemetryWorkspace } from "./TelemetryWorkspace";
import ControlPanel, { ControlPanelRuntimeHeader } from "./ControlPanel";
import {
  buildProgramTree,
  coercePlannerDiagnostics,
  encodePointsForApi,
  encodePoseWaypointsForApi,
  coercePoseWaypointList,
  mergePoseWaypointMotionMetadata,
  previewFromPlannerPayload,
  previewFromTrajectoryDetail,
  type PlannerDiagnostics,
  type ProgramNode,
  type ProgramTreeViewMode,
  type Point3,
  type PoseWaypoint,
  type PreviewPlan,
  type SavedRobotProgramRecord,
  type WaypointMoveType,
} from "./previewUtils";
import { SidebarRail, type SidebarItem } from "./components/SidebarRail";
import { SidebarDrawer } from "./components/SidebarDrawer";
import { ProgramFeatureTree } from "./components/ProgramFeatureTree";
import {
  ProgramTimeline,
  type ProgramTimelineItem,
  type ProgramTimelineLane,
} from "./components/ProgramTimeline";
import { LiveStateProvider, LIVE_MONITOR_STALE_MS } from "./liveState";

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
  pos_counts?: number;
  torque_raw?: number;
  statusword?: number;
  statusword_hex?: string;
  error_code?: number;
  error_code_hex?: string;
  mode_display?: number;
  mode_display_name?: string;
  ds402_state?: string;
  ds402_state_code?: number;
  di_bits?: number;
  di_bits_hex?: string;
  axis_fault_flags?: number;
  brake_state?: number;
  logical_joint?: number | null;
  axis_index?: number;
  status_names?: string[];
};

type DriveFaultReference = {
  profile_id?: string;
  label?: string;
  source_path?: string;
  source_path_relative?: string;
  available?: boolean;
};

type DriveFaultDetail = {
  error_code_hex?: string;
  profile_id?: string;
  decoded?: boolean;
  code?: string;
  name?: string;
  class?: string | null;
  resettable?: boolean;
  bus_fault_code_hex?: string | null;
  bus_fault_name?: string | null;
  source?: string;
};

type DriveStartupConfig = {
  profile_id?: string;
  setting_key?: string;
  setting_label?: string;
  object?: string;
  configured?: boolean;
  commanded?: number;
  commanded_value_label?: string;
  readback_valid?: boolean;
  readback?: number;
  readback_value_label?: string;
  verified?: boolean;
};

type DriveFaultAxis = {
  axis: number;
  logical_joint?: number | null;
  ds402_state: string;
  statusword: number;
  error_code: number;
  error_code_hex?: string;
  manufacturer_error_code?: number;
  manufacturer_error_code_hex?: string;
  startup_drive_config?: DriveStartupConfig | null;
  slave_online?: boolean;
  slave_operational?: boolean;
  slave_al_state?: number;
  slave_al_state_name?: string;
  pos_counts?: number;
  fault?: DriveFaultDetail | null;
  manufacturer_fault?: DriveFaultDetail | null;
};

type DriveFaultSnapshot = {
  servo_backend?: string | null;
  drive_profile?: string | null;
  configured_drive_profile?: string | null;
  live_drive_profile?: string | null;
  drive_profile_source?: string | null;
  fieldbus_profile?: string | null;
  reference?: DriveFaultReference | null;
  physical_state?: string;
  driver_state?: string;
  ethercat_master_state?: string;
  rtcore_state?: string;
  armed?: number;
  axis_enable_mask?: number;
  axis_enable_mask_hex?: string;
  op_enabled_axes?: number;
  num_axes?: number;
  faulted_axes?: number;
  link_up?: number;
  responding?: number;
  online?: number;
  operational?: number;
  startup_ready?: number;
  startup_drive_config_configured_axes?: number;
  startup_drive_config_verified_axes?: number;
  startup_drive_config_mismatch_axes?: number;
  wkc_actual?: number;
  wkc_expected?: number;
  master_al?: number;
  master_al_hex?: string;
  axes?: DriveFaultAxis[];
  metrics_path?: string;
};

type TelemetryEvent = {
  timestamp: number;
  raw: string;
  joints?: number[];
  gripper?: number;
  servos?: Record<string, ServoSample>;
  alerts?: Alert[];
  drive_faults?: DriveFaultSnapshot | null;
  weld_active?: boolean;
  weld_type?: string;
  comms?: Record<string, unknown>;
  motion_status?: MotionStatusResponse | null;
};

type MotionExecutionPayload = {
  controller_motion_state?: string;
  controller_thread_running?: boolean;
  state_name?: string;
  active_mode_name?: string;
  active_traj_id?: number;
  queue_depth?: number;
  queue_capacity?: number;
  motion_done?: boolean;
  stale_command?: boolean;
  underrun_count?: number;
  last_submitted_traj_id?: number;
};

type ProgramStatusPayload = {
  name?: string | null;
  active?: boolean;
  state?: string;
  terminal_reason?: string | null;
  failing_step_index?: number | null;
  completed_step_count?: number;
  completed_loop_count?: number;
  loop_enabled?: boolean;
  use_cache?: boolean;
  step_count?: number;
  move_steps?: number;
  pause_steps?: number;
  joint_move_steps?: number;
  rtcore_segments?: boolean;
  segment_execution_policy?: string;
  current_step_index?: number | null;
  current_step_type?: string | null;
  loop_iteration?: number;
};

type MotionStatusResponse = {
  status?: string;
  detail?: string;
  accepted?: boolean;
  command_acknowledged?: boolean;
  execution_mode?: string;
  runtime_mode?: string;
  state?: string;
  completion_scope?: string;
  trajectory_id?: number;
  source_of_truth?: string;
  execution?: MotionExecutionPayload;
  program?: ProgramStatusPayload;
};

type PlannerFailureState = {
  message: string;
  plannerDiagnostics?: PlannerDiagnostics;
};

type EditableProgramMove = {
  moveIndex: number;
  waypointIndex: number;
  moveType: WaypointMoveType;
  startWaypointIndex: number;
  endWaypointIndex: number;
  moveDistanceMm: number;
  speedMode: "linear" | "angular";
  speedValue: number | null;
  speedUnitLabel: string;
  speedStep: number;
  hasCustomSpeed: boolean;
  supportsAccelerationEdit: boolean;
  accelerationValue: number | null;
  accelerationUnitLabel: string;
  accelerationStep: number;
  hasCustomAcceleration: boolean;
  supportsPauseEdit: boolean;
  pauseAfterSeconds: number | null;
  pauseStep: number;
  hasCustomPause: boolean;
};

function parseApiErrorPayload(rawBody: string, fallbackMessage: string): PlannerFailureState {
  const trimmedBody = rawBody.trim();
  if (!trimmedBody) {
    return { message: fallbackMessage };
  }
  try {
    const payload = JSON.parse(trimmedBody) as { detail?: unknown };
    if (typeof payload?.detail === "string") {
      return { message: payload.detail.trim() || fallbackMessage };
    }
    if (payload?.detail && typeof payload.detail === "object") {
      const detail = payload.detail as Record<string, unknown>;
      const parsedDiagnostics = coercePlannerDiagnostics(
        detail.planner_diagnostics ?? detail.plannerDiagnostics,
      );
      const message =
        typeof detail.message === "string" && detail.message.trim()
          ? detail.message.trim()
          : fallbackMessage;
      return {
        message,
        plannerDiagnostics: parsedDiagnostics,
      };
    }
  } catch {
    // Non-JSON error body; fall through to plain-text message.
  }
  return { message: trimmedBody || fallbackMessage };
}

async function readApiErrorResponse(response: Response): Promise<PlannerFailureState> {
  const fallbackMessage = `${response.status} ${response.statusText}`.trim();
  let rawBody = "";
  try {
    rawBody = await response.text();
  } catch {
    rawBody = "";
  }
  return parseApiErrorPayload(rawBody, fallbackMessage);
}

const LIVE_JOINT_FALLBACK_MAX_AGE_S = LIVE_MONITOR_STALE_MS / 1000;
const TERMINAL_MOTION_STATES = new Set(["idle", "completed", "aborted", "faulted", "underrun", "timeout"]);
const FAILED_MOTION_STATES = new Set(["aborted", "faulted", "underrun", "timeout"]);

function areJointArraysClose(a: number[] | undefined, b: number[]): boolean {
  if (!Array.isArray(a) || a.length !== b.length) {
    return false;
  }
  for (let index = 0; index < b.length; index += 1) {
    if (Math.abs((a[index] ?? 0) - b[index]) > 1e-5) {
      return false;
    }
  }
  return true;
}

function formatPlannerNumber(
  value: number | null | undefined,
  digits = 3,
): string | null {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    return null;
  }
  return value.toFixed(digits);
}

function describePlannerJump(
  residuals: PlannerDiagnostics["residuals"] | undefined,
): string | null {
  if (!residuals) {
    return null;
  }
  const parts: string[] = [];
  if (typeof residuals.jumpJointIndex === "number") {
    parts.push(`joint J${Math.round(residuals.jumpJointIndex)}`);
  }
  const prevValue = formatPlannerNumber(residuals.jumpJointPreviousRad);
  const currValue = formatPlannerNumber(residuals.jumpJointCurrentRad);
  if (prevValue && currValue) {
    parts.push(`prev ${prevValue} -> curr ${currValue}`);
  }
  const rawStep = formatPlannerNumber(residuals.jumpJointRawStepRad);
  if (rawStep) {
    parts.push(`raw ${rawStep} rad`);
  }
  const wrappedStep = formatPlannerNumber(residuals.jumpJointWrappedStepRad);
  if (wrappedStep) {
    parts.push(`wrapped ${wrappedStep} rad`);
  }
  if (typeof residuals.jumpPoseIndex === "number") {
    parts.push(`sample ${Math.round(residuals.jumpPoseIndex) + 1}`);
  }
  if (residuals.stepSource) {
    parts.push(`source ${residuals.stepSource}`);
  }
  if (residuals.jumpContext) {
    parts.push(`stage ${residuals.jumpContext}`);
  }
  return parts.length > 0 ? parts.join(" | ") : null;
}

type Axis3 = { x: number; y: number; z: number };

type RuntimeOffset = {
  position_m: Axis3;
  rotation_deg: Axis3;
};

type ShellPaneLayout = Pick<
  PersistedSettings,
  "leftPaneWidthPx" | "rightPaneWidthPx" | "timelineHeightPx"
>;

type ActiveShellDrag = {
  kind: "left" | "right" | "timeline" | "rightDock";
  startX: number;
  startY: number;
  initialLayout: ShellPaneLayout;
  initialProgramTreeHeightPx: number;
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
  default_drive_profile?: string | null;
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
  drive_profile?: {
    configured_profile?: string | null;
    configured_source?: string | null;
    live_profile?: string | null;
    live_source?: string | null;
    effective_profile?: string | null;
    source?: string;
    backend_default_profile?: string | null;
    override_profile?: string | null;
  };
  rtcore?: {
    configured_max_rpm?: number | null;
    configured_source?: string | null;
    effective_max_rpm?: number | null;
    source?: string | null;
    default_max_rpm?: number | null;
    override_max_rpm?: number | null;
    clamp_disabled?: boolean;
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
    sim_mode?: boolean;
    active_tool_id?: string | null;
    allow_unsafe_overrides?: boolean;
    overrides?: {
      ik_solver_backend?: string | null;
      servo_backend?: string | null;
      drive_profile?: string | null;
      rt_max_rpm?: number | null;
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
  showGripperPanel: boolean;
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
  leftPaneWidthPx: number;
  rightPaneWidthPx: number;
  timelineHeightPx: number;
  programTreeHeightPx: number;
};

type SidebarPanelId = "step" | "trajectory" | "tools" | "weld" | "telemetry";

type TimelineSyntheticSelection = {
  id: string;
  label: string;
  detail?: string;
  openPanel?: "trajectory" | "weld";
  pathRange?: { start: number; end: number } | null;
  waypointIndices?: number[];
  relatedMoveIndex?: number | null;
};

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

type TrajectoryProgramRecord = SavedRobotProgramRecord & {
  kind: "trajectory";
  authoring: {
    waypoints?: unknown;
    metadata?: Record<string, unknown>;
  };
};

type RuntimeExecutionMode = "simulate" | "live";

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
const TRAJECTORY_MOVE_POSITION_EPS_M = 1e-5;
const TRAJECTORY_MOVE_ROTATION_EPS_DEG = 1e-2;

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

function normalizeMotionState(status: MotionStatusResponse | null | undefined): string {
  return (
    status?.state?.trim().toLowerCase() ||
    status?.execution?.state_name?.trim().toLowerCase() ||
    "idle"
  );
}

function resolveMotionTrajectoryId(status: MotionStatusResponse | null | undefined): number | null {
  const directId = status?.trajectory_id;
  if (typeof directId === "number" && Number.isFinite(directId) && directId > 0) {
    return directId;
  }
  const activeId = status?.execution?.active_traj_id;
  if (typeof activeId === "number" && Number.isFinite(activeId) && activeId > 0) {
    return activeId;
  }
  const submittedId = status?.execution?.last_submitted_traj_id;
  if (typeof submittedId === "number" && Number.isFinite(submittedId) && submittedId > 0) {
    return submittedId;
  }
  return null;
}

function shouldShowMotionStatus(status: MotionStatusResponse | null | undefined): boolean {
  if (!status) {
    return false;
  }
  const programState = status.program?.state?.trim().toLowerCase();
  if (programState && programState !== "idle") {
    return true;
  }
  const state = normalizeMotionState(status);
  if (!TERMINAL_MOTION_STATES.has(state)) {
    return true;
  }
  return Boolean(
    status.accepted ||
    status.command_acknowledged ||
    resolveMotionTrajectoryId(status) !== null ||
    (typeof status.execution?.queue_depth === "number" && status.execution.queue_depth > 0),
  );
}

function isMotionActive(status: MotionStatusResponse | null | undefined): boolean {
  if (!status) {
    return false;
  }
  const programState = status.program?.state?.trim().toLowerCase();
  if (status.program?.active || (programState && !TERMINAL_MOTION_STATES.has(programState))) {
    return true;
  }
  const state = normalizeMotionState(status);
  if (TERMINAL_MOTION_STATES.has(state)) {
    return false;
  }
  return Boolean(
    status.accepted ||
    status.command_acknowledged ||
    resolveMotionTrajectoryId(status) !== null ||
    (typeof status.execution?.queue_depth === "number" && status.execution.queue_depth > 0),
  );
}

type MotionStatusCardProps = {
  title: string;
  motionStatus: MotionStatusResponse | null;
  isSubmitting?: boolean;
  submittingLabel?: string;
  alwaysVisible?: boolean;
  compact?: boolean;
  fixedHeightClassName?: string;
};

function MotionStatusCard({
  title,
  motionStatus,
  isSubmitting = false,
  submittingLabel = "Submitting motion request…",
  alwaysVisible = false,
  compact = false,
  fixedHeightClassName = "",
}: MotionStatusCardProps) {
  if (!alwaysVisible && !isSubmitting && !shouldShowMotionStatus(motionStatus)) {
    return null;
  }
  const state = isSubmitting ? "submitting" : (motionStatus ? normalizeMotionState(motionStatus) : "idle");
  const trajectoryId = motionStatus ? resolveMotionTrajectoryId(motionStatus) : null;
  const queueDepth = motionStatus?.execution?.queue_depth;
  const queueCapacity = motionStatus?.execution?.queue_capacity;
  const source = motionStatus?.source_of_truth ?? "controller";
  const scope = motionStatus?.completion_scope ?? "unknown";
  const program = motionStatus?.program;
  const motionActive = isSubmitting || isMotionActive(motionStatus);
  const toneClasses = motionActive
    ? {
        wrapper: "border-cyan-400/30 bg-cyan-500/10",
        title: "text-cyan-200/90",
        text: "text-cyan-100/90",
        badge: "border-cyan-300/40 bg-cyan-400/15 text-cyan-100",
        meta: "border-cyan-300/20 bg-cyan-400/10 text-cyan-100/80",
      }
    : state === "completed"
      ? {
          wrapper: "border-emerald-400/30 bg-emerald-500/10",
          title: "text-emerald-200/90",
          text: "text-emerald-100/90",
          badge: "border-emerald-300/40 bg-emerald-400/15 text-emerald-100",
          meta: "border-emerald-300/20 bg-emerald-400/10 text-emerald-100/80",
        }
      : state === "idle"
        ? {
            wrapper: "border-slate-700/60 bg-slate-950/45",
            title: "text-slate-300/90",
            text: "text-slate-200/90",
            badge: "border-slate-600/70 bg-slate-900/70 text-slate-200",
            meta: "border-slate-700/60 bg-slate-900/55 text-slate-300/85",
          }
        : FAILED_MOTION_STATES.has(state)
          ? {
              wrapper: "border-rose-400/30 bg-rose-500/10",
              title: "text-rose-200/90",
              text: "text-rose-100/90",
              badge: "border-rose-300/40 bg-rose-400/15 text-rose-100",
              meta: "border-rose-300/20 bg-rose-400/10 text-rose-100/80",
            }
          : {
              wrapper: "border-amber-400/30 bg-amber-500/10",
              title: "text-amber-200/90",
              text: "text-amber-100/90",
              badge: "border-amber-300/40 bg-amber-400/15 text-amber-100",
              meta: "border-amber-300/20 bg-amber-400/10 text-amber-100/80",
            };
  const detailParts = [`source ${source}`, `scope ${scope}`];
  if (motionStatus?.runtime_mode) {
    detailParts.push(`runtime ${motionStatus.runtime_mode}`);
  }
  if (motionStatus?.execution_mode) {
    detailParts.push(`request ${motionStatus.execution_mode}`);
  }
  if (typeof queueDepth === "number" && Number.isFinite(queueDepth)) {
    detailParts.push(
      typeof queueCapacity === "number" && Number.isFinite(queueCapacity) && queueCapacity > 0
        ? `queue ${queueDepth}/${queueCapacity}`
        : `queue ${queueDepth}`,
    );
  }
  if (motionStatus?.execution?.controller_motion_state) {
    detailParts.push(`controller ${motionStatus.execution.controller_motion_state}`);
  }
  if (program?.name) {
    detailParts.push(`program ${program.name}`);
  }
  if (program?.segment_execution_policy) {
    detailParts.push(`program path ${program.segment_execution_policy}`);
  }
  const detailMessage =
    typeof motionStatus?.detail === "string" && motionStatus.detail.trim()
      ? motionStatus.detail.trim()
      : "";

  const programState = program?.state?.trim().toLowerCase();
  const programSummaryParts: string[] = [];
  if (program && programState && programState !== "idle") {
    const currentProgram = program;
    programSummaryParts.push(`program ${programState}`);
    if (
      typeof currentProgram.current_step_index === "number" &&
      typeof currentProgram.step_count === "number" &&
      currentProgram.step_count > 0
    ) {
      programSummaryParts.push(
        `step ${currentProgram.current_step_index + 1}/${currentProgram.step_count}`,
      );
    } else if (
      typeof currentProgram.completed_step_count === "number" &&
      typeof currentProgram.step_count === "number" &&
      currentProgram.step_count > 0
    ) {
      programSummaryParts.push(
        `completed ${currentProgram.completed_step_count}/${currentProgram.step_count}`,
      );
    }
    if (
      typeof currentProgram.completed_loop_count === "number" &&
      currentProgram.completed_loop_count > 0
    ) {
      programSummaryParts.push(`loops ${currentProgram.completed_loop_count}`);
    }
    if (currentProgram.terminal_reason) {
      programSummaryParts.push(`reason ${currentProgram.terminal_reason}`);
    }
    if (typeof currentProgram.failing_step_index === "number") {
      programSummaryParts.push(`failed step ${currentProgram.failing_step_index + 1}`);
    }
  }

  const stateLabel = state.replace(/_/g, " ");
  const summaryText = isSubmitting
    ? submittingLabel
    : motionActive
      ? (program?.name ? `${program.name} is currently executing.` : "Controller is executing the latest motion request.")
      : state === "completed"
        ? (program?.name ? `${program.name} completed successfully.` : "Latest motion request completed.")
        : state === "idle"
          ? "No controller execution in progress."
          : detailMessage || `Latest motion request ended in ${stateLabel}.`;
  const runtimeValue = motionStatus?.runtime_mode?.trim().toUpperCase()
    || motionStatus?.execution?.active_mode_name?.trim().toUpperCase()
    || "READY";
  const requestValue = motionStatus?.execution_mode?.trim().toUpperCase()
    || (isSubmitting ? "PENDING" : "IDLE");
  const progressValue = (
    typeof program?.current_step_index === "number" &&
    typeof program?.step_count === "number" &&
    program.step_count > 0
  )
    ? `${program.current_step_index + 1}/${program.step_count}`
    : (
        typeof program?.completed_step_count === "number" &&
        typeof program?.step_count === "number" &&
        program.step_count > 0
      )
      ? `${program.completed_step_count}/${program.step_count}`
      : (typeof queueDepth === "number" && Number.isFinite(queueDepth))
        ? (typeof queueCapacity === "number" && Number.isFinite(queueCapacity) && queueCapacity > 0
            ? `${queueDepth}/${queueCapacity}`
            : `${queueDepth}`)
        : state === "completed"
          ? "done"
          : state === "idle"
            ? "waiting"
            : stateLabel;
  const compactMeta = [
    { label: "Runtime", value: runtimeValue },
    { label: "Request", value: requestValue },
    { label: trajectoryId !== null ? "Traj" : "Program", value: trajectoryId !== null ? String(trajectoryId) : (program?.name?.trim() || "none") },
    { label: "Progress", value: progressValue },
  ];
  const compactDetail = detailMessage || programSummaryParts.join(" | ") || detailParts.join(" | ");

  if (compact) {
    return (
      <div className={`flex flex-col rounded-xl border px-3 py-2.5 ${toneClasses.wrapper} ${fixedHeightClassName}`.trim()}>
        <div className="flex items-start justify-between gap-2">
          <div className={`text-[10px] font-semibold uppercase tracking-[0.16em] ${toneClasses.title}`}>
            {title}
          </div>
          <span className={`shrink-0 rounded-full border px-2 py-0.5 text-[10px] font-semibold uppercase tracking-[0.14em] ${toneClasses.badge}`}>
            {stateLabel}
          </span>
        </div>
        <div className={`mt-2 text-[12px] font-medium leading-4 ${toneClasses.text}`}>
          {summaryText}
        </div>
        <div
          className={`mt-1 truncate text-[10px] leading-4 ${toneClasses.text} opacity-80`}
          title={compactDetail || undefined}
        >
          {compactDetail || "Awaiting the next trajectory execution request."}
        </div>
        <div className="mt-auto grid grid-cols-4 gap-1.5 pt-2">
          {compactMeta.map((item) => (
            <div
              key={`${title}-${item.label}`}
              className={`min-w-0 rounded-md border px-1.5 py-1 ${toneClasses.meta}`}
            >
              <div className="truncate text-[8px] font-semibold uppercase tracking-[0.16em] opacity-75">
                {item.label}
              </div>
              <div className="truncate text-[10px] font-semibold">
                {item.value}
              </div>
            </div>
          ))}
        </div>
      </div>
    );
  }

  return (
    <div className={`rounded border px-2 py-2 ${toneClasses.wrapper}`}>
      <div className={`mb-1 text-[11px] font-semibold uppercase tracking-[0.14em] ${toneClasses.title}`}>
        {title}
      </div>
      <div className="mb-1 flex items-center gap-2">
        <span className={`rounded-full border px-2 py-0.5 text-[10px] font-semibold uppercase tracking-[0.14em] ${toneClasses.badge}`}>
          {stateLabel}
        </span>
        {trajectoryId !== null ? (
          <span className={`text-[11px] font-semibold ${toneClasses.text}`}>traj {trajectoryId}</span>
        ) : null}
      </div>
      <div className={`text-[12px] leading-5 ${toneClasses.text}`}>{detailParts.join(" | ")}</div>
      {programSummaryParts.length > 0 ? (
        <div className={`mt-1 text-[12px] leading-5 ${toneClasses.text}`}>
          {programSummaryParts.join(" | ")}
        </div>
      ) : null}
      {detailMessage ? (
        <div className={`mt-1 text-[12px] leading-5 ${toneClasses.text}`}>
          {detailMessage}
        </div>
      ) : null}
    </div>
  );
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
const DEFAULT_LEFT_PANE_WIDTH_PX = 368;
const DEFAULT_RIGHT_PANE_WIDTH_PX = 384;
const DEFAULT_TIMELINE_HEIGHT_PX = 188;
const DEFAULT_PROGRAM_TREE_HEIGHT_PX = 320;
const MIN_LEFT_PANE_WIDTH_PX = 220;
const MAX_LEFT_PANE_WIDTH_PX = 640;
const MIN_RIGHT_PANE_WIDTH_PX = 220;
const MAX_RIGHT_PANE_WIDTH_PX = 560;
const MIN_TIMELINE_HEIGHT_PX = 72;
const MAX_TIMELINE_HEIGHT_PX = 520;
const MIN_PROGRAM_TREE_HEIGHT_PX = 180;
const MIN_ROBOT_CONTROL_HEIGHT_PX = 220;
const MIN_CENTER_STAGE_WIDTH_PX = 320;
const SHELL_SPLITTER_SIZE_PX = 8;
const SHELL_WIDTH_CHROME_PX = 120;
const SHELL_HEIGHT_CHROME_PX = 220;
// Keep initial visualizer identity stable before runtime snapshot hydration completes.
const DEFAULT_VISUALIZER_ROBOT_ID = "gradient-05";
const LazyArmVisualizer = lazy(async () => {
  const module = await import("./ArmVisualizer");
  return { default: module.ArmVisualizer };
});
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

function coercePaneSize(value: unknown, fallback: number): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function clampShellPaneLayout(
  layout: ShellPaneLayout,
  viewportWidth: number,
  viewportHeight: number,
): ShellPaneLayout {
  const safeViewportWidth = Number.isFinite(viewportWidth) ? viewportWidth : 1600;
  const safeViewportHeight = Number.isFinite(viewportHeight) ? viewportHeight : 900;

  const initialLeft = clamp(
    coercePaneSize(layout.leftPaneWidthPx, DEFAULT_LEFT_PANE_WIDTH_PX),
    MIN_LEFT_PANE_WIDTH_PX,
    MAX_LEFT_PANE_WIDTH_PX,
  );
  const rightMax = Math.max(
    MIN_RIGHT_PANE_WIDTH_PX,
    Math.min(
      MAX_RIGHT_PANE_WIDTH_PX,
      safeViewportWidth - initialLeft - MIN_CENTER_STAGE_WIDTH_PX - SHELL_WIDTH_CHROME_PX,
    ),
  );
  const right = clamp(
    coercePaneSize(layout.rightPaneWidthPx, DEFAULT_RIGHT_PANE_WIDTH_PX),
    MIN_RIGHT_PANE_WIDTH_PX,
    rightMax,
  );
  const leftMax = Math.max(
    MIN_LEFT_PANE_WIDTH_PX,
    Math.min(
      MAX_LEFT_PANE_WIDTH_PX,
      safeViewportWidth - right - MIN_CENTER_STAGE_WIDTH_PX - SHELL_WIDTH_CHROME_PX,
    ),
  );
  const left = clamp(initialLeft, MIN_LEFT_PANE_WIDTH_PX, leftMax);
  const timelineMax = Math.max(
    MIN_TIMELINE_HEIGHT_PX,
    Math.min(MAX_TIMELINE_HEIGHT_PX, safeViewportHeight - SHELL_HEIGHT_CHROME_PX),
  );
  const timeline = clamp(
    coercePaneSize(layout.timelineHeightPx, DEFAULT_TIMELINE_HEIGHT_PX),
    MIN_TIMELINE_HEIGHT_PX,
    timelineMax,
  );

  return {
    leftPaneWidthPx: left,
    rightPaneWidthPx: right,
    timelineHeightPx: timeline,
  };
}

function sameShellPaneLayout(a: ShellPaneLayout, b: ShellPaneLayout): boolean {
  return (
    a.leftPaneWidthPx === b.leftPaneWidthPx &&
    a.rightPaneWidthPx === b.rightPaneWidthPx &&
    a.timelineHeightPx === b.timelineHeightPx
  );
}

function clampProgramTreePaneHeight(value: unknown, containerHeight: number): number {
  const safeContainerHeight =
    Number.isFinite(containerHeight) && containerHeight > 0
      ? containerHeight
      : DEFAULT_PROGRAM_TREE_HEIGHT_PX + MIN_ROBOT_CONTROL_HEIGHT_PX + SHELL_SPLITTER_SIZE_PX;
  const maxProgramTreeHeight = Math.max(
    MIN_PROGRAM_TREE_HEIGHT_PX,
    safeContainerHeight - MIN_ROBOT_CONTROL_HEIGHT_PX - SHELL_SPLITTER_SIZE_PX,
  );
  return clamp(
    coercePaneSize(value, DEFAULT_PROGRAM_TREE_HEIGHT_PX),
    MIN_PROGRAM_TREE_HEIGHT_PX,
    maxProgramTreeHeight,
  );
}

function loadPersistedSettings(): PersistedSettings {
  const defaults: PersistedSettings = {
    showBoundingBox: true,
    showGripperPanel: false,
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
    leftPaneWidthPx: DEFAULT_LEFT_PANE_WIDTH_PX,
    rightPaneWidthPx: DEFAULT_RIGHT_PANE_WIDTH_PX,
    timelineHeightPx: DEFAULT_TIMELINE_HEIGHT_PX,
    programTreeHeightPx: DEFAULT_PROGRAM_TREE_HEIGHT_PX,
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
        showGripperPanel:
          typeof parsed.showGripperPanel === "boolean"
            ? parsed.showGripperPanel
            : defaults.showGripperPanel,
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
        leftPaneWidthPx: coercePaneSize(parsed.leftPaneWidthPx, defaults.leftPaneWidthPx),
        rightPaneWidthPx: coercePaneSize(parsed.rightPaneWidthPx, defaults.rightPaneWidthPx),
        timelineHeightPx: coercePaneSize(parsed.timelineHeightPx, defaults.timelineHeightPx),
        programTreeHeightPx: coercePaneSize(
          parsed.programTreeHeightPx,
          defaults.programTreeHeightPx,
        ),
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

type StepTransformMatrix = {
  position: Point3;
  scale: number;
  quaternion: { x: number; y: number; z: number; w: number };
};

function degToRad(value: number): number {
  return (value * Math.PI) / 180;
}

function buildStepTransformMatrix(transform: StepTransform): StepTransformMatrix {
  const safeScale = Number.isFinite(transform.scale) ? Math.max(1e-4, transform.scale) : 1;
  const x = degToRad(transform.rotationDeg.x) * 0.5;
  const y = degToRad(transform.rotationDeg.y) * 0.5;
  const z = degToRad(transform.rotationDeg.z) * 0.5;
  const c1 = Math.cos(x);
  const c2 = Math.cos(y);
  const c3 = Math.cos(z);
  const s1 = Math.sin(x);
  const s2 = Math.sin(y);
  const s3 = Math.sin(z);
  return {
    position: {
      x: transform.position.x,
      y: transform.position.y,
      z: transform.position.z,
    },
    scale: safeScale,
    quaternion: {
      x: s1 * c2 * c3 + c1 * s2 * s3,
      y: c1 * s2 * c3 - s1 * c2 * s3,
      z: c1 * c2 * s3 + s1 * s2 * c3,
      w: c1 * c2 * c3 - s1 * s2 * s3,
    },
  };
}

function rotatePointByQuaternion(
  point: Point3,
  quaternion: StepTransformMatrix["quaternion"],
): Point3 {
  const { x, y, z, w } = quaternion;
  const ix = w * point.x + y * point.z - z * point.y;
  const iy = w * point.y + z * point.x - x * point.z;
  const iz = w * point.z + x * point.y - y * point.x;
  const iw = -x * point.x - y * point.y - z * point.z;

  return {
    x: ix * w + iw * -x + iy * -z - iz * -y,
    y: iy * w + iw * -y + iz * -x - ix * -z,
    z: iz * w + iw * -z + ix * -y - iy * -x,
  };
}

function transformTopologyPointToScene(
  point: Point3,
  topologyOffset: Point3 | null,
  stepMatrix: StepTransformMatrix,
): Point3 {
  const localPoint = {
    x: (topologyOffset ? point.x - topologyOffset.x : point.x) * stepMatrix.scale,
    y: (topologyOffset ? point.y - topologyOffset.y : point.y) * stepMatrix.scale,
    z: (topologyOffset ? point.z - topologyOffset.z : point.z) * stepMatrix.scale,
  };
  const rotated = rotatePointByQuaternion(localPoint, stepMatrix.quaternion);
  return {
    x: rotated.x + stepMatrix.position.x,
    y: rotated.y + stepMatrix.position.y,
    z: rotated.z + stepMatrix.position.z,
  };
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

function buildPoseWaypoint(
  point: Point3,
  orientation?: Partial<
    Pick<
      PoseWaypoint,
      | "rollDeg"
      | "pitchDeg"
      | "yawDeg"
      | "moveType"
      | "linearSpeedMmPerSec"
      | "linearAccelerationMmPerSec2"
      | "rotationSpeedDegPerSec"
      | "pauseAfterSeconds"
    >
  > | null,
): PoseWaypoint {
  return {
    x: Number(point.x),
    y: Number(point.y),
    z: Number(point.z),
    rollDeg: Number.isFinite(Number(orientation?.rollDeg)) ? Number(orientation?.rollDeg) : null,
    pitchDeg: Number.isFinite(Number(orientation?.pitchDeg)) ? Number(orientation?.pitchDeg) : null,
    yawDeg: Number.isFinite(Number(orientation?.yawDeg)) ? Number(orientation?.yawDeg) : null,
    moveType:
      orientation?.moveType === "joint" || orientation?.moveType === "home"
        ? orientation.moveType
        : "linear",
    linearSpeedMmPerSec:
      Number.isFinite(Number(orientation?.linearSpeedMmPerSec)) && Number(orientation?.linearSpeedMmPerSec) > 0
        ? Number(orientation?.linearSpeedMmPerSec)
        : null,
    linearAccelerationMmPerSec2:
      Number.isFinite(Number(orientation?.linearAccelerationMmPerSec2)) &&
      Number(orientation?.linearAccelerationMmPerSec2) > 0
        ? Number(orientation?.linearAccelerationMmPerSec2)
        : null,
    rotationSpeedDegPerSec:
      Number.isFinite(Number(orientation?.rotationSpeedDegPerSec)) && Number(orientation?.rotationSpeedDegPerSec) > 0
        ? Number(orientation?.rotationSpeedDegPerSec)
        : null,
    pauseAfterSeconds:
      Number.isFinite(Number(orientation?.pauseAfterSeconds)) && Number(orientation?.pauseAfterSeconds) > 0
        ? Number(orientation?.pauseAfterSeconds)
        : null,
  };
}

function wrappedAngleDeltaDeg(left: number, right: number): number {
  return Math.abs((((left - right) % 360) + 540) % 360 - 180);
}

function poseWaypointRotationDeltaDeg(start: PoseWaypoint | null | undefined, end: PoseWaypoint): number {
  if (
    !start ||
    start.rollDeg === null ||
    start.pitchDeg === null ||
    start.yawDeg === null ||
    end.rollDeg === null ||
    end.pitchDeg === null ||
    end.yawDeg === null
  ) {
    return 0;
  }
  return Math.max(
    wrappedAngleDeltaDeg(end.rollDeg, start.rollDeg),
    wrappedAngleDeltaDeg(end.pitchDeg, start.pitchDeg),
    wrappedAngleDeltaDeg(end.yawDeg, start.yawDeg),
  );
}

function poseWaypointTranslationDistanceM(start: PoseWaypoint | null | undefined, end: PoseWaypoint): number {
  if (!start) {
    return 0;
  }
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const dz = end.z - start.z;
  return Math.sqrt((dx * dx) + (dy * dy) + (dz * dz));
}

function shouldUseAngularMoveSpeed(start: PoseWaypoint | null | undefined, end: PoseWaypoint): boolean {
  if (end.moveType === "joint" || end.moveType === "home") {
    return true;
  }
  if (!start) {
    return false;
  }
  const translationDistance = poseWaypointTranslationDistanceM(start, end);
  return (
    translationDistance <= TRAJECTORY_MOVE_POSITION_EPS_M &&
    poseWaypointRotationDeltaDeg(start, end) > TRAJECTORY_MOVE_ROTATION_EPS_DEG
  );
}

function poseOrientationSummary(waypoint: PoseWaypoint): string {
  const values = [waypoint.rollDeg, waypoint.pitchDeg, waypoint.yawDeg];
  const moveSummary =
    waypoint.moveType === "joint"
      ? "joint move"
      : waypoint.moveType === "home"
        ? "move to home"
        : "linear move";
  if (values.some((value) => value === null || !Number.isFinite(Number(value)))) {
    return `${moveSummary}, orientation inherited`;
  }
  return `${moveSummary}, R ${Number(waypoint.rollDeg).toFixed(1)} deg, P ${Number(waypoint.pitchDeg).toFixed(1)} deg, Y ${Number(waypoint.yawDeg).toFixed(1)} deg`;
}

function clonePoseWaypointList(waypoints: PoseWaypoint[]): PoseWaypoint[] {
  return waypoints.map((waypoint) => ({ ...waypoint }));
}

function poseWaypointListsMatch(left: PoseWaypoint[], right: PoseWaypoint[]): boolean {
  if (left.length !== right.length) {
    return false;
  }
  const posTolerance = 1e-4;
  const orientToleranceDeg = 0.5;
  return left.every((waypoint, index) => {
    const planned = right[index];
    const positionMatches =
      Math.abs(planned.x - waypoint.x) <= posTolerance &&
      Math.abs(planned.y - waypoint.y) <= posTolerance &&
      Math.abs(planned.z - waypoint.z) <= posTolerance;
    if (!positionMatches) {
      return false;
    }
    if ((waypoint.moveType ?? "linear") !== (planned.moveType ?? "linear")) {
      return false;
    }
    if ((waypoint.linearSpeedMmPerSec ?? null) !== (planned.linearSpeedMmPerSec ?? null)) {
      return false;
    }
    if ((waypoint.linearAccelerationMmPerSec2 ?? null) !== (planned.linearAccelerationMmPerSec2 ?? null)) {
      return false;
    }
    if ((waypoint.rotationSpeedDegPerSec ?? null) !== (planned.rotationSpeedDegPerSec ?? null)) {
      return false;
    }
    if ((waypoint.pauseAfterSeconds ?? null) !== (planned.pauseAfterSeconds ?? null)) {
      return false;
    }
    const authoredValues = [waypoint.rollDeg, waypoint.pitchDeg, waypoint.yawDeg];
    const plannedValues = [planned.rollDeg, planned.pitchDeg, planned.yawDeg];
    const authoredHasOrientation = authoredValues.every((value) => value !== null && Number.isFinite(Number(value)));
    const plannedHasOrientation = plannedValues.every((value) => value !== null && Number.isFinite(Number(value)));
    if (authoredHasOrientation !== plannedHasOrientation) {
      return false;
    }
    if (!authoredHasOrientation) {
      return true;
    }
    return authoredValues.every(
      (value, axisIndex) => Math.abs(Number(value) - Number(plannedValues[axisIndex])) <= orientToleranceDeg,
    );
  });
}

function previewPlanMatchesWaypoints(plan: PreviewPlan | null, waypoints: PoseWaypoint[]): boolean {
  if (!plan) {
    return false;
  }
  return poseWaypointListsMatch(plan.waypoints, waypoints);
}

function pointDistanceSquared(a: Point3, b: Point3): number {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  const dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

function describeWaypointMoveType(moveType: WaypointMoveType): string {
  return moveType === "joint" ? "Joint move" : moveType === "home" ? "Move to home" : "Linear move";
}

function formatPauseAfterSecondsLabel(seconds: number): string {
  return `${Number(seconds.toFixed(seconds >= 10 ? 0 : 1))}s pause`;
}

function describeTrajectoryControlPointKind(moveType: WaypointMoveType): string {
  return moveType === "linear" ? "Pose Capture" : "Waypoint";
}

function buildOrderedWaypointPathIndices(pathPoints: Point3[], waypoints: Point3[]): number[] {
  if (pathPoints.length === 0 || waypoints.length === 0) {
    return [];
  }
  let searchStart = 0;
  return waypoints.map((waypoint) => {
    let bestIndex = Math.max(0, Math.min(pathPoints.length - 1, searchStart));
    let bestDistance = Number.POSITIVE_INFINITY;
    for (let index = searchStart; index < pathPoints.length; index += 1) {
      const distance = pointDistanceSquared(pathPoints[index], waypoint);
      if (distance <= bestDistance) {
        bestDistance = distance;
        bestIndex = index;
      }
    }
    searchStart = bestIndex;
    return bestIndex;
  });
}

function normalizeTrajectoryProgramRecord(raw: unknown): TrajectoryProgramRecord | null {
  if (!raw || typeof raw !== "object") {
    return null;
  }
  const record = raw as Record<string, unknown>;
  if (record.kind !== "trajectory") {
    return null;
  }
  const name = typeof record.name === "string" ? record.name.trim() : "";
  const authoring =
    record.authoring && typeof record.authoring === "object"
      ? (record.authoring as TrajectoryProgramRecord["authoring"])
      : null;
  if (!name || !authoring) {
    return null;
  }
  return {
    name,
    kind: "trajectory",
    saved_at: typeof record.saved_at === "string" ? record.saved_at : undefined,
    authoring,
    planned_trajectory:
      record.planned_trajectory && typeof record.planned_trajectory === "object"
        ? (record.planned_trajectory as PreviewPlan)
        : null,
    metadata:
      record.metadata && typeof record.metadata === "object"
        ? (record.metadata as Record<string, unknown>)
        : undefined,
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

function PaneResizeHandle({
  orientation,
  active = false,
  ariaLabel,
  onPointerDown,
}: {
  orientation: "vertical" | "horizontal";
  active?: boolean;
  ariaLabel?: string;
  onPointerDown: (event: ReactPointerEvent<HTMLButtonElement>) => void;
}) {
  const isVertical = orientation === "vertical";
  return (
    <div
      className={`flex items-center justify-center ${
        isVertical ? "h-full w-full cursor-col-resize" : "h-full w-full cursor-row-resize"
      }`}
    >
      <button
        type="button"
        aria-label={ariaLabel ?? (isVertical ? "Resize side panes" : "Resize timeline height")}
        onPointerDown={onPointerDown}
        className={`group relative flex items-center justify-center rounded-full border border-slate-800/80 bg-slate-950/88 transition ${
          isVertical
            ? "h-20 w-1.5 cursor-col-resize"
            : "h-1.5 w-20 cursor-row-resize"
        } ${active ? "border-cyan-400/60 bg-cyan-500/15" : "hover:border-cyan-400/40 hover:bg-cyan-500/10"}`}
      >
        <span
          className={`rounded-full bg-slate-500/70 transition group-hover:bg-cyan-200/80 ${
            isVertical ? "h-10 w-px" : "h-px w-10"
          } ${active ? "bg-cyan-200/90" : ""}`}
        />
      </button>
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
  isSubmittingRun: boolean;
  isMotionActive: boolean;
  motionStatus: MotionStatusResponse | null;
  preview: PreviewPlan | null;
  plannerFailure: PlannerFailureState | null;
  plannerPoints: PoseWaypoint[];
  savedTrajectories: string[];
  selectedTrajectory: string;
  isTrajectoryListLoading: boolean;
  isLoadingSavedTrajectory: boolean;
  trajectoryProgramName: string;
  onTrajectoryProgramNameChange: (value: string) => void;
  onSaveTrajectory: () => void;
  isSavingTrajectory: boolean;
  runtimeMode: RuntimeExecutionMode | null;
  onPlanToggle: () => void;
  onSimulate: () => void;
  onRunLive: () => void;
  loopEnabled: boolean;
  onLoopEnabledChange: (enabled: boolean) => void;
  onCapturePose: () => void;
  onAddWaypoint: () => void;
  onAddHomeWaypoint: () => void;
  onClear: () => void;
  onRefreshTrajectories: () => void;
  onSelectTrajectory: (value: string) => void;
  onLoadTrajectory: () => void;
  onRegenerateTrajectory: () => void;
  onUndoPoint: () => void;
};

function TrajectoryPanel({
  isPlanning,
  isPlanLoading,
  isSubmittingRun,
  isMotionActive,
  motionStatus,
  preview,
  plannerFailure,
  plannerPoints,
  savedTrajectories,
  selectedTrajectory,
  isTrajectoryListLoading,
  isLoadingSavedTrajectory,
  trajectoryProgramName,
  onTrajectoryProgramNameChange,
  onSaveTrajectory,
  isSavingTrajectory,
  runtimeMode,
  onPlanToggle,
  onSimulate,
  onRunLive,
  loopEnabled,
  onLoopEnabledChange,
  onCapturePose,
  onAddWaypoint,
  onAddHomeWaypoint,
  onClear,
  onRefreshTrajectories,
  onSelectTrajectory,
  onLoadTrajectory,
  onRegenerateTrajectory,
  onUndoPoint,
}: TrajectoryPanelProps) {
  const waypointList = plannerPoints.length > 0 ? plannerPoints : preview?.waypoints ?? [];
  const waypointCount = waypointList.length;
  const hasSavedTrajectories = savedTrajectories.length > 0;
  const lastPoint =
    waypointList.length > 0 ? waypointList[waypointList.length - 1] : null;
  const previewMatchesDraft = previewPlanMatchesWaypoints(preview, plannerPoints);
  const hasRunnablePreview = Boolean(preview) && previewMatchesDraft && !preview?.isStale;
  const canSimulate = runtimeMode === "simulate" && hasRunnablePreview;
  const canRunLive = runtimeMode === "live" && hasRunnablePreview;
  const hasDraftWaypoints = plannerPoints.length > 0;
  const interactionLocked = isPlanLoading || isSubmittingRun || isMotionActive;
  const runtimeModeLabel = runtimeMode === "simulate" ? "SIM" : runtimeMode === "live" ? "LIVE" : "UNKNOWN";
  const executionGuidance =
    runtimeMode === "simulate"
      ? "Simulated execution is available while the controller is running in SIM mode."
      : runtimeMode === "live"
        ? "Simulated execution is disabled while the controller is in LIVE mode. Use Run Trajectory for the real robot, or switch the controller to SIM."
        : "Connect to the controller to resolve whether SIM or LIVE execution is currently available.";
  const simulateDisabledReason =
    interactionLocked
      ? "Execution is temporarily locked while planning or another motion is active."
      : !hasRunnablePreview
        ? "Create or regenerate a fresh trajectory preview before simulating."
        : runtimeMode === "live"
          ? "Simulated execution is only available while the controller is in SIM mode."
          : runtimeMode === null
            ? "Connect to the controller to resolve the active runtime mode."
            : null;
  const runDisabledReason =
    interactionLocked
      ? "Execution is temporarily locked while planning or another motion is active."
      : !hasRunnablePreview
        ? "Create or regenerate a fresh trajectory preview before running."
        : runtimeMode === "simulate"
          ? "Live execution is unavailable while the controller is in SIM mode."
          : runtimeMode === null
            ? "Connect to the controller to resolve the active runtime mode."
            : null;
  const savedPlanIsStale = Boolean(preview?.isStale);
  const planningWarnings = preview?.planningWarnings ?? [];
  const activePlannerDiagnostics = plannerFailure?.plannerDiagnostics ?? preview?.plannerDiagnostics;
  const plannerSummaryParts: string[] = [];
  if (activePlannerDiagnostics?.attempt) {
    plannerSummaryParts.push(`attempt ${activePlannerDiagnostics.attempt}`);
  }
  if (activePlannerDiagnostics?.reasonCode) {
    plannerSummaryParts.push(`reason ${activePlannerDiagnostics.reasonCode}`);
  }
  if (typeof activePlannerDiagnostics?.fallbackLevel === "number") {
    plannerSummaryParts.push(`fallback ${activePlannerDiagnostics.fallbackLevel}`);
  }
  if (typeof activePlannerDiagnostics?.residuals?.jumpPoseIndex === "number") {
    plannerSummaryParts.push(
      `jump sample ${Math.round(activePlannerDiagnostics.residuals.jumpPoseIndex) + 1}`,
    );
  }
  if (activePlannerDiagnostics?.residuals?.stepSource) {
    plannerSummaryParts.push(`source ${activePlannerDiagnostics.residuals.stepSource}`);
  }
  if (typeof activePlannerDiagnostics?.residuals?.maxJointStepRad === "number") {
    plannerSummaryParts.push(
      `max step ${activePlannerDiagnostics.residuals.maxJointStepRad.toFixed(3)} rad`,
    );
  }
  if (typeof activePlannerDiagnostics?.residuals?.jumpJointIndex === "number") {
    plannerSummaryParts.push(`jump joint J${Math.round(activePlannerDiagnostics.residuals.jumpJointIndex)}`);
  }
  if (activePlannerDiagnostics?.branchAnchorAvailable) {
    plannerSummaryParts.push("branch anchor available");
  }
  if (activePlannerDiagnostics?.recovery?.used) {
    plannerSummaryParts.push(
      `jump recovery ${activePlannerDiagnostics.recovery.strategy ?? activePlannerDiagnostics.recovery.kind ?? "used"}`,
    );
  }
  if (activePlannerDiagnostics?.splitRecovery?.used) {
    const splitCount = activePlannerDiagnostics.splitRecovery.splitCount;
    plannerSummaryParts.push(
      typeof splitCount === "number"
        ? `segment split ${splitCount}x`
        : "segment split used",
    );
  }
  const plannerJumpDetail = describePlannerJump(activePlannerDiagnostics?.residuals);
  const plannerRawJumpDetail = describePlannerJump(activePlannerDiagnostics?.rawJump);
  const plannerRecoveryAttemptDetails = (activePlannerDiagnostics?.recovery?.attempts ?? []).map(
    (attempt, index) => {
      const parts: string[] = [];
      if (attempt.strategy) {
        parts.push(attempt.strategy);
      } else {
        parts.push(`attempt ${index + 1}`);
      }
      if (typeof attempt.suffixStartIndex === "number") {
        parts.push(`suffix ${Math.round(attempt.suffixStartIndex) + 1}`);
      }
      if (attempt.reasonCode) {
        parts.push(`reason ${attempt.reasonCode}`);
      }
      if (typeof attempt.accepted === "boolean") {
        parts.push(attempt.accepted ? "accepted" : "rejected");
      }
      const rawJumpDetail = describePlannerJump(attempt.rawJump);
      if (rawJumpDetail) {
        parts.push(`raw ${rawJumpDetail}`);
      }
      const attemptJumpDetail = describePlannerJump(attempt.residuals);
      if (attemptJumpDetail) {
        parts.push(`gate ${attemptJumpDetail}`);
      }
      return parts.join(" | ");
    },
  );
  const splitSegmentDetails = (activePlannerDiagnostics?.splitRecovery?.segments ?? []).map(
    (segment, index) => {
      const parts: string[] = [];
      const segmentIndex = Number(segment.segment_index ?? segment.segmentIndex ?? index + 1);
      if (Number.isFinite(segmentIndex)) {
        parts.push(`segment ${Math.round(segmentIndex)}`);
      }
      if (typeof segment.accepted === "boolean") {
        parts.push(segment.accepted ? "accepted" : "rejected");
      }
      const plannerDiagnostics = coercePlannerDiagnostics(
        segment.planner_diagnostics ?? segment.plannerDiagnostics,
      );
      if (plannerDiagnostics?.reasonCode) {
        parts.push(`reason ${plannerDiagnostics.reasonCode}`);
      }
      const segmentJumpDetail = describePlannerJump(plannerDiagnostics?.residuals);
      if (segmentJumpDetail) {
        parts.push(segmentJumpDetail);
      }
      return parts.join(" | ");
    },
  );
  return (
    <div className="pointer-events-auto w-full">
      <MotionStatusCard
        title="Execution Status"
        motionStatus={motionStatus}
        isSubmitting={isSubmittingRun}
        submittingLabel="Submitting trajectory run..."
        alwaysVisible
        compact
        fixedHeightClassName="h-[112px]"
      />
      <div className="mt-3 rounded-xl border border-slate-700/60 bg-slate-950/40 px-3 py-3">
        <div className="mb-2 flex items-center justify-between">
          <span className={DRAWER_SECTION_TITLE_CLASS}>Create Trajectory</span>
          <span className="rounded border border-cyan-500/20 bg-cyan-500/8 px-2 py-1 text-[10px] font-semibold uppercase tracking-[0.14em] text-cyan-100/90">
            {hasDraftWaypoints ? `${plannerPoints.length} draft waypoint(s)` : "Start here"}
          </span>
        </div>
        <div className="rounded-lg border border-slate-700/60 bg-slate-900/40 px-2.5 py-2 text-[12px] leading-5 text-slate-300/90">
          <div>
            1. Click <span className="font-semibold text-slate-100">{isPlanning ? "Stop Editing" : "Start Editing"}</span>.
          </div>
          <div>
            2. Use <span className="font-semibold text-slate-100">Shift-click in the 3D workspace</span> or <span className="font-semibold text-slate-100">Linear Move</span> to add a linear move.
          </div>
          <div>
            3. Use <span className="font-semibold text-slate-100">Joint Move</span> for a joint move or <span className="font-semibold text-slate-100">Move to Home</span> for a home return.
          </div>
          <div>
            4. Adjust XYZ and roll/pitch/yaw in the <span className="font-semibold text-slate-100">Program Tree</span>.
          </div>
        </div>
        <div className="mt-3 grid grid-cols-2 gap-2">
          <button
            type="button"
            onClick={onPlanToggle}
            disabled={interactionLocked}
            className={`rounded-xl border px-3 py-2 text-left transition ${
              isPlanning
                ? "border-cyan-400/60 bg-cyan-500/18 text-cyan-50 shadow-inner shadow-cyan-500/10"
                : "border-cyan-500/40 bg-cyan-500/12 text-cyan-50 hover:border-cyan-300/70 hover:bg-cyan-500/18"
            } ${interactionLocked ? "opacity-60" : ""}`}
          >
            <div className={`${DRAWER_ACTION_TEXT_CLASS}`}>{isPlanning ? "Stop Editing" : "Start Editing"}</div>
            <div className="mt-0.5 text-[11px] text-current/75">
              {isPlanning ? "Finish placing points" : "Enable waypoint placement"}
            </div>
          </button>
          <button
            type="button"
            onClick={onCapturePose}
            disabled={interactionLocked}
            className={`rounded-xl border border-slate-600/60 bg-slate-900/60 px-3 py-2 text-left text-slate-100 transition hover:border-slate-400 hover:text-slate-50 ${
              interactionLocked ? "opacity-60" : ""
            }`}
          >
            <div className={`${DRAWER_ACTION_TEXT_CLASS}`}>Linear Move</div>
            <div className="mt-0.5 text-[11px] text-slate-400">
              Read the robot TCP and append a linear move
            </div>
          </button>
          <button
            type="button"
            onClick={onAddWaypoint}
            disabled={interactionLocked}
            className={`rounded-xl border border-slate-600/60 bg-slate-900/60 px-3 py-2 text-left text-slate-100 transition hover:border-slate-400 hover:text-slate-50 ${
              interactionLocked ? "opacity-60" : ""
            }`}
          >
            <span className={`inline-flex items-center gap-2 ${DRAWER_ACTION_TEXT_CLASS}`}>
              <Plus size={14} />
              Joint Move
            </span>
            <div className="mt-0.5 text-[11px] text-slate-400">
              Read the robot joints and append a joint move
            </div>
          </button>
          <button
            type="button"
            onClick={onAddHomeWaypoint}
            disabled={interactionLocked}
            className={`rounded-xl border border-slate-600/60 bg-slate-900/60 px-3 py-2 text-left text-slate-100 transition hover:border-slate-400 hover:text-slate-50 ${
              interactionLocked ? "opacity-60" : ""
            }`}
          >
            <div className={`inline-flex items-center gap-2 ${DRAWER_ACTION_TEXT_CLASS}`}>
              <Home size={14} />
              Move to Home
            </div>
            <div className="mt-0.5 text-[11px] text-slate-400">
              Insert a commanded home move
            </div>
          </button>
          <button
            type="button"
            onClick={onUndoPoint}
            disabled={plannerPoints.length === 0 || interactionLocked}
            className={`col-span-2 rounded-xl border border-slate-600/60 bg-slate-900/60 px-3 py-2 text-left text-slate-100 transition hover:border-slate-400 hover:text-slate-50 ${
              plannerPoints.length === 0 || interactionLocked ? "opacity-60" : ""
            }`}
          >
            <div className={`inline-flex items-center gap-2 ${DRAWER_ACTION_TEXT_CLASS}`}>
              <Undo2 size={14} />
              Undo Last
            </div>
            <div className="mt-0.5 text-[11px] text-slate-400">
              Remove the most recent point
            </div>
          </button>
        </div>
      </div>
      <div className="mt-3 rounded-xl border border-slate-700/60 bg-slate-950/40 px-3 py-3">
        <div className="mb-2 flex items-center justify-between">
          <span className={DRAWER_SECTION_TITLE_CLASS}>Draft Summary</span>
          <span className={DRAWER_META_TEXT_CLASS}>
            {preview ? "preview ready" : hasDraftWaypoints ? "draft only" : "empty"}
          </span>
        </div>
        <div className="grid grid-cols-2 gap-2">
          <div className="rounded-lg border border-slate-700/60 bg-slate-900/45 px-2.5 py-2">
            <div className="text-[10px] font-semibold uppercase tracking-[0.14em] text-slate-400">Waypoints</div>
            <div className="mt-1 text-[18px] font-semibold text-slate-100">{waypointCount}</div>
          </div>
          <div className="rounded-lg border border-slate-700/60 bg-slate-900/45 px-2.5 py-2">
            <div className="text-[10px] font-semibold uppercase tracking-[0.14em] text-slate-400">Preview</div>
            <div className="mt-1 text-[18px] font-semibold text-slate-100">
              {preview ? "Ready" : isPlanLoading ? "Planning" : "None"}
            </div>
          </div>
        </div>
        <div className="mt-2 rounded-lg border border-slate-700/60 bg-slate-900/35 px-2.5 py-2 text-[12px] leading-5 text-slate-300/90">
          {isPlanLoading ? (
            <p>Planning preview trajectory…</p>
          ) : isSubmittingRun ? (
            <p>Submitting trajectory run…</p>
          ) : lastPoint ? (
            <>
              <div>
                Last point:{" "}
                <span className="font-semibold text-slate-100">
                  {lastPoint.x.toFixed(3)}, {lastPoint.y.toFixed(3)}, {lastPoint.z.toFixed(3)} m
                </span>
              </div>
              <div className="mt-1">
                Pose: <span className="font-semibold text-slate-100">{poseOrientationSummary(lastPoint)}</span>
              </div>
            </>
          ) : (
            <p>Start editing, place or capture waypoints, and the preview will appear automatically.</p>
          )}
        </div>
      </div>
      <div className="mt-3 rounded-xl border border-slate-700/60 bg-slate-950/40 px-3 py-3">
        <div className="mb-2 flex items-center justify-between">
          <span className={DRAWER_SECTION_TITLE_CLASS}>Execution</span>
          <span className={DRAWER_META_TEXT_CLASS}>path preview is local</span>
        </div>
        <div className="mb-3 rounded-lg border border-slate-700/60 bg-slate-900/35 px-2.5 py-2 text-[12px] leading-5 text-slate-300/90">
          <div className="mb-1 flex items-center gap-2">
            <span className="rounded border border-slate-700/70 bg-slate-900/60 px-2 py-1 text-[11px] font-semibold uppercase tracking-[0.14em] text-slate-200">
              Runtime {runtimeModeLabel}
            </span>
          </div>
          <div>Preview path is already drawn locally in the authoring stage.</div>
          <div className="mt-1">{executionGuidance}</div>
        </div>
        <label
          className={`mb-3 flex cursor-pointer items-start gap-3 rounded-lg border border-slate-700/60 bg-slate-900/35 px-2.5 py-2 ${
            interactionLocked ? "opacity-60" : ""
          }`}
        >
          <input
            type="checkbox"
            checked={loopEnabled}
            onChange={(event) => onLoopEnabledChange(event.target.checked)}
            disabled={interactionLocked}
            className="mt-0.5 h-4 w-4 rounded border-slate-500/70 bg-slate-950 text-cyan-400 accent-cyan-400"
          />
          <span className="min-w-0">
            <span className="block text-[12px] font-semibold text-slate-100">Loop trajectory</span>
            <span className="block text-[11px] leading-5 text-slate-300/80">
              Runtime only. The controller moves to the first waypoint before the loop and closes each pass back to the start without saving that wrapper move into the trajectory.
            </span>
          </span>
        </label>
        <div className="grid grid-cols-2 gap-2">
          <button
            type="button"
            onClick={onSimulate}
            disabled={!canSimulate || interactionLocked}
            title={simulateDisabledReason ?? "Send the planned path to the controller running in SIM mode."}
            className={`rounded-xl border border-cyan-500/50 bg-cyan-500/10 px-3 py-2 ${DRAWER_ACTION_TEXT_CLASS} text-cyan-100 transition hover:border-cyan-300 hover:text-cyan-50 ${
              !canSimulate || interactionLocked ? "cursor-not-allowed opacity-60" : ""
            }`}
          >
            Simulate Trajectory
          </button>
          <button
            type="button"
            onClick={onRunLive}
            disabled={!canRunLive || interactionLocked}
            title={runDisabledReason ?? "Execute the planned path on the current LIVE controller."}
            className={`rounded-xl border border-rose-500/50 bg-rose-500/10 px-3 py-2 ${DRAWER_ACTION_TEXT_CLASS} text-rose-100 transition hover:border-rose-300 hover:text-rose-50 ${
              !canRunLive || interactionLocked ? "cursor-not-allowed opacity-60" : ""
            }`}
          >
            Run Trajectory
          </button>
        </div>
      </div>
      <div className="mt-3 rounded-xl border border-slate-700/60 bg-slate-950/40 px-3 py-3">
        <div className="mb-2 flex items-center justify-between">
          <span className={DRAWER_SECTION_TITLE_CLASS}>Save Program</span>
          <span className={DRAWER_META_TEXT_CLASS}>
            {preview ? "ready to save" : "needs preview"}
          </span>
        </div>
        <div className="flex items-center gap-2">
          <input
            value={trajectoryProgramName}
            onChange={(event) => onTrajectoryProgramNameChange(event.target.value)}
            placeholder="trajectory_program"
            className={`flex-1 ${DRAWER_INLINE_INPUT_CLASS} px-3 py-2`}
          />
          <button
            type="button"
            onClick={onSaveTrajectory}
            disabled={isSavingTrajectory || !preview}
            className={`rounded-lg border border-slate-600/60 bg-slate-900/60 px-3 py-2 ${DRAWER_ACTION_TEXT_CLASS} text-slate-100 transition hover:border-slate-400 hover:text-slate-50 ${
              isSavingTrajectory || !preview ? "opacity-60" : ""
            }`}
          >
            {isSavingTrajectory ? "Saving…" : "Save"}
          </button>
        </div>
      </div>
      <div className="mt-3 rounded-xl border border-slate-700/60 bg-slate-950/40 px-3 py-3">
        <div className="mb-2 flex items-center justify-between">
          <span className={DRAWER_SECTION_TITLE_CLASS}>
            Saved Programs
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
          <p className={DRAWER_META_TEXT_CLASS}>Loading programs…</p>
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
            <button
              type="button"
              onClick={onRegenerateTrajectory}
              disabled={!hasDraftWaypoints || interactionLocked}
              className={`rounded-lg border border-slate-600/60 bg-slate-900/60 px-3 py-2 ${DRAWER_ACTION_TEXT_CLASS} text-slate-100 transition hover:border-slate-400 hover:text-slate-50 ${
                (!hasDraftWaypoints || interactionLocked)
                  ? "cursor-not-allowed opacity-60"
                  : ""
              }`}
              aria-label="Regenerate the trajectory path from saved waypoints"
            >
              Regenerate
            </button>
          </div>
        ) : (
          <p className={DRAWER_META_TEXT_CLASS}>
            No saved programs available.
          </p>
        )}
        {savedPlanIsStale ? (
          <p className={`${DRAWER_META_TEXT_CLASS} mt-2 text-amber-300`}>
            Saved path is stale for these waypoints. Regenerate to rebuild the runnable path without re-teaching points.
          </p>
        ) : null}
        {planningWarnings.length > 0 ? (
          <div className="mt-3 rounded-lg border border-amber-500/40 bg-amber-500/10 px-2.5 py-2">
            <div className="mb-1 text-[11px] font-semibold uppercase tracking-[0.14em] text-amber-200">
              Planning Warnings
            </div>
            <div className="space-y-1 text-[12px] leading-5 text-amber-100/90">
              {planningWarnings.map((warning) => (
                <div
                  key={`trajectory-planning-warning-${warning}`}
                  className="rounded border border-amber-500/20 bg-amber-500/5 px-1.5 py-1"
                >
                  {warning}
                </div>
              ))}
            </div>
          </div>
        ) : null}
        {plannerFailure || plannerSummaryParts.length > 0 ? (
          <div
            className={`mt-3 rounded-lg border px-2.5 py-2 ${
              plannerFailure
                ? "border-rose-500/40 bg-rose-500/10"
                : "border-cyan-500/30 bg-cyan-500/10"
            }`}
          >
            <div
              className={`mb-1 text-[11px] font-semibold uppercase tracking-[0.14em] ${
                plannerFailure ? "text-rose-200" : "text-cyan-200"
              }`}
            >
              {plannerFailure ? "Planner Failure Details" : "Planner Diagnostics"}
            </div>
            {plannerFailure ? (
              <div className="text-[12px] leading-5 text-rose-100/90">
                {plannerFailure.message}
              </div>
            ) : null}
            {plannerSummaryParts.length > 0 ? (
              <div
                className={`text-[12px] leading-5 ${
                  plannerFailure ? "mt-1 text-rose-100/85" : "text-cyan-100/85"
                }`}
              >
                {plannerSummaryParts.join(" | ")}
              </div>
            ) : null}
            {plannerJumpDetail ? (
              <div
                className={`mt-1 rounded border px-1.5 py-1 text-[12px] leading-5 ${
                  plannerFailure
                    ? "border-rose-400/20 bg-rose-500/5 text-rose-100/85"
                    : "border-cyan-400/20 bg-cyan-500/5 text-cyan-100/85"
                }`}
              >
                Gate jump: {plannerJumpDetail}
              </div>
            ) : null}
            {plannerRawJumpDetail ? (
              <div
                className={`mt-1 rounded border px-1.5 py-1 text-[12px] leading-5 ${
                  plannerFailure
                    ? "border-rose-400/20 bg-rose-500/5 text-rose-100/80"
                    : "border-cyan-400/20 bg-cyan-500/5 text-cyan-100/80"
                }`}
              >
                Raw solver jump: {plannerRawJumpDetail}
              </div>
            ) : null}
            {plannerRecoveryAttemptDetails.length > 0 ? (
              <div className="mt-2">
                <div
                  className={`text-[10px] font-semibold uppercase tracking-[0.14em] ${
                    plannerFailure ? "text-rose-200/80" : "text-cyan-200/80"
                  }`}
                >
                  Recovery Attempts
                </div>
                <div className="mt-1 space-y-1 text-[12px] leading-5">
                  {plannerRecoveryAttemptDetails.map((detail, index) => (
                    <div
                      key={`planner-recovery-attempt-${index}`}
                      className={`rounded border px-1.5 py-1 ${
                        plannerFailure
                          ? "border-rose-400/15 bg-rose-500/5 text-rose-100/85"
                          : "border-cyan-400/15 bg-cyan-500/5 text-cyan-100/85"
                      }`}
                    >
                      {detail}
                    </div>
                  ))}
                </div>
              </div>
            ) : null}
            {splitSegmentDetails.length > 0 ? (
              <div className="mt-2">
                <div
                  className={`text-[10px] font-semibold uppercase tracking-[0.14em] ${
                    plannerFailure ? "text-rose-200/80" : "text-cyan-200/80"
                  }`}
                >
                  Split Segments
                </div>
                <div className="mt-1 space-y-1 text-[12px] leading-5">
                  {splitSegmentDetails.map((detail, index) => (
                    <div
                      key={`planner-split-segment-${index}`}
                      className={`rounded border px-1.5 py-1 ${
                        plannerFailure
                          ? "border-rose-400/15 bg-rose-500/5 text-rose-100/85"
                          : "border-cyan-400/15 bg-cyan-500/5 text-cyan-100/85"
                      }`}
                    >
                      {detail}
                    </div>
                  ))}
                </div>
              </div>
            ) : null}
          </div>
        ) : null}
      </div>
      <div className="mt-3 flex justify-end">
        <button
          type="button"
          onClick={onClear}
          disabled={(!preview && !isPlanning && plannerPoints.length === 0) || isPlanLoading}
          className={`inline-flex items-center gap-2 rounded-lg border border-slate-600/60 bg-slate-900/60 px-3 py-2 text-[12px] font-semibold text-slate-200 transition hover:border-slate-400 hover:text-slate-100 ${
            ((!preview && !isPlanning && plannerPoints.length === 0) || isPlanLoading) ? "opacity-60" : ""
          }`}
        >
          <Trash2 size={14} />
          Clear Draft
        </button>
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
  isSubmittingRun: boolean;
  isMotionActive: boolean;
  motionStatus: MotionStatusResponse | null;
  canRunPreview: boolean;
  weldActive: boolean;
  planningWarnings: string[];
  onToggleSelection: () => void;
  onSelectEdge: (edgeId: string) => void;
  onRemoveEdge: (edgeId: string) => void;
  onPlanFromEdge: () => void;
  runtimeMode: RuntimeExecutionMode | null;
  onSimulate: () => void;
  onRunLive: () => void;
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
  isSubmittingRun,
  isMotionActive,
  motionStatus,
  canRunPreview,
  weldActive,
  planningWarnings,
  onToggleSelection,
  onSelectEdge,
  onRemoveEdge,
  onPlanFromEdge,
  runtimeMode,
  onSimulate,
  onRunLive,
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
      !isSubmittingRun &&
      !isMotionActive,
  );
  const canSimulate = runtimeMode === "simulate" && canRunPreview;
  const canRunLive = runtimeMode === "live" && canRunPreview;

  return (
    <div className="pointer-events-auto w-full">
      <div className="space-y-2 text-[13px] leading-[1.35] text-slate-200/90">
        <MotionStatusCard
          title="Execution Status"
          motionStatus={motionStatus}
          isSubmitting={isSubmittingRun}
          submittingLabel="Submitting weld preview run..."
        />
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
        <div className="rounded border border-slate-700/50 bg-slate-950/50 px-2 py-2 text-[12px] text-slate-300">
          <div className="mb-1 flex items-center gap-2">
            <span className="rounded border border-slate-700/70 bg-slate-900/60 px-2 py-1 text-[11px] font-semibold uppercase tracking-[0.14em] text-slate-200">
              Runtime {runtimeMode === "simulate" ? "SIM" : runtimeMode === "live" ? "LIVE" : "UNKNOWN"}
            </span>
          </div>
          <div>Preview draws the weld path locally. Simulated execution only runs safely when the controller is in SIM mode.</div>
        </div>
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
      <div className="mt-3 grid grid-cols-2 gap-2">
        <button
          type="button"
          onClick={onSimulate}
          disabled={!canSimulate || isSubmittingRun || isPlanningWeld || isMotionActive}
          className={`inline-flex w-full items-center justify-center gap-2 rounded border border-cyan-500/40 bg-cyan-500/20 px-3 py-2 text-sm font-semibold text-cyan-100 transition hover:bg-cyan-500/30 ${
            (!canSimulate || isSubmittingRun || isPlanningWeld || isMotionActive) ? "opacity-60" : ""
          }`}
        >
          <Play size={14} /> Simulate Weld
        </button>
        <button
          type="button"
          onClick={onRunLive}
          disabled={!canRunLive || isSubmittingRun || isPlanningWeld || isMotionActive}
          className={`inline-flex w-full items-center justify-center gap-2 rounded border border-orange-400/40 bg-orange-500/20 px-3 py-2 text-sm font-semibold text-orange-100 transition hover:bg-orange-500/30 ${
            (!canRunLive || isSubmittingRun || isPlanningWeld || isMotionActive) ? "opacity-60" : ""
          }`}
        >
          <Play size={14} /> Run Weld
        </button>
      </div>
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
  showGripperPanel: boolean;
  robots: RobotPolicyOption[];
  selectedRobotName: string;
  selectedRtMaxRpmInput: string;
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
  onShowGripperPanelChange: (value: boolean) => void;
  onSelectedRobotNameChange: (value: string) => void;
  onSelectedRtMaxRpmInputChange: (value: string) => void;
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
  showGripperPanel,
  robots,
  selectedRobotName,
  selectedRtMaxRpmInput,
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
  onShowGripperPanelChange,
  onSelectedRobotNameChange,
  onSelectedRtMaxRpmInputChange,
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
                      ? `active ${activeRuntimeConfig.robot.display_name} · ${activeRuntimeConfig.mode?.sim ? "SIM" : "LIVE"} · servo ${activeRuntimeConfig.servo_backend.effective_backend} · IK ${activeRuntimeConfig.ik_solver.effective_backend} · drive ${activeRuntimeConfig.drive_profile?.effective_profile ?? "none"} (${activeRuntimeConfig.drive_profile?.source ?? "unknown"})`
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
                {" · "}
                Drive profile:{" "}
                <span className="font-semibold text-cyan-100">
                  {selectedRobot?.default_drive_profile ?? "none"}
                </span>
              </div>
              {activeRuntimeConfig?.drive_profile ? (
                <div className="mt-2 text-[11px] text-slate-400">
                  Active RT drive profile: {activeRuntimeConfig.drive_profile.effective_profile ?? "none"} from{" "}
                  {activeRuntimeConfig.drive_profile.source ?? "unknown"}.
                  {" "}Configured: {activeRuntimeConfig.drive_profile.configured_profile ?? "none"}.
                  {" "}Live: {activeRuntimeConfig.drive_profile.live_profile ?? "unavailable"}.
                </div>
              ) : null}
              <label className="mt-3 block text-xs text-slate-300/90">
                RTCore max RPM
                <input
                  type="number"
                  min="0"
                  step="1"
                  className="mt-1 w-full rounded border border-slate-600/70 bg-slate-900/70 px-2 py-1 text-sm text-slate-100"
                  value={selectedRtMaxRpmInput}
                  onChange={(event) => onSelectedRtMaxRpmInputChange(event.target.value)}
                  disabled={isRuntimeConfigBusy || isRestartingController}
                />
                <span className="mt-1 block text-[11px] text-slate-400">
                  `0` disables the RTCore clamp. `6000` matches the drive absolute max and effectively removes the old `100 RPM` cap for normal motion.
                </span>
              </label>
              {activeRuntimeConfig?.rtcore ? (
                <div className="mt-2 text-[11px] text-slate-400">
                  Active RT clamp: {activeRuntimeConfig.rtcore.effective_max_rpm ?? "unknown"} RPM from{" "}
                  {activeRuntimeConfig.rtcore.source ?? "unknown"}.
                  {" "}Default: {activeRuntimeConfig.rtcore.default_max_rpm ?? "unknown"}.
                  {activeRuntimeConfig.rtcore.clamp_disabled ? " Clamp is disabled." : ""}
                </div>
              ) : null}
              {activeRuntimeConfig?.ik_solver?.source === "dev_override" ||
              activeRuntimeConfig?.servo_backend?.source === "dev_override" ||
              activeRuntimeConfig?.drive_profile?.source === "dev_override" ? (
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
              checked={showGripperPanel}
              onChange={(event) => onShowGripperPanelChange(event.target.checked)}
            />
            Show gripper panel
            </label>
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
  const emphasisFor = (alert: Alert) =>
    alert.kind === "JOINT_LIMIT" ? "ring-1 ring-amber-300/40 shadow-lg shadow-amber-900/20" : "";
  const iconFor = (lvl: Alert["level"]) =>
    lvl === "error" ? <Octagon size={16} /> : lvl === "warning" ? <Octagon size={16} /> : <Octagon size={16} />;
  return (
    <div className="pointer-events-auto flex max-w-sm flex-col gap-2">
      {alerts.slice(-4).map((a, idx) => (
        <div
          key={`${a.kind}:${a.ts ?? idx}:${idx}`}
          className={`flex items-start gap-2 rounded-lg border px-3 py-2 text-sm shadow-md ${colorFor(a.level)} ${emphasisFor(a)}`}
        >
          <div className="mt-0.5 shrink-0">{iconFor(a.level)}</div>
          <div className="flex-1">
            {a.kind === "JOINT_LIMIT" ? (
              <div className="mb-0.5 text-[10px] font-semibold uppercase tracking-[0.18em] text-current/80">
                Joint Limit Warning
              </div>
            ) : null}
            <div className="font-medium">{a.message}</div>
            {Array.isArray((a.details as { joint_labels?: unknown } | undefined)?.joint_labels) &&
            ((a.details as { joint_labels?: unknown[] }).joint_labels?.length ?? 0) > 0 ? (
              <div className="text-xs opacity-90">
                Joints: {((a.details as { joint_labels?: unknown[] }).joint_labels ?? []).map((value) => String(value)).join(", ")}
              </div>
            ) : null}
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
  const [isVisualizerEnabled, setIsVisualizerEnabled] = useState(false);
  const [isConnected, setIsConnected] = useState(false);
  const [latest, setLatest] = useState<TelemetryEvent | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [visionError, setVisionError] = useState<string | null>(null);
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const [settingsInitialTab, setSettingsInitialTab] = useState<SettingsDialogTab>("general");
  const [robotOptions, setRobotOptions] = useState<RobotPolicyOption[]>([]);
  const [runtimeConfigSnapshot, setRuntimeConfigSnapshot] = useState<RuntimeConfigSnapshot | null>(null);
  const [selectedRobotName, setSelectedRobotName] = useState("");
  const [selectedRuntimeMode, setSelectedRuntimeMode] = useState<RuntimeExecutionMode>("live");
  const [selectedRtMaxRpmInput, setSelectedRtMaxRpmInput] = useState("6000");
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
  const [trajectoryPlannerFailure, setTrajectoryPlannerFailure] = useState<PlannerFailureState | null>(null);
  const [plannerPoints, setPlannerPoints] = useState<PoseWaypoint[]>([]);
  const [isPlanning, setIsPlanning] = useState(false);
  const [isPlanLoading, setIsPlanLoading] = useState(false);
  const [isRunningPreview, setIsRunningPreview] = useState(false);
  const [showTrajectoryRunPreparationOverlay, setShowTrajectoryRunPreparationOverlay] = useState(false);
  const [motionStatus, setMotionStatus] = useState<MotionStatusResponse | null>(null);
  const [savedTrajectories, setSavedTrajectories] = useState<string[]>([]);
  const [isTrajectoryListLoading, setIsTrajectoryListLoading] = useState(false);
  const [selectedTrajectory, setSelectedTrajectory] = useState("");
  const [isLoadingSavedTrajectory, setIsLoadingSavedTrajectory] = useState(false);
  const [trajectoryLoopEnabled, setTrajectoryLoopEnabled] = useState(false);
  const [trajectoryProgramName, setTrajectoryProgramName] = useState("trajectory_program");
  const [isSavingTrajectoryProgram, setIsSavingTrajectoryProgram] = useState(false);
  const [loadedTrajectoryMetadata, setLoadedTrajectoryMetadata] = useState<Record<string, unknown> | null>(null);
  const [savedTrajectoryWaypoints, setSavedTrajectoryWaypoints] = useState<PoseWaypoint[]>([]);
  const [trajectoryDraftHistory, setTrajectoryDraftHistory] = useState<PoseWaypoint[][]>([]);
  const [selectedTimelineSyntheticId, setSelectedTimelineSyntheticId] = useState<string | null>(null);
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
  const [weldEditableWaypoints, setWeldEditableWaypoints] = useState<PoseWaypoint[]>([]);
  const [weldPreviewGhostJoints, setWeldPreviewGhostJoints] = useState<number[] | null>(null);
  const [weldPreviewCacheReady, setWeldPreviewCacheReady] = useState(false);
  const [showEndEffectorFrame, setShowEndEffectorFrame] = useState(false);
  const [weldProgramName, setWeldProgramName] = useState("weld_program");
  const [savedWeldPrograms, setSavedWeldPrograms] = useState<string[]>([]);
  const [selectedWeldProgram, setSelectedWeldProgram] = useState("");
  const [isWeldProgramListLoading, setIsWeldProgramListLoading] = useState(false);
  const [isSavingWeldProgram, setIsSavingWeldProgram] = useState(false);
  const [isLoadingWeldProgram, setIsLoadingWeldProgram] = useState(false);
  const [liveClockMs, setLiveClockMs] = useState(() => Date.now());
  const [pendingWeldProgramRestore, setPendingWeldProgramRestore] = useState<{
    weldDraft: WeldDraft;
    editableWaypoints: PoseWaypoint[];
    previewPlan: PreviewPlan | null;
  } | null>(null);
  const showBoundingBox = settings.showBoundingBox;
  const showGripperPanel = settings.showGripperPanel;
  const activePanel = settings.activePanel;
  const showProgramTree = settings.showProgramTree;
  const programTreeViewMode = settings.programTreeViewMode;
  const isRobotControlCollapsed = settings.collapseRobotControl;
  const expandedProgramTreeNodeIds = settings.expandedProgramTreeNodeIds;
  const selectedProgramNodeId = settings.selectedProgramNodeId;
  const visualizerRef = useRef<ArmVisualizerHandle | null>(null);
  const plannerPointsRef = useRef<PoseWaypoint[]>(plannerPoints);
  const previewPlanRef = useRef<PreviewPlan | null>(previewPlan);
  const trajectoryPreviewAbortRef = useRef<AbortController | null>(null);
  const trajectoryPreviewDebounceRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const normalizedApiHost = useMemo(() => normaliseApiHost(apiHost), [apiHost]);
  const motionStatusActive = useMemo(() => isMotionActive(motionStatus), [motionStatus]);
  const normalisedVisionHost = useMemo(
    () => normaliseVisionHost(visionHost),
    [visionHost],
  );

  useEffect(() => {
    plannerPointsRef.current = plannerPoints;
  }, [plannerPoints]);

  useEffect(() => {
    previewPlanRef.current = previewPlan;
  }, [previewPlan]);

  useEffect(() => {
    if (previewPlan) {
      setTrajectoryPlannerFailure(null);
    }
  }, [previewPlan]);

  useEffect(() => {
    if (!previewPlan) {
      return;
    }
    if (previewPlanMatchesWaypoints(previewPlan, plannerPoints)) {
      return;
    }
    setPreviewPlan(null);
  }, [plannerPoints, previewPlan]);

  const cancelTrajectoryPreviewRequest = useCallback(() => {
    trajectoryPreviewAbortRef.current?.abort();
    trajectoryPreviewAbortRef.current = null;
  }, []);

  const clearTrajectoryPreviewDebounce = useCallback(() => {
    if (!trajectoryPreviewDebounceRef.current) {
      return;
    }
    clearTimeout(trajectoryPreviewDebounceRef.current);
    trajectoryPreviewDebounceRef.current = null;
  }, []);

  useEffect(
    () => () => {
      clearTrajectoryPreviewDebounce();
    },
    [clearTrajectoryPreviewDebounce],
  );

  const getLatestTrajectoryDraftWaypoints = useCallback((): PoseWaypoint[] => {
    const current = plannerPointsRef.current;
    return current.length > 0 ? current : previewPlanRef.current?.waypoints ?? [];
  }, []);

  const updateSettings = useCallback(
    (partial: Partial<PersistedSettings>) => {
      setSettings((prev) => {
        return { ...prev, ...partial };
      });
    },
    [],
  );
  const [shellPaneLayout, setShellPaneLayout] = useState<ShellPaneLayout>(() =>
    clampShellPaneLayout(
      {
        leftPaneWidthPx: settings.leftPaneWidthPx,
        rightPaneWidthPx: settings.rightPaneWidthPx,
        timelineHeightPx: settings.timelineHeightPx,
      },
      typeof window !== "undefined" ? window.innerWidth : 1600,
      typeof window !== "undefined" ? window.innerHeight : 900,
    ),
  );
  const [activeShellDrag, setActiveShellDrag] = useState<ActiveShellDrag | null>(null);
  const shellPaneLayoutRef = useRef<ShellPaneLayout>(shellPaneLayout);
  const rightDockContainerRef = useRef<HTMLDivElement | null>(null);
  const [rightDockHeightPx, setRightDockHeightPx] = useState(0);
  const [programTreeHeightPx, setProgramTreeHeightPx] = useState(() =>
    clampProgramTreePaneHeight(settings.programTreeHeightPx, 0),
  );
  const programTreeHeightRef = useRef(programTreeHeightPx);
  const rightDockHeightRef = useRef(rightDockHeightPx);

  useEffect(() => {
    shellPaneLayoutRef.current = shellPaneLayout;
  }, [shellPaneLayout]);

  useEffect(() => {
    programTreeHeightRef.current = programTreeHeightPx;
  }, [programTreeHeightPx]);

  useEffect(() => {
    rightDockHeightRef.current = rightDockHeightPx;
  }, [rightDockHeightPx]);

  useEffect(() => {
    const node = rightDockContainerRef.current;
    if (!node) {
      return;
    }
    const updateHeight = () => {
      const measured = node.getBoundingClientRect().height;
      setRightDockHeightPx((prev) => (Math.abs(prev - measured) < 1 ? prev : measured));
    };
    updateHeight();
    const resizeObserver = new ResizeObserver(() => updateHeight());
    resizeObserver.observe(node);
    return () => resizeObserver.disconnect();
  }, []);

  useEffect(() => {
    const nextHeight = clampProgramTreePaneHeight(settings.programTreeHeightPx, rightDockHeightPx);
    setProgramTreeHeightPx((prev) => (Math.abs(prev - nextHeight) < 1 ? prev : nextHeight));
  }, [rightDockHeightPx, settings.programTreeHeightPx]);

  useEffect(() => {
    const handleViewportResize = () => {
      setShellPaneLayout((prev) => {
        const next = clampShellPaneLayout(prev, window.innerWidth, window.innerHeight);
        return sameShellPaneLayout(prev, next) ? prev : next;
      });
    };
    handleViewportResize();
    window.addEventListener("resize", handleViewportResize);
    return () => window.removeEventListener("resize", handleViewportResize);
  }, []);

  useEffect(() => {
    if (!activeShellDrag) {
      return;
    }
    const previousCursor = document.body.style.cursor;
    const previousUserSelect = document.body.style.userSelect;
    document.body.style.cursor =
      activeShellDrag.kind === "timeline" || activeShellDrag.kind === "rightDock"
        ? "row-resize"
        : "col-resize";
    document.body.style.userSelect = "none";

    const onPointerMove = (event: PointerEvent) => {
      const deltaX = event.clientX - activeShellDrag.startX;
      const deltaY = event.clientY - activeShellDrag.startY;
      const base = activeShellDrag.initialLayout;
      if (activeShellDrag.kind === "rightDock") {
        const nextProgramTreeHeight = clampProgramTreePaneHeight(
          activeShellDrag.initialProgramTreeHeightPx + deltaY,
          rightDockHeightRef.current,
        );
        programTreeHeightRef.current = nextProgramTreeHeight;
        setProgramTreeHeightPx(nextProgramTreeHeight);
        return;
      }
      const next =
        activeShellDrag.kind === "left"
          ? {
              ...base,
              leftPaneWidthPx: base.leftPaneWidthPx + deltaX,
            }
          : activeShellDrag.kind === "right"
            ? {
                ...base,
                rightPaneWidthPx: base.rightPaneWidthPx - deltaX,
              }
            : {
                ...base,
                timelineHeightPx: base.timelineHeightPx - deltaY,
              };
      const clamped = clampShellPaneLayout(next, window.innerWidth, window.innerHeight);
      shellPaneLayoutRef.current = clamped;
      setShellPaneLayout(clamped);
    };

    const stopDragging = () => {
      if (activeShellDrag.kind === "rightDock") {
        const finalProgramTreeHeight = clampProgramTreePaneHeight(
          programTreeHeightRef.current,
          rightDockHeightRef.current,
        );
        setProgramTreeHeightPx(finalProgramTreeHeight);
        updateSettings({ programTreeHeightPx: finalProgramTreeHeight });
        setActiveShellDrag(null);
        document.body.style.cursor = previousCursor;
        document.body.style.userSelect = previousUserSelect;
        return;
      }
      const finalLayout = clampShellPaneLayout(
        shellPaneLayoutRef.current,
        window.innerWidth,
        window.innerHeight,
      );
      setShellPaneLayout(finalLayout);
      updateSettings(finalLayout);
      setActiveShellDrag(null);
      document.body.style.cursor = previousCursor;
      document.body.style.userSelect = previousUserSelect;
    };

    window.addEventListener("pointermove", onPointerMove);
    window.addEventListener("pointerup", stopDragging);
    window.addEventListener("pointercancel", stopDragging);
    return () => {
      window.removeEventListener("pointermove", onPointerMove);
      window.removeEventListener("pointerup", stopDragging);
      window.removeEventListener("pointercancel", stopDragging);
      document.body.style.cursor = previousCursor;
      document.body.style.userSelect = previousUserSelect;
    };
  }, [activeShellDrag, updateSettings]);

  const startShellDrag = useCallback(
    (kind: ActiveShellDrag["kind"]) => (event: ReactPointerEvent<HTMLButtonElement>) => {
      event.preventDefault();
      setActiveShellDrag({
        kind,
        startX: event.clientX,
        startY: event.clientY,
        initialLayout: shellPaneLayoutRef.current,
        initialProgramTreeHeightPx: programTreeHeightRef.current,
      });
    },
    [],
  );

  const activeRobotId = runtimeConfigSnapshot?.active?.robot?.robot_id ?? null;
  const activeToolId = runtimeConfigSnapshot?.active?.tool?.active_tool_id ?? null;
  const runtimeMode: RuntimeExecutionMode | null = useMemo(() => {
    const active = runtimeConfigSnapshot?.active;
    if (!active) {
      return null;
    }
    if (active.mode?.sim || active.servo_backend?.effective_backend === "simulation") {
      return "simulate";
    }
    return "live";
  }, [runtimeConfigSnapshot]);
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
    const desiredSimMode = runtimePayload?.desired?.sim_mode;
    const desiredTool = runtimePayload?.desired?.active_tool_id;
    const desiredRtMaxRpm = runtimePayload?.desired?.overrides?.rt_max_rpm;
    const activeRtMaxRpm = runtimePayload?.active?.rtcore?.effective_max_rpm;
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
    setSelectedRuntimeMode((previous) => {
      if (typeof desiredSimMode === "boolean") {
        return desiredSimMode ? "simulate" : "live";
      }
      const activeMode = runtimePayload?.active?.mode?.sim ||
        runtimePayload?.active?.servo_backend?.effective_backend === "simulation";
      if (runtimePayload?.active) {
        return activeMode ? "simulate" : "live";
      }
      return previous;
    });
    setSelectedRtMaxRpmInput(() => {
      if (typeof desiredRtMaxRpm === "number" && Number.isFinite(desiredRtMaxRpm)) {
        return String(desiredRtMaxRpm);
      }
      if (typeof activeRtMaxRpm === "number" && Number.isFinite(activeRtMaxRpm)) {
        return String(activeRtMaxRpm);
      }
      return "6000";
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

  const patchRuntimeConfig = useCallback(async (patch: Record<string, unknown>) => {
    setIsRuntimeConfigBusy(true);
    try {
      const response = await fetch(`${normalizedApiHost}/info/runtime-config`, {
        method: "PATCH",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          ...patch,
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
      const snapshot = payload as RuntimeConfigSnapshot;
      setRuntimeConfigSnapshot(snapshot);
      setSelectedRuntimeMode(snapshot?.desired?.sim_mode ? "simulate" : "live");
      setRuntimeConfigError(null);
      return snapshot;
    } catch (err) {
      setRuntimeConfigError((err as Error).message);
      await fetchRuntimeConfigSnapshot().catch(() => undefined);
      return null;
    } finally {
      setIsRuntimeConfigBusy(false);
    }
  }, [fetchRuntimeConfigSnapshot, normalizedApiHost]);

  const handleApplyRuntimeConfig = useCallback(async () => {
    if (!selectedRobotName) {
      return;
    }
    const trimmedRtMaxRpm = selectedRtMaxRpmInput.trim();
    const parsedRtMaxRpm = trimmedRtMaxRpm.length === 0 ? 6000 : Number(trimmedRtMaxRpm);
    if (!Number.isFinite(parsedRtMaxRpm) || parsedRtMaxRpm < 0) {
      setRuntimeConfigError("RTCore max RPM must be a number greater than or equal to 0.");
      return;
    }
    await patchRuntimeConfig({
      robot: selectedRobotName,
      active_tool_id: selectedToolId || null,
      overrides: {
        rt_max_rpm: parsedRtMaxRpm,
      },
    });
  }, [patchRuntimeConfig, selectedRobotName, selectedRtMaxRpmInput, selectedToolId]);

  const switchRuntimeMode = useCallback(async (nextMode: RuntimeExecutionMode) => {
    const desiredMode: RuntimeExecutionMode | null =
      runtimeConfigSnapshot?.desired?.sim_mode === true
        ? "simulate"
        : runtimeConfigSnapshot?.desired?.sim_mode === false
          ? "live"
          : runtimeMode;
    if (desiredMode === nextMode && runtimeMode === nextMode) {
      setSelectedRuntimeMode(nextMode);
      return true;
    }

    const confirmed = window.confirm(
      nextMode === "simulate"
        ? "Switch controller into SIM mode now? The controller will stop active motion, wait for idle, and hot-swap to the simulation backend without restarting."
        : "Switch controller into LIVE mode now? The controller will stop active motion, wait for idle, and hot-swap back to the hardware backend without restarting.",
    );
    if (!confirmed) {
      return false;
    }

    setIsRuntimeConfigBusy(true);
    setRuntimeConfigError(null);
    try {
      const response = await fetch(`${normalizedApiHost}/control/runtime-mode`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode: nextMode }),
      });
      const payload = await response.json();
      if (!response.ok) {
        const message =
          typeof payload?.detail === "string"
            ? payload.detail
            : response.statusText || "Failed to switch runtime mode.";
        throw new Error(message);
      }
      const snapshot = payload as RuntimeConfigSnapshot;
      setRuntimeConfigSnapshot(snapshot);
      setSelectedRuntimeMode(snapshot?.desired?.sim_mode ? "simulate" : "live");
      setRuntimeConfigError(null);
      return true;
    } catch (err) {
      setRuntimeConfigError((err as Error).message);
      await fetchRuntimeConfigSnapshot().catch(() => undefined);
      return false;
    } finally {
      setIsRuntimeConfigBusy(false);
    }
  }, [fetchRuntimeConfigSnapshot, normalizedApiHost, runtimeConfigSnapshot?.desired?.sim_mode, runtimeMode]);

  const requestControllerRestart = useCallback(async (reason: string) => {
    setIsRestartingController(true);
    setRuntimeConfigError(null);
    try {
      const response = await fetch(`${normalizedApiHost}/control/restart-controller`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ reason }),
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
      return true;
    } catch (err) {
      setRuntimeConfigError((err as Error).message);
      return false;
    } finally {
      setIsRestartingController(false);
    }
  }, [fetchRuntimeConfigSnapshot, normalizedApiHost]);

  const handleSelectRuntimeMode = useCallback(async (nextMode: RuntimeExecutionMode) => {
    await switchRuntimeMode(nextMode);
  }, [switchRuntimeMode]);

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
    await requestControllerRestart("runtime-config-change");
  }, [requestControllerRestart, runtimeConfigSnapshot?.restart_required]);
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
  const programTreePlan = useMemo<PreviewPlan | null>(() => {
    if (previewPlan) {
      return previewPlan;
    }
    if (plannerPoints.length === 0 || weldDraft) {
      return null;
    }
    const draftName = trajectoryProgramName.trim() || selectedTrajectory.trim() || "Trajectory Draft";
    return {
      name: draftName,
      trajectory: {
        description: "Authored trajectory draft",
        moves: [],
      },
      pathPoints: plannerPoints,
      waypoints: plannerPoints,
      planningWarnings: [],
      useCache: false,
      isStale: true,
    };
  }, [plannerPoints, previewPlan, selectedTrajectory, trajectoryProgramName, weldDraft]);
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
        plan: programTreePlan,
        weldSegments: weldDraft?.segments ?? [],
        weldType: weldDraft?.weldType,
        viewMode: programTreeViewMode,
      }),
    [programTreePlan, weldDraft, programTreeViewMode],
  );
  const programNodeById = useMemo(
    () => indexProgramNodes(programTreeRoot),
    [programTreeRoot],
  );
  const programMoveNodeIdByIndex = useMemo(() => {
    const preferredByIndex = new Map<number, string>();
    const fallbackByIndex = new Map<number, string>();
    programNodeById.forEach((node, nodeId) => {
      const moveIndex = node.focus?.moveIndex;
      if (typeof moveIndex !== "number" || !Number.isInteger(moveIndex)) {
        return;
      }
      if (
        !fallbackByIndex.has(moveIndex) &&
        (node.type === "move" || node.type === "operationGroup")
      ) {
        fallbackByIndex.set(moveIndex, nodeId);
      }
      const isPreferredNode =
        programTreeViewMode === "grouped"
          ? node.type === "operationGroup" && node.id.startsWith("move_group_")
          : node.type === "move";
      if (isPreferredNode && !preferredByIndex.has(moveIndex)) {
        preferredByIndex.set(moveIndex, nodeId);
      }
    });
    const combined = new Map<number, string>();
    fallbackByIndex.forEach((nodeId, moveIndex) => combined.set(moveIndex, nodeId));
    preferredByIndex.forEach((nodeId, moveIndex) => combined.set(moveIndex, nodeId));
    return combined;
  }, [programNodeById, programTreeViewMode]);
  const selectedProgramNode = useMemo(
    () =>
      selectedProgramNodeId
        ? (programNodeById.get(selectedProgramNodeId) ?? null)
        : null,
    [selectedProgramNodeId, programNodeById],
  );
  const selectedProgramPathRange = selectedProgramNode?.focus?.pathRange ?? null;
  const selectedProgramWaypointIndices = selectedProgramNode?.focus?.waypointIndices ?? [];
  const previewMoves = Array.isArray(previewPlan?.trajectory?.moves)
    ? previewPlan.trajectory.moves
    : [];
  const waypointPathIndices = useMemo(
    () => buildOrderedWaypointPathIndices(visualPathPoints, visualWaypoints),
    [visualPathPoints, visualWaypoints],
  );
  const trajectoryMoveTimelineSelections = useMemo<TimelineSyntheticSelection[]>(() => {
    if (visualWaypoints.length < 2) {
      return [];
    }
    const pathPointCount = visualPathPoints.length;
    const moveCountMatchesWaypoints = previewMoves.length === visualWaypoints.length;
    return visualWaypoints.slice(0, -1).map((point, index) => {
      const nextPoint = visualWaypoints[index + 1];
      const relatedMoveIndex = moveCountMatchesWaypoints
        ? index + 1
        : index < previewMoves.length
          ? index
          : null;
      const rawStart = waypointPathIndices[index] ?? 0;
      const rawEnd = waypointPathIndices[index + 1] ?? rawStart;
      let start = Math.max(0, Math.min(pathPointCount - 1, rawStart));
      let end = Math.max(start, Math.min(pathPointCount - 1, rawEnd));
      if (pathPointCount >= 2 && start === end) {
        if (end < pathPointCount - 1) {
          end += 1;
        } else if (start > 0) {
          start -= 1;
        }
      }
      const pathRange = pathPointCount >= 2 ? { start, end } : null;
      return {
        id: `timeline_segment_${index}`,
        label: `M${index + 1}`,
        detail: `CP${index + 1} -> CP${index + 2} • ${describeWaypointMoveType(nextPoint.moveType)}${
          nextPoint.pauseAfterSeconds !== null && index < visualWaypoints.length - 2
            ? ` • ${formatPauseAfterSecondsLabel(nextPoint.pauseAfterSeconds)}`
            : ""
        }`,
        openPanel: "trajectory",
        pathRange,
        waypointIndices: [index, index + 1],
        relatedMoveIndex,
      };
    });
  }, [previewMoves.length, visualPathPoints.length, visualWaypoints, waypointPathIndices]);
  const selectedTimelineFocus = useMemo(
    () =>
      selectedTimelineSyntheticId
        ? (trajectoryMoveTimelineSelections.find((item) => item.id === selectedTimelineSyntheticId) ?? null)
        : null,
    [selectedTimelineSyntheticId, trajectoryMoveTimelineSelections],
  );
  const selectedProgramMoveTimelineFocus = useMemo(() => {
    const moveIndex = selectedProgramNode?.focus?.moveIndex;
    if (typeof moveIndex !== "number" || !Number.isInteger(moveIndex)) {
      return null;
    }
    return (
      trajectoryMoveTimelineSelections.find((item) => item.relatedMoveIndex === moveIndex) ?? null
    );
  }, [selectedProgramNode, trajectoryMoveTimelineSelections]);
  const activeSelectionPathRange =
    selectedTimelineFocus?.pathRange ??
    selectedProgramMoveTimelineFocus?.pathRange ??
    selectedProgramPathRange;
  const activeSelectionWaypointIndices =
    selectedTimelineFocus?.waypointIndices ??
    selectedProgramMoveTimelineFocus?.waypointIndices ??
    selectedProgramWaypointIndices;
  const selectedNodeFocusPoints = useMemo(() => {
    if (!selectedProgramNode && !selectedTimelineFocus && !selectedProgramMoveTimelineFocus) {
      return [];
    }
    const out: Point3[] = [];
    if (activeSelectionPathRange && visualPathPoints.length > 0) {
      const start = Math.max(0, Math.min(visualPathPoints.length - 1, activeSelectionPathRange.start));
      const end = Math.max(start, Math.min(visualPathPoints.length - 1, activeSelectionPathRange.end));
      out.push(...visualPathPoints.slice(start, end + 1));
    }
    if (Array.isArray(activeSelectionWaypointIndices) && activeSelectionWaypointIndices.length > 0) {
      activeSelectionWaypointIndices.forEach((index) => {
        if (index >= 0 && index < visualWaypoints.length) {
          out.push(visualWaypoints[index]);
        }
      });
    }
    if (out.length === 0 && selectedProgramNode?.type === "program") {
      return visualPathPoints.length > 0 ? visualPathPoints : visualWaypoints;
    }
    return out;
  }, [
    activeSelectionPathRange,
    activeSelectionWaypointIndices,
    selectedProgramMoveTimelineFocus,
    selectedProgramNode,
    selectedTimelineFocus,
    visualPathPoints,
    visualWaypoints,
  ]);
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
  const selectedProgramMove = useMemo<EditableProgramMove | null>(() => {
    if (weldDraft) {
      return null;
    }
    const directFocus = selectedProgramNode?.focus;
    let moveIndex =
      typeof directFocus?.moveIndex === "number" && Number.isInteger(directFocus.moveIndex)
        ? directFocus.moveIndex
        : null;
    let waypointIndex =
      Array.isArray(directFocus?.waypointIndices) && directFocus.waypointIndices.length > 1
        ? directFocus.waypointIndices[1]
        : null;
    if (
      moveIndex === null &&
      typeof selectedProgramMoveTimelineFocus?.relatedMoveIndex === "number" &&
      Number.isInteger(selectedProgramMoveTimelineFocus.relatedMoveIndex)
    ) {
      moveIndex = selectedProgramMoveTimelineFocus.relatedMoveIndex;
    }
    if (
      (typeof waypointIndex !== "number" || !Number.isInteger(waypointIndex)) &&
      Array.isArray(selectedProgramMoveTimelineFocus?.waypointIndices) &&
      selectedProgramMoveTimelineFocus.waypointIndices.length > 1
    ) {
      waypointIndex = selectedProgramMoveTimelineFocus.waypointIndices[1];
    }
    if (
      typeof moveIndex !== "number" ||
      !Number.isInteger(moveIndex) ||
      typeof waypointIndex !== "number" ||
      !Number.isInteger(waypointIndex) ||
      waypointIndex <= 0 ||
      waypointIndex >= treeEditableWaypoints.length
    ) {
      return null;
    }
    const startPoint = treeEditableWaypoints[waypointIndex - 1] ?? null;
    const point = treeEditableWaypoints[waypointIndex];
    if (!point) {
      return null;
    }
    const usesAngularSpeed = shouldUseAngularMoveSpeed(startPoint, point);
    const translationDistanceMm = poseWaypointTranslationDistanceM(startPoint, point) * 1000;
    const supportsAccelerationEdit = point.moveType === "linear" && !usesAngularSpeed;
    const supportsPauseEdit = waypointIndex < treeEditableWaypoints.length - 1;
    const resolvedAccelerationMmPerSec2 =
      supportsAccelerationEdit
        ? point.linearAccelerationMmPerSec2 ?? point.linearSpeedMmPerSec ?? null
        : null;
    return {
      moveIndex,
      waypointIndex,
      moveType: point.moveType,
      startWaypointIndex: waypointIndex - 1,
      endWaypointIndex: waypointIndex,
      moveDistanceMm: translationDistanceMm,
      speedMode: usesAngularSpeed ? ("angular" as const) : ("linear" as const),
      speedValue: usesAngularSpeed ? point.rotationSpeedDegPerSec : point.linearSpeedMmPerSec,
      speedUnitLabel: usesAngularSpeed ? "deg/s" : "mm/s",
      speedStep: usesAngularSpeed ? 0.5 : 1,
      hasCustomSpeed: usesAngularSpeed
        ? point.rotationSpeedDegPerSec !== null
        : point.linearSpeedMmPerSec !== null,
      supportsAccelerationEdit,
      accelerationValue: resolvedAccelerationMmPerSec2,
      accelerationUnitLabel: "mm/s^2",
      accelerationStep: 1,
      hasCustomAcceleration: supportsAccelerationEdit ? point.linearAccelerationMmPerSec2 !== null : false,
      supportsPauseEdit,
      pauseAfterSeconds: supportsPauseEdit ? point.pauseAfterSeconds : null,
      pauseStep: 0.1,
      hasCustomPause: supportsPauseEdit ? point.pauseAfterSeconds !== null : false,
    };
  }, [selectedProgramMoveTimelineFocus, selectedProgramNode, treeEditableWaypoints, weldDraft]);
  const selectedMoveEditorSelectionKey = `${selectedProgramNodeId ?? ""}|${selectedTimelineSyntheticId ?? ""}`;
  const lastSelectedProgramMoveRef = useRef<{
    selectionKey: string;
    move: EditableProgramMove;
  } | null>(null);
  useEffect(() => {
    if (selectedProgramMove) {
      lastSelectedProgramMoveRef.current = {
        selectionKey: selectedMoveEditorSelectionKey,
        move: selectedProgramMove,
      };
      return;
    }
    if (
      lastSelectedProgramMoveRef.current &&
      lastSelectedProgramMoveRef.current.selectionKey !== selectedMoveEditorSelectionKey
    ) {
      lastSelectedProgramMoveRef.current = null;
      return;
    }
    if (!selectedProgramNodeId && !selectedTimelineSyntheticId) {
      lastSelectedProgramMoveRef.current = null;
    }
  }, [
    selectedMoveEditorSelectionKey,
    selectedProgramMove,
    selectedProgramNodeId,
    selectedTimelineSyntheticId,
  ]);
  const visibleSelectedProgramMove =
    selectedProgramMove ??
    (lastSelectedProgramMoveRef.current?.selectionKey === selectedMoveEditorSelectionKey
      ? lastSelectedProgramMoveRef.current.move
      : null);
  const trajectoryDraftName = trajectoryProgramName.trim() || selectedTrajectory.trim() || "trajectory_program";
  const hasTrajectoryUnsavedChanges = useMemo(
    () => !weldDraft && !poseWaypointListsMatch(plannerPoints, savedTrajectoryWaypoints),
    [plannerPoints, savedTrajectoryWaypoints, weldDraft],
  );
  const canTreeWaypointValueEdit = !isPlanningWeld && !isRunningPreview && !motionStatusActive;
  const minRemainingWaypoints = weldDraft ? 2 : 1;
  const canTreeWaypointAdd = canTreeWaypointValueEdit;
  const canTreeWaypointRemove = Boolean(
      selectedProgramControlPoint &&
      treeEditableWaypoints.length > minRemainingWaypoints &&
      canTreeWaypointValueEdit,
  );
  const canTreeWaypointMoveUp = Boolean(
    selectedProgramControlPoint &&
      selectedProgramControlPoint.index > 0 &&
      canTreeWaypointValueEdit,
  );
  const canTreeWaypointMoveDown = Boolean(
    selectedProgramControlPoint &&
      selectedProgramControlPoint.index < treeEditableWaypoints.length - 1 &&
      canTreeWaypointValueEdit,
  );
  const canTreeWaypointApply =
    canTreeWaypointValueEdit &&
    (weldDraft ? weldEditableWaypoints.length > 1 : treeEditableWaypoints.length > 0);
  const canUndoTrajectoryDraftEdit = Boolean(
    !weldDraft && trajectoryDraftHistory.length > 0 && canTreeWaypointValueEdit,
  );
  const canRevertTrajectoryDraft = Boolean(
    !weldDraft && hasTrajectoryUnsavedChanges && canTreeWaypointValueEdit,
  );
  const canSaveTrajectoryDraft = Boolean(
    !weldDraft &&
      plannerPoints.length > 0 &&
      !isSavingTrajectoryProgram &&
      !isLoadingSavedTrajectory &&
      !isRunningPreview &&
      !motionStatusActive,
  );
  const activePanelTitle = activePanel === "step"
    ? "STEP Import"
    : activePanel === "trajectory"
      ? "Trajectory Authoring"
      : activePanel === "tools"
        ? "Tool Library"
        : activePanel === "weld"
          ? "Weld Authoring"
          : activePanel === "telemetry"
            ? "Live Charts"
            : "Workspace";
  const selectedProgramNodeTypeLabel = selectedProgramNode
    ? selectedProgramNode.type
        .replace(/([A-Z])/g, " $1")
        .replace(/^./, (value) => value.toUpperCase())
    : null;
  const currentPlayheadLabel =
    selectedTimelineFocus?.label ??
    selectedProgramMoveTimelineFocus?.label ??
    selectedProgramNode?.label ??
    "No timeline block selected";
  const currentPlayheadDetail = selectedTimelineFocus?.detail
    ?? selectedProgramMoveTimelineFocus?.detail
    ?? selectedProgramNode?.subtitle
    ?? (selectedProgramNodeTypeLabel ? `Selected ${selectedProgramNodeTypeLabel}` : activePanelTitle);
  const timelineLanes = useMemo<ProgramTimelineLane[]>(() => {
    const lanes: ProgramTimelineLane[] = [];
    if (visualWaypoints.length > 0) {
      const trajectoryItems: ProgramTimelineItem[] = [];
      visualWaypoints.forEach((point, index) => {
        const nodeId = `control_point_${index}`;
        trajectoryItems.push({
          id: nodeId,
          label: `CP${index + 1}`,
          metaLabel: describeTrajectoryControlPointKind(point.moveType),
          subtitle: `${point.x.toFixed(3)}, ${point.y.toFixed(3)}, ${point.z.toFixed(3)} m`,
          tone: "cyan",
          active:
            selectedProgramNodeId === nodeId ||
            selectedProgramWaypointIndices.includes(index),
          disabled: !programNodeById.has(nodeId),
        });
        if (index < trajectoryMoveTimelineSelections.length) {
          const moveSelection = trajectoryMoveTimelineSelections[index];
          trajectoryItems.push({
            id: moveSelection.id,
            label: moveSelection.label,
            metaLabel:
              visualWaypoints[index + 1]
                ? describeWaypointMoveType(visualWaypoints[index + 1].moveType)
                : "Move",
            subtitle: moveSelection.detail,
            tone: "amber",
            active:
              selectedTimelineSyntheticId === moveSelection.id ||
              selectedProgramMoveTimelineFocus?.id === moveSelection.id,
            disabled: !moveSelection.pathRange,
          });
        }
      });
      lanes.push({
        id: "trajectory-sequence",
        title: "Trajectory Sequence",
        subtitle: `${visualWaypoints.length} waypoint(s) • ${trajectoryMoveTimelineSelections.length} move segment(s) between them`,
        tone: "slate",
        items: trajectoryItems,
      });
    }
    if (weldDraft?.segments && weldDraft.segments.length > 0) {
      lanes.push({
        id: "weld-segments",
        title: "Weld Sections",
        subtitle: `${weldDraft.segments.length} selected weld segment(s)`,
        tone: "emerald",
        items: weldDraft.segments.map((segment, index) => {
          const nodeId = findWeldProgramNodeIdByEdge(programNodeById, segment.edgeId);
          return {
            id: nodeId ?? `weld-segment-${index}`,
            label: `Segment ${index + 1}`,
            metaLabel: "Weld Segment",
            subtitle: `${segment.weldType} • ${segment.edgeId}`,
            tone: "emerald" as const,
            active:
              selectedProgramNode?.focus?.weldSegmentEdgeId === segment.edgeId ||
              activeWeldSegment?.edgeId === segment.edgeId,
            disabled: !nodeId,
          };
        }),
      });
    }
    return lanes;
  }, [
    activeWeldSegment?.edgeId,
    programNodeById,
    selectedProgramMoveTimelineFocus,
    selectedProgramNode,
    selectedProgramNodeId,
    selectedTimelineSyntheticId,
    selectedProgramWaypointIndices,
    visualWaypoints,
    weldDraft,
    trajectoryMoveTimelineSelections,
  ]);
  const { leftPaneWidthPx, rightPaneWidthPx, timelineHeightPx } = shellPaneLayout;
  const shellGridStyle = {
    gridTemplateRows: `minmax(0,1fr) ${SHELL_SPLITTER_SIZE_PX}px ${timelineHeightPx}px`,
    gridTemplateColumns: `minmax(0,1fr) ${SHELL_SPLITTER_SIZE_PX}px ${rightPaneWidthPx}px`,
  } as const;
  const workspaceColumnsStyle = {
    gridTemplateColumns: `${leftPaneWidthPx}px ${SHELL_SPLITTER_SIZE_PX}px minmax(0,1fr)`,
  } as const;
  const rightDockStyle = {
    gridTemplateRows: `${clampProgramTreePaneHeight(programTreeHeightPx, rightDockHeightPx)}px ${SHELL_SPLITTER_SIZE_PX}px minmax(0,1fr)`,
  } as const;
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
    cancelTrajectoryPreviewRequest();
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
    setTrajectoryProgramName("trajectory_program");
    setIsSavingTrajectoryProgram(false);
    setMotionStatus(null);
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
  }, [cancelTrajectoryPreviewRequest]);

  const handleMessage = useCallback((payload: string) => {
    let joints: number[] | undefined;
    let gripper: number | undefined;
    let servos: Record<string, ServoSample> | undefined;
    let parsedAlerts: Alert[] | undefined;
    let driveFaultsValue: DriveFaultSnapshot | null = null;
    let weldActiveValue: boolean | undefined;
    let weldTypeValue: string | undefined;
    let sourceTimeSec: number | undefined;
    let commsValue: Record<string, unknown> | undefined;
    let motionStatusValue: MotionStatusResponse | null = null;

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
            if (typeof s.pos_counts === "number") sample.pos_counts = s.pos_counts;
            if (typeof s.torque_raw === "number") sample.torque_raw = s.torque_raw;
            if (typeof s.statusword === "number") sample.statusword = s.statusword;
            if (typeof s.statusword_hex === "string") sample.statusword_hex = s.statusword_hex;
            if (typeof s.error_code === "number") sample.error_code = s.error_code;
            if (typeof s.error_code_hex === "string") sample.error_code_hex = s.error_code_hex;
            if (typeof s.mode_display === "number") sample.mode_display = s.mode_display;
            if (typeof s.mode_display_name === "string") sample.mode_display_name = s.mode_display_name;
            if (typeof s.ds402_state === "string") sample.ds402_state = s.ds402_state;
            if (typeof s.ds402_state_code === "number") sample.ds402_state_code = s.ds402_state_code;
            if (typeof s.di_bits === "number") sample.di_bits = s.di_bits;
            if (typeof s.di_bits_hex === "string") sample.di_bits_hex = s.di_bits_hex;
            if (typeof s.axis_fault_flags === "number") sample.axis_fault_flags = s.axis_fault_flags;
            if (typeof s.brake_state === "number") sample.brake_state = s.brake_state;
            if (typeof s.logical_joint === "number") sample.logical_joint = s.logical_joint;
            if (typeof s.axis_index === "number") sample.axis_index = s.axis_index;
            if (Array.isArray(s.status_names)) {
              sample.status_names = s.status_names.filter((value): value is string => typeof value === "string");
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
        if (maybeObj.drive_faults && typeof maybeObj.drive_faults === "object") {
          driveFaultsValue = maybeObj.drive_faults as DriveFaultSnapshot;
        }
        if (typeof maybeObj.weld_active === "boolean") {
          weldActiveValue = maybeObj.weld_active;
        }
        if (typeof maybeObj.weld_type === "string" && maybeObj.weld_type.trim()) {
          weldTypeValue = maybeObj.weld_type.trim();
        }
        if (maybeObj.comms && typeof maybeObj.comms === "object") {
          commsValue = maybeObj.comms as Record<string, unknown>;
        }
        if (maybeObj.motion_status && typeof maybeObj.motion_status === "object") {
          motionStatusValue = maybeObj.motion_status as MotionStatusResponse;
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
      drive_faults: driveFaultsValue,
      weld_active: weldActiveValue,
      weld_type: weldTypeValue,
      comms: commsValue,
      motion_status: motionStatusValue,
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

    setLatest((prev) => ({
      ...next,
      // The monitor stream only includes drive fault snapshots on some packets.
      // Preserve the last known drive-power state between those updates so the
      // power controls do not flicker back to an indeterminate/disarmed UI.
      drive_faults: next.drive_faults ?? prev?.drive_faults ?? null,
    }));
    if (motionStatusValue) {
      setMotionStatus(motionStatusValue);
    }
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

  const handleFallbackJointFeedback = useCallback(
    (anglesDeg: number[], gripperDeg?: number) => {
      const nextJoints = anglesDeg
        .map((value) => Number(value))
        .filter((value) => Number.isFinite(value))
        .map((value) => degToRad(value));
      if (nextJoints.length === 0) {
        return;
      }

      const lastTelemetrySec = lastTelemetrySourceTimeRef.current;
      const nowSec = Date.now() / 1000;
      // Only fall back when the monitor stream is visibly late; otherwise let
      // the primary SSE telemetry remain the single source of truth.
      if (lastTelemetrySec !== null && nowSec - lastTelemetrySec < LIVE_JOINT_FALLBACK_MAX_AGE_S) {
        return;
      }

      const nextGripper =
        typeof gripperDeg === "number" && Number.isFinite(gripperDeg)
          ? degToRad(gripperDeg)
          : undefined;

      lastAcceptedJointsRef.current = nextJoints.slice();
      setLatest((prev) => {
        if (
          areJointArraysClose(prev?.joints, nextJoints) &&
          (nextGripper === undefined || Math.abs((prev?.gripper ?? 0) - nextGripper) <= 1e-5)
        ) {
          return prev;
        }
        return {
          timestamp: Date.now(),
          raw: prev?.raw ?? "fallback:/info/joints",
          joints: nextJoints,
          gripper: nextGripper ?? prev?.gripper,
          servos: prev?.servos,
          alerts: prev?.alerts,
          drive_faults: prev?.drive_faults ?? null,
          weld_active: prev?.weld_active,
          weld_type: prev?.weld_type,
        };
      });
    },
    [],
  );

  const toggleConnection = useCallback(() => {
    if (isConnected) {
      disconnect();
      return;
    }
    connect();
  }, [connect, disconnect, isConnected]);

  useEffect(() => {
    const timer = window.setInterval(() => {
      setLiveClockMs(Date.now());
    }, 250);
    return () => {
      window.clearInterval(timer);
    };
  }, []);

  const toggleVision = useCallback(() => {
    if (isVisionActive) {
      setIsVisionActive(false);
      setVisionError(null);
      return;
    }
    setVisionError(null);
    setIsVisionActive(true);
  }, [isVisionActive]);
  const showStage3d = useCallback(() => {
    setVisionError(null);
    setIsVisualizerEnabled(true);
    setIsVisionActive(false);
  }, []);
  const showVisionStage = useCallback(() => {
    setVisionError(null);
    setIsVisionActive(true);
  }, []);

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
    async (
      waypoints: PoseWaypoint[],
      options?: {
        persistProgramName?: string;
        persistMetadata?: Record<string, unknown> | null;
      },
    ): Promise<PreviewPlan | null> => {
      cancelTrajectoryPreviewRequest();
      if (waypoints.length === 0) {
        setIsPlanLoading(false);
        setPreviewPlan(null);
        setTrajectoryPlannerFailure(null);
        setPlannerPoints([]);
        setWeldPreviewGhostJoints(null);
        setWeldPreviewCacheReady(false);
        return null;
      }
      const abortController = new AbortController();
      trajectoryPreviewAbortRef.current = abortController;
      setIsPlanLoading(true);
      setError(null);
      setTrajectoryPlannerFailure(null);
      try {
        const response = await fetch(`${normalizedApiHost}/trajectory/plan-points`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          signal: abortController.signal,
          body: JSON.stringify({ waypoints: encodePoseWaypointsForApi(waypoints) }),
        });
        if (!response.ok) {
          const parsedError = await readApiErrorResponse(response);
          setTrajectoryPlannerFailure(parsedError);
          throw new Error(parsedError.message || `Plan request failed (${response.status})`);
        }
        const data = await response.json();
        const { plan } = previewFromPlannerPayload(data);
        const mergedWaypoints = mergePoseWaypointMotionMetadata(waypoints, plan.waypoints);
        const authoredPlan: PreviewPlan = {
          ...plan,
          // Keep authored control points as the source of truth for pose/order, but
          // merge controller-computed motion metadata like resolved move speeds back in.
          waypoints: mergedWaypoints.map((waypoint) => ({ ...waypoint })),
        };
        if (options?.persistProgramName) {
          const persistResponse = await fetch(`${normalizedApiHost}/robot-program/save`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            signal: abortController.signal,
            body: JSON.stringify({
              kind: "trajectory",
              name: options.persistProgramName,
              authoring: {
                waypoints: encodePoseWaypointsForApi(mergedWaypoints),
                metadata: options.persistMetadata ?? {},
              },
              planned_trajectory: authoredPlan,
            }),
          });
          if (!persistResponse.ok) {
            const parsedError = await readApiErrorResponse(persistResponse);
            throw new Error(
              parsedError.message || `Failed to overwrite saved trajectory (${persistResponse.status})`,
            );
          }
        }
        if (trajectoryPreviewAbortRef.current !== abortController) {
          return null;
        }
        setPreviewPlan(authoredPlan);
        setPlannerPoints((current) =>
          poseWaypointListsMatch(current, mergedWaypoints) ? current : clonePoseWaypointList(mergedWaypoints),
        );
        setTrajectoryPlannerFailure(null);
        setWeldPreviewGhostJoints(null);
        setWeldPreviewCacheReady(false);
        return authoredPlan;
      } catch (err) {
        if (abortController.signal.aborted) {
          return null;
        }
        setError(`Failed to plan trajectory: ${(err as Error).message}`);
        setWeldPreviewGhostJoints(null);
        return null;
      } finally {
        if (trajectoryPreviewAbortRef.current === abortController) {
          trajectoryPreviewAbortRef.current = null;
          setIsPlanLoading(false);
        }
      }
    },
    [cancelTrajectoryPreviewRequest, normalizedApiHost],
  );

  const queueTrajectoryPreview = useCallback(
    (waypoints: PoseWaypoint[], delayMs = 220) => {
      clearTrajectoryPreviewDebounce();
      const nextWaypoints = clonePoseWaypointList(waypoints);
      trajectoryPreviewDebounceRef.current = setTimeout(() => {
        trajectoryPreviewDebounceRef.current = null;
        void requestPlannerPreview(nextWaypoints);
      }, delayMs);
    },
    [clearTrajectoryPreviewDebounce, requestPlannerPreview],
  );

  const ensureTrajectoryPreview = useCallback(
    async (waypoints: PoseWaypoint[]): Promise<PreviewPlan | null> => {
      clearTrajectoryPreviewDebounce();
      if (waypoints.length === 0) {
        cancelTrajectoryPreviewRequest();
        setPreviewPlan(null);
        setTrajectoryPlannerFailure(null);
        return null;
      }
      const currentPreview = previewPlanRef.current;
      if (currentPreview && previewPlanMatchesWaypoints(currentPreview, waypoints) && !currentPreview.isStale) {
        return currentPreview;
      }
      return requestPlannerPreview(waypoints);
    },
    [cancelTrajectoryPreviewRequest, clearTrajectoryPreviewDebounce, requestPlannerPreview],
  );

  const commitTrajectoryDraftEdit = useCallback(
    (
      nextWaypointsInput: PoseWaypoint[] | ((current: PoseWaypoint[]) => PoseWaypoint[]),
      options?: {
        previewMode?: "debounced" | "immediate" | "none";
        recordHistory?: boolean;
      },
    ): PoseWaypoint[] => {
      const currentWaypoints = clonePoseWaypointList(getLatestTrajectoryDraftWaypoints());
      const resolvedWaypoints =
        typeof nextWaypointsInput === "function"
          ? nextWaypointsInput(currentWaypoints)
          : nextWaypointsInput;
      const nextWaypoints = clonePoseWaypointList(resolvedWaypoints);
      if (poseWaypointListsMatch(currentWaypoints, nextWaypoints)) {
        return currentWaypoints;
      }
      if (options?.recordHistory !== false) {
        setTrajectoryDraftHistory((currentHistory) => [
          ...currentHistory.slice(-49),
          currentWaypoints,
        ]);
      }
      setError(null);
      setTrajectoryPlannerFailure(null);
      setPlannerPoints(nextWaypoints);
      if ((options?.previewMode ?? "debounced") === "immediate") {
        clearTrajectoryPreviewDebounce();
        void requestPlannerPreview(nextWaypoints);
      } else if ((options?.previewMode ?? "debounced") === "debounced") {
        queueTrajectoryPreview(nextWaypoints);
      }
      return nextWaypoints;
    },
    [
      clearTrajectoryPreviewDebounce,
      getLatestTrajectoryDraftWaypoints,
      queueTrajectoryPreview,
      requestPlannerPreview,
    ],
  );

  const markTrajectoryDraftSaved = useCallback((waypoints: PoseWaypoint[]) => {
    setSavedTrajectoryWaypoints(clonePoseWaypointList(waypoints));
    setTrajectoryDraftHistory([]);
  }, []);

  const handlePlanToggle = useCallback(() => {
    if (isPlanLoading || isRunningPreview || motionStatusActive) {
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
  }, [isPlanLoading, isRunningPreview, motionStatusActive, previewPlan]);

  const fetchLiveTrajectoryPoseWaypoint = useCallback(
    async (moveType: WaypointMoveType): Promise<PoseWaypoint> => {
      const response = await fetch(`${normalizedApiHost}/info/pose`);
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || `Pose request failed (${response.status})`);
      }
      const pose = (await response.json()) as {
        position_m?: { x?: number; y?: number; z?: number };
        orientation_euler_deg?: { roll?: number; pitch?: number; yaw?: number };
      };
      return buildPoseWaypoint(
        {
          x: Number(pose.position_m?.x ?? 0),
          y: Number(pose.position_m?.y ?? 0),
          z: Number(pose.position_m?.z ?? 0),
        },
        {
          rollDeg: Number(pose.orientation_euler_deg?.roll ?? null),
          pitchDeg: Number(pose.orientation_euler_deg?.pitch ?? null),
          yawDeg: Number(pose.orientation_euler_deg?.yaw ?? null),
          moveType,
        },
      );
    },
    [normalizedApiHost],
  );

  const handleAddTrajectoryWaypoint = useCallback(async () => {
    if (isPlanLoading || isRunningPreview || motionStatusActive) {
      return;
    }
    setError(null);
    const basePoints = getLatestTrajectoryDraftWaypoints();
    let nextPoints: PoseWaypoint[];
    try {
      const nextWaypoint = await fetchLiveTrajectoryPoseWaypoint("joint");
      nextPoints = [...basePoints, nextWaypoint];
    } catch (err) {
      setError(`Failed to capture current joint waypoint: ${(err as Error).message}`);
      return;
    }
    setIsPlanning(true);
    commitTrajectoryDraftEdit(nextPoints, { previewMode: "immediate" });
  }, [
    commitTrajectoryDraftEdit,
    getLatestTrajectoryDraftWaypoints,
    isPlanLoading,
    isRunningPreview,
    motionStatusActive,
    fetchLiveTrajectoryPoseWaypoint,
  ]);

  const handleAddTrajectoryHomeWaypoint = useCallback(async () => {
    if (isPlanLoading || isRunningPreview || motionStatusActive) {
      return;
    }
    const basePoints = getLatestTrajectoryDraftWaypoints();
    let nextPoints: PoseWaypoint[];
    if (basePoints.length === 0) {
      try {
        nextPoints = [await fetchLiveTrajectoryPoseWaypoint("home")];
      } catch (err) {
        setError(`Failed to seed home waypoint from current pose: ${(err as Error).message}`);
        return;
      }
    } else {
      const source = basePoints[basePoints.length - 1];
      nextPoints = [...basePoints, { ...source, moveType: "home" }];
      updateSettings({ selectedProgramNodeId: `control_point_${nextPoints.length - 1}` });
    }
    setIsPlanning(true);
    commitTrajectoryDraftEdit(nextPoints, { previewMode: "immediate" });
  }, [
    commitTrajectoryDraftEdit,
    getLatestTrajectoryDraftWaypoints,
    isPlanLoading,
    isRunningPreview,
    motionStatusActive,
    fetchLiveTrajectoryPoseWaypoint,
    updateSettings,
  ]);

  const handlePointSelected = useCallback(
    async (point: Point3) => {
      if (!isPlanning || isPlanLoading) {
        return;
      }
      const currentPoints = getLatestTrajectoryDraftWaypoints();
      const lastOrientationSource = currentPoints[currentPoints.length - 1] ?? null;
      const nextPoints = [
        ...currentPoints,
        buildPoseWaypoint(point, {
          rollDeg: lastOrientationSource?.rollDeg ?? null,
          pitchDeg: lastOrientationSource?.pitchDeg ?? null,
          yawDeg: lastOrientationSource?.yawDeg ?? null,
          moveType: "linear",
        }),
      ];
      commitTrajectoryDraftEdit(nextPoints, { previewMode: "immediate" });
    },
    [commitTrajectoryDraftEdit, getLatestTrajectoryDraftWaypoints, isPlanning, isPlanLoading],
  );

  const handleUndoPoint = useCallback(async () => {
    const currentPoints = getLatestTrajectoryDraftWaypoints();
    if (currentPoints.length === 0 || isPlanLoading) {
      return;
    }
    const nextPoints = currentPoints.slice(0, -1);
    if (nextPoints.length === 0) {
      clearTrajectoryPreviewDebounce();
      cancelTrajectoryPreviewRequest();
      setPreviewPlan(null);
      setTrajectoryPlannerFailure(null);
      setPlannerPoints([]);
      setWeldPreviewGhostJoints(null);
      return;
    }
    commitTrajectoryDraftEdit(nextPoints, { previewMode: "immediate" });
  }, [
    cancelTrajectoryPreviewRequest,
    clearTrajectoryPreviewDebounce,
    commitTrajectoryDraftEdit,
    getLatestTrajectoryDraftWaypoints,
    isPlanLoading,
  ]);

  const handleClearPreview = useCallback(() => {
    clearTrajectoryPreviewDebounce();
    cancelTrajectoryPreviewRequest();
    setError(null);
    setPreviewPlan(null);
    setTrajectoryPlannerFailure(null);
    setPlannerPoints([]);
    setSavedTrajectoryWaypoints([]);
    setTrajectoryDraftHistory([]);
    setWeldEditableWaypoints([]);
    setWeldPreviewGhostJoints(null);
    setWeldPreviewCacheReady(false);
    setIsPlanning(false);
    setIsPlanLoading(false);
  }, [cancelTrajectoryPreviewRequest, clearTrajectoryPreviewDebounce]);

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

  const handleRunPreview = useCallback(async (executionMode: RuntimeExecutionMode) => {
    if (!previewPlan || isPlanLoading || isRunningPreview || motionStatusActive) {
      return;
    }
    setIsRunningPreview(true);
    setError(null);
      setTrajectoryPlannerFailure(null);
    try {
      if (!previewPlanMatchesWaypoints(previewPlan, plannerPoints)) {
        throw new Error("Preview path is out of sync with the current waypoints. Regenerate before running.");
      }
      if (previewPlan.isStale) {
        throw new Error("Saved path is stale. Regenerate the path from the loaded waypoints before running.");
      }
      const trajectoryMeta = previewPlan.trajectory as unknown as Record<string, unknown>;
      const isWeldPreview =
        Boolean(trajectoryMeta) &&
        typeof trajectoryMeta.weld === "object" &&
        trajectoryMeta.weld !== null;
      setShowTrajectoryRunPreparationOverlay(executionMode === "live" && !isWeldPreview);
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
        body: JSON.stringify({
          name: runName,
          use_cache: useCache,
          execution_mode: executionMode,
          loop_override: trajectoryLoopEnabled,
        }),
      });
      if (!response.ok) {
        const parsedError = await readApiErrorResponse(response);
        if (parsedError.plannerDiagnostics) {
          setTrajectoryPlannerFailure(parsedError);
        }
        throw new Error(parsedError.message || `Run request failed (${response.status})`);
      }
      const payload = (await response.json()) as MotionStatusResponse;
      setMotionStatus(payload);
    } catch (err) {
      setError(`Failed to ${executionMode === "simulate" ? "simulate" : "run"} trajectory: ${(err as Error).message}`);
    } finally {
      setShowTrajectoryRunPreparationOverlay(false);
      setIsRunningPreview(false);
    }
  }, [
    plannerPoints,
    previewPlan,
    isPlanLoading,
    isRunningPreview,
    motionStatusActive,
    normalizedApiHost,
    trajectoryLoopEnabled,
    weldDraft,
    requestWeldPreview,
  ]);
  const handleSimulatePreview = useCallback(() => {
    void handleRunPreview("simulate");
  }, [handleRunPreview]);
  const handleRunPreviewLive = useCallback(() => {
    void handleRunPreview("live");
  }, [handleRunPreview]);

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
    (
      index: number,
      axis: "x" | "y" | "z" | "rollDeg" | "pitchDeg" | "yawDeg",
      value: number,
    ) => {
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
      commitTrajectoryDraftEdit(
        (current) =>
          current.map((point, pointIndex) =>
            pointIndex === index
              ? {
                  ...point,
                  [axis]: Number.isFinite(value) ? value : point[axis],
                }
              : point,
          ),
        { previewMode: "debounced" },
      );
    },
    [commitTrajectoryDraftEdit, weldDraft],
  );

  const handleTreeWaypointMoveTypeChange = useCallback(
    (index: number, moveType: WaypointMoveType) => {
      if (weldDraft) {
        return;
      }
      commitTrajectoryDraftEdit(
        (current) =>
          current.map((point, pointIndex) =>
            pointIndex === index
              ? {
                  ...point,
                  moveType,
                  ...(moveType === "linear"
                    ? {}
                    : {
                        linearSpeedMmPerSec: null,
                        linearAccelerationMmPerSec2: null,
                      }),
                }
              : point,
          ),
        { previewMode: "debounced" },
      );
    },
    [commitTrajectoryDraftEdit, weldDraft],
  );

  const handleTreeWaypointSpeedChange = useCallback(
    (index: number, speedMode: "linear" | "angular", value: number) => {
      if (weldDraft) {
        return;
      }
      const normalized = Number(value);
      if (!Number.isFinite(normalized) || normalized <= 0) {
        return;
      }
      commitTrajectoryDraftEdit(
        (current) =>
          current.map((point, pointIndex) =>
            pointIndex === index
              ? {
                  ...point,
                  ...(speedMode === "angular"
                    ? { rotationSpeedDegPerSec: normalized }
                    : { linearSpeedMmPerSec: normalized }),
                }
              : point,
          ),
        { previewMode: "debounced" },
      );
    },
    [commitTrajectoryDraftEdit, weldDraft],
  );

  const handleTreeWaypointSpeedReset = useCallback(
    (index: number, speedMode: "linear" | "angular") => {
      if (weldDraft) {
        return;
      }
      commitTrajectoryDraftEdit(
        (current) =>
          current.map((point, pointIndex) =>
            pointIndex === index
              ? {
                  ...point,
                  ...(speedMode === "angular"
                    ? { rotationSpeedDegPerSec: null }
                    : { linearSpeedMmPerSec: null }),
                }
              : point,
          ),
        { previewMode: "debounced" },
      );
    },
    [commitTrajectoryDraftEdit, weldDraft],
  );

  const handleTreeWaypointAccelerationChange = useCallback(
    (index: number, value: number) => {
      if (weldDraft) {
        return;
      }
      const normalized = Number(value);
      if (!Number.isFinite(normalized) || normalized <= 0) {
        return;
      }
      commitTrajectoryDraftEdit(
        (current) =>
          current.map((point, pointIndex) =>
            pointIndex === index
              ? {
                  ...point,
                  linearAccelerationMmPerSec2: normalized,
                }
              : point,
          ),
        { previewMode: "debounced" },
      );
    },
    [commitTrajectoryDraftEdit, weldDraft],
  );

  const handleTreeWaypointAccelerationReset = useCallback(
    (index: number) => {
      if (weldDraft) {
        return;
      }
      commitTrajectoryDraftEdit(
        (current) =>
          current.map((point, pointIndex) =>
            pointIndex === index
              ? {
                  ...point,
                  linearAccelerationMmPerSec2: null,
                }
              : point,
          ),
        { previewMode: "debounced" },
      );
    },
    [commitTrajectoryDraftEdit, weldDraft],
  );

  const handleTreeWaypointPauseChange = useCallback(
    (index: number, value: number) => {
      if (weldDraft) {
        return;
      }
      const normalized = Number(value);
      if (!Number.isFinite(normalized) || normalized <= 0) {
        return;
      }
      commitTrajectoryDraftEdit(
        (current) =>
          current.map((point, pointIndex) =>
            pointIndex === index
              ? {
                  ...point,
                  pauseAfterSeconds: normalized,
                }
              : point,
          ),
        { previewMode: "debounced" },
      );
    },
    [commitTrajectoryDraftEdit, weldDraft],
  );

  const handleTreeWaypointPauseReset = useCallback(
    (index: number) => {
      if (weldDraft) {
        return;
      }
      commitTrajectoryDraftEdit(
        (current) =>
          current.map((point, pointIndex) =>
            pointIndex === index
              ? {
                  ...point,
                  pauseAfterSeconds: null,
                }
              : point,
          ),
        { previewMode: "debounced" },
      );
    },
    [commitTrajectoryDraftEdit, weldDraft],
  );

  const handleTreeAddWaypoint = useCallback(() => {
    if (weldDraft) {
      setWeldEditableWaypoints((current) => {
        if (current.length === 0) {
          return [
            buildPoseWaypoint({ x: 0, y: 0, z: 0 }),
            buildPoseWaypoint({ x: 0.02, y: 0, z: 0 }),
          ];
        }
        const insertFromIndex =
          selectedProgramControlPointIndex !== null
            ? Math.max(0, Math.min(current.length - 1, selectedProgramControlPointIndex))
            : current.length - 1;
        const source = current[insertFromIndex] ?? current[current.length - 1];
        return [
          ...current.slice(0, insertFromIndex + 1),
          { ...source },
          ...current.slice(insertFromIndex + 1),
        ];
      });
      setWeldPreviewCacheReady(false);
      return;
    }
    const currentPoints = getLatestTrajectoryDraftWaypoints();
    const nextPoints: PoseWaypoint[] =
      currentPoints.length === 0
        ? [buildPoseWaypoint({ x: 0, y: 0, z: 0 }, { moveType: "joint" })]
        : [
            ...currentPoints,
            {
              ...currentPoints[currentPoints.length - 1],
              moveType: "joint" as const,
              linearSpeedMmPerSec: null,
              linearAccelerationMmPerSec2: null,
              rotationSpeedDegPerSec: null,
              pauseAfterSeconds: null,
            },
          ];
    commitTrajectoryDraftEdit(nextPoints, { previewMode: "immediate" });
    updateSettings({ selectedProgramNodeId: `control_point_${nextPoints.length - 1}` });
  }, [
    commitTrajectoryDraftEdit,
    getLatestTrajectoryDraftWaypoints,
    selectedProgramControlPointIndex,
    updateSettings,
    weldDraft,
  ]);

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
    commitTrajectoryDraftEdit(
      (current) =>
        current.length <= 1
          ? current
          : current.filter((_, pointIndex) => pointIndex !== index),
      { previewMode: "immediate" },
    );
  }, [commitTrajectoryDraftEdit, weldDraft]);

  const handleTreeMoveWaypoint = useCallback(
    (direction: -1 | 1) => {
      if (selectedProgramControlPointIndex === null) {
        return;
      }
      const nextIndex = selectedProgramControlPointIndex + direction;
      if (nextIndex < 0 || nextIndex >= treeEditableWaypoints.length) {
        return;
      }
      if (weldDraft) {
        setWeldEditableWaypoints((current) => {
          const next = [...current];
          const [moved] = next.splice(selectedProgramControlPointIndex, 1);
          next.splice(nextIndex, 0, moved);
          return next;
        });
        setWeldPreviewCacheReady(false);
        updateSettings({ selectedProgramNodeId: `control_point_${nextIndex}` });
        return;
      }
      commitTrajectoryDraftEdit((current) => {
        const next = [...current];
        const [moved] = next.splice(selectedProgramControlPointIndex, 1);
        next.splice(nextIndex, 0, moved);
        return next;
      }, { previewMode: "immediate" });
      updateSettings({ selectedProgramNodeId: `control_point_${nextIndex}` });
    },
    [
      commitTrajectoryDraftEdit,
      selectedProgramControlPointIndex,
      treeEditableWaypoints.length,
      updateSettings,
      weldDraft,
    ],
  );

  const handleApplyTreeWaypointEdits = useCallback(async () => {
    if (weldDraft) {
      await handleApplyWeldWaypointEdits();
      return;
    }
    await ensureTrajectoryPreview(treeEditableWaypoints);
  }, [
    ensureTrajectoryPreview,
    weldDraft,
    handleApplyWeldWaypointEdits,
    treeEditableWaypoints,
  ]);

  const handleUndoTrajectoryDraftEdit = useCallback(async () => {
    if (weldDraft || trajectoryDraftHistory.length === 0) {
      return;
    }
    const previousSnapshot = clonePoseWaypointList(trajectoryDraftHistory[trajectoryDraftHistory.length - 1] ?? []);
    setTrajectoryDraftHistory((currentHistory) => currentHistory.slice(0, -1));
    clearTrajectoryPreviewDebounce();
    setError(null);
    setTrajectoryPlannerFailure(null);
    setPlannerPoints(previousSnapshot);
    if (previousSnapshot.length === 0) {
      cancelTrajectoryPreviewRequest();
      setPreviewPlan(null);
      setWeldPreviewGhostJoints(null);
      return;
    }
    await requestPlannerPreview(previousSnapshot);
  }, [
    cancelTrajectoryPreviewRequest,
    clearTrajectoryPreviewDebounce,
    requestPlannerPreview,
    trajectoryDraftHistory,
    weldDraft,
  ]);

  const handleRevertTrajectoryDraft = useCallback(async () => {
    if (weldDraft) {
      return;
    }
    const baselineSnapshot = clonePoseWaypointList(savedTrajectoryWaypoints);
    setTrajectoryDraftHistory([]);
    clearTrajectoryPreviewDebounce();
    setError(null);
    setTrajectoryPlannerFailure(null);
    setPlannerPoints(baselineSnapshot);
    if (baselineSnapshot.length === 0) {
      cancelTrajectoryPreviewRequest();
      setPreviewPlan(null);
      setWeldPreviewGhostJoints(null);
      return;
    }
    await requestPlannerPreview(baselineSnapshot);
  }, [
    cancelTrajectoryPreviewRequest,
    clearTrajectoryPreviewDebounce,
    requestPlannerPreview,
    savedTrajectoryWaypoints,
    weldDraft,
  ]);

  const handleCaptureTrajectoryPose = useCallback(async () => {
    if (isPlanLoading || isRunningPreview || motionStatusActive) {
      return;
    }
    setError(null);
    try {
      const nextWaypoint = await fetchLiveTrajectoryPoseWaypoint("linear");
      const nextWaypoints = [...getLatestTrajectoryDraftWaypoints(), nextWaypoint];
      setPlannerPoints(nextWaypoints);
      await requestPlannerPreview(nextWaypoints);
    } catch (err) {
      setError(`Failed to capture current pose: ${(err as Error).message}`);
    }
  }, [
    getLatestTrajectoryDraftWaypoints,
    fetchLiveTrajectoryPoseWaypoint,
    isPlanLoading,
    isRunningPreview,
    motionStatusActive,
    requestPlannerPreview,
  ]);


  const refreshWeldProgramList = useCallback(async () => {
    setIsWeldProgramListLoading(true);
    try {
      const response = await fetch(`${normalizedApiHost}/robot-program/list?kind=weld`);
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
      const response = await fetch(`${normalizedApiHost}/robot-program/save`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          kind: "weld",
          name: programName,
          authoring: {
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
          },
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
        `${normalizedApiHost}/robot-program/${encodeURIComponent(selectedWeldProgram.trim())}?kind=weld`,
      );
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || `Load weld program failed (${response.status})`);
      }
      const record = (await response.json()) as SavedRobotProgramRecord;
      const authoring =
        record.authoring && typeof record.authoring === "object"
          ? (record.authoring as Record<string, unknown>)
          : null;
      const program = authoring
        ? ({
            name: record.name,
            saved_at: record.saved_at,
            step: authoring.step,
            weld_draft: authoring.weld_draft,
            editable_waypoints: authoring.editable_waypoints,
            planned_trajectory: record.planned_trajectory ?? null,
          } as WeldProgramRecord)
        : null;
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
        editableWaypoints: coercePoseWaypointList(program.editable_waypoints),
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
      const response = await fetch(`${normalizedApiHost}/robot-program/list?kind=trajectory`);
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || `List request failed (${response.status})`);
      }
      const data = (await response.json()) as { programs?: unknown };
      const names = Array.isArray(data.programs)
        ? data.programs
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
      setError(`Failed to load trajectory programs: ${(err as Error).message}`);
      setSavedTrajectories([]);
      setSelectedTrajectory("");
    } finally {
      setIsTrajectoryListLoading(false);
      trajectoryRefreshInFlight.current = false;
    }
  }, [normalizedApiHost]);

  const handleSaveTrajectoryProgram = useCallback(async () => {
    if (plannerPoints.length === 0) {
      setError("Add at least one waypoint before saving.");
      return;
    }
    const programName = trajectoryProgramName.trim();
    if (!programName) {
      setError("Program name is required.");
      return;
    }
    setIsSavingTrajectoryProgram(true);
    setError(null);
    try {
      const plannedTrajectoryForSave = await ensureTrajectoryPreview(plannerPoints);
      const authoringWaypoints = plannedTrajectoryForSave?.waypoints ?? plannerPoints;
      const response = await fetch(`${normalizedApiHost}/robot-program/save`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          kind: "trajectory",
          name: programName,
          authoring: {
            waypoints: authoringWaypoints,
            metadata: {
              source: "trajectory_editor",
            },
          },
          planned_trajectory: plannedTrajectoryForSave,
        }),
      });
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || `Save trajectory program failed (${response.status})`);
      }
      await refreshTrajectoryList();
      setSelectedTrajectory(programName);
      setLoadedTrajectoryMetadata({ source: "trajectory_editor" });
      markTrajectoryDraftSaved(authoringWaypoints);
    } catch (err) {
      setError(`Failed to save trajectory program: ${(err as Error).message}`);
    } finally {
      setIsSavingTrajectoryProgram(false);
    }
  }, [
    ensureTrajectoryPreview,
    markTrajectoryDraftSaved,
    normalizedApiHost,
    plannerPoints,
    refreshTrajectoryList,
    trajectoryProgramName,
  ]);

  const handleSelectTrajectory = useCallback((value: string) => {
    setSelectedTrajectory(value);
  }, []);

  const handleRegenerateTrajectory = useCallback(async () => {
    if (plannerPoints.length === 0 || isPlanLoading || isRunningPreview || motionStatusActive) {
      return;
    }
    clearTrajectoryPreviewDebounce();
    setError(null);
    const persistProgramName =
      selectedTrajectory && trajectoryProgramName.trim() === selectedTrajectory
        ? selectedTrajectory
        : undefined;
    await requestPlannerPreview(plannerPoints, {
      persistProgramName,
      persistMetadata: persistProgramName ? loadedTrajectoryMetadata : null,
    });
  }, [
    clearTrajectoryPreviewDebounce,
    plannerPoints,
    isPlanLoading,
    isRunningPreview,
    motionStatusActive,
    requestPlannerPreview,
    selectedTrajectory,
    trajectoryProgramName,
    loadedTrajectoryMetadata,
  ]);

  const handleLoadTrajectory = useCallback(async () => {
    if (!selectedTrajectory || isLoadingSavedTrajectory) {
      return;
    }
    cancelTrajectoryPreviewRequest();
    setIsPlanLoading(false);
    setError(null);
    setIsLoadingSavedTrajectory(true);
    try {
      const response = await fetch(
        `${normalizedApiHost}/robot-program/${encodeURIComponent(selectedTrajectory)}?kind=trajectory`,
      );
      if (!response.ok) {
        const message = await response.text();
        throw new Error(message || `Load request failed (${response.status})`);
      }
      const record = normalizeTrajectoryProgramRecord(await response.json());
      if (!record) {
        throw new Error("Saved trajectory program is invalid.");
      }
      const savedPlan = record.planned_trajectory ?? null;
      const planWaypoints =
        savedPlan?.trajectory
          ? previewFromTrajectoryDetail(record.name, savedPlan.trajectory).waypoints
          : [];
      const waypoints = mergePoseWaypointMotionMetadata(
        coercePoseWaypointList(record.authoring.waypoints),
        savedPlan?.waypoints ?? planWaypoints,
      );
      if (waypoints.length === 0) {
        throw new Error("Saved trajectory program has no waypoints.");
      }
      const savedPlanMatches = previewPlanMatchesWaypoints(savedPlan, waypoints);
      const hydratedSavedPlan = savedPlan
        ? {
            ...savedPlan,
            name: record.name,
            useCache: false,
            isStale: !savedPlanMatches || Boolean(savedPlan.isStale),
            sourcePlanName: savedPlan.sourcePlanName ?? savedPlan.name,
          }
        : null;
      clearTrajectoryPreviewDebounce();
      setPlannerPoints(waypoints);
      setPreviewPlan(hydratedSavedPlan);
      setTrajectoryProgramName(record.name);
      setLoadedTrajectoryMetadata(record.authoring?.metadata ?? null);
      markTrajectoryDraftSaved(waypoints);
      setWeldDraft(null);
      setWeldEditableWaypoints([]);
      setWeldPreviewGhostJoints(null);
      setWeldPreviewCacheReady(false);
      if (savedPlan && !savedPlanMatches) {
        setError("Saved computed path does not match the stored waypoints. Regenerate the path to resolve it.");
      }
      setIsPlanning(false);
    } catch (err) {
      setError(`Failed to load trajectory: ${(err as Error).message}`);
      setWeldPreviewGhostJoints(null);
    } finally {
      setIsLoadingSavedTrajectory(false);
    }
  }, [
    cancelTrajectoryPreviewRequest,
    clearTrajectoryPreviewDebounce,
    isLoadingSavedTrajectory,
    markTrajectoryDraftSaved,
    normalizedApiHost,
    selectedTrajectory,
  ]);

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
      const payload = (await response.json()) as MotionStatusResponse;
      setMotionStatus(payload);
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
      const payload = (await response.json()) as MotionStatusResponse;
      setMotionStatus(payload);
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
      const payload = (await response.json()) as MotionStatusResponse;
      setMotionStatus(payload);
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

  useEffect(() => {
    persistSettings(settings);
  }, [settings]);

  useEffect(() => {
    if (!selectedProgramNodeId) {
      return;
    }
    if (isPlanLoading) {
      return;
    }
    if (programNodeById.has(selectedProgramNodeId)) {
      return;
    }
    updateSettings({ selectedProgramNodeId: null });
  }, [isPlanLoading, selectedProgramNodeId, programNodeById, updateSettings]);

  useEffect(() => {
    if (!selectedProgramNodeId) {
      return;
    }
    setSelectedTimelineSyntheticId(null);
  }, [selectedProgramNodeId]);

  useEffect(() => {
    if (!selectedTimelineSyntheticId) {
      return;
    }
    if (isPlanLoading) {
      return;
    }
    if (trajectoryMoveTimelineSelections.some((item) => item.id === selectedTimelineSyntheticId)) {
      return;
    }
    setSelectedTimelineSyntheticId(null);
  }, [isPlanLoading, selectedTimelineSyntheticId, trajectoryMoveTimelineSelections]);

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
    if (
      !selectedProgramNodeId &&
      !selectedTimelineSyntheticId &&
      !selectedProgramMoveTimelineFocus
    ) {
      return;
    }
    if (selectedNodeFocusPoints.length === 0) {
      return;
    }
    visualizerRef.current?.focusOnPoints(selectedNodeFocusPoints);
  }, [
    selectedProgramMoveTimelineFocus,
    selectedProgramNodeId,
    selectedTimelineSyntheticId,
    selectedNodeFocusPoints,
  ]);

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
      setSelectedTimelineSyntheticId(null);
      panelSelectionOriginRef.current = "tree";
      updateSettings({
        selectedProgramNodeId: nodeId,
        activePanel: nextPanel ?? activePanel,
      });
    },
    [programNodeById, updateSettings, activePanel],
  );
  const handleSelectTimelineItem = useCallback(
    (itemId: string) => {
      if (itemId.startsWith("timeline_segment_")) {
        const selectedSegment = trajectoryMoveTimelineSelections.find((item) => item.id === itemId);
        if (!selectedSegment) {
          return;
        }
        const matchingMoveNodeId =
          typeof selectedSegment.relatedMoveIndex === "number"
            ? (programMoveNodeIdByIndex.get(selectedSegment.relatedMoveIndex) ?? null)
            : null;
        setSelectedTimelineSyntheticId(matchingMoveNodeId ? null : itemId);
        panelSelectionOriginRef.current = null;
        updateSettings({
          selectedProgramNodeId: matchingMoveNodeId,
          activePanel: selectedSegment.openPanel ?? activePanel,
        });
        return;
      }
      handleSelectProgramTreeNode(itemId);
    },
    [
      activePanel,
      handleSelectProgramTreeNode,
      programMoveNodeIdByIndex,
      trajectoryMoveTimelineSelections,
      updateSettings,
    ],
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
  const monitorLastMessageAtMs = latest?.timestamp ?? null;
  const monitorFreshnessMs = monitorLastMessageAtMs === null
    ? null
    : Math.max(0, liveClockMs - monitorLastMessageAtMs);
  const isMonitorFresh = monitorFreshnessMs !== null && monitorFreshnessMs <= LIVE_MONITOR_STALE_MS;
  const apiHealthError = !isConnected
    ? (
        error === "Connection lost. Ensure the API is reachable and CORS allows this origin."
          ? "API unreachable. Check that gradient-api is running."
          : null
      )
    : (monitorLastMessageAtMs !== null && !isMonitorFresh
        ? `Controller telemetry stale (${Math.round(monitorFreshnessMs ?? 0)} ms).`
        : null);
  const headerAlert = [error, visionError].filter(Boolean).join(" • ");
  const hasHeaderAlert = headerAlert.length > 0;
  const alertTone = error ? "rose" : "amber";
  const activeRuntimeModeLabel = runtimeMode === "simulate" ? "SIM" : runtimeMode === "live" ? "LIVE" : "UNKNOWN";
  const desiredRuntimeModeLabel = selectedRuntimeMode === "simulate" ? "SIM" : "LIVE";
  const runtimeModeChangePending = runtimeMode !== null && selectedRuntimeMode !== runtimeMode;
  const trajectoryRunPreparationTitle = "Recalculating Trajectory Path";
  const trajectoryRunPreparationMessage =
    "Rebuilding the runnable path from the robot's current pose before motion starts.";
  const stageSurfaceHeading = isVisionActive
    ? "Live Camera Workspace"
    : (selectedProgramNode?.label ?? "Robot Workspace");
  const currentProgramLabel = programTreeRoot?.label?.trim() ?? "";
  const showCurrentProgramLabel = currentProgramLabel.length > 0 && currentProgramLabel.toLowerCase() !== "no program";
  const stageGuidance = isVisionActive
    ? (
        visionError
          ? "Vision stream unavailable. Retry the feed or switch back to the 3D workspace while the camera comes back."
          : "Vision view is active. Switch back to 3D to inspect robot pose, paths, and weld geometry."
      )
    : (
        weldSelectionMode
          ? "Weld selection is active. Click highlighted edges in the 3D stage to build weld sections."
          : isPlanning
            ? "Trajectory editing is active. Shift-click the stage or capture the live robot pose to add waypoints."
            : "Select a program node or timeline block to focus the stage on that part of the job."
      );
  const controllerStatusTone = apiHealthError?.startsWith("API unreachable")
    ? "rose"
    : apiHealthError
      ? "amber"
      : null;
  const activeDrawerHeader = activePanel === "step"
    ? (
        <span className="text-xs font-semibold uppercase tracking-[0.25em] text-cyan-200/80">
          STEP Import
        </span>
      )
    : activePanel === "trajectory"
      ? (
          <div className="min-w-0">
            <div className="text-xs font-semibold uppercase tracking-[0.25em] text-cyan-200/80">
              Trajectory
            </div>
            <div className="mt-1 max-w-[18rem] text-[11px] leading-4 text-slate-300/80">
              Waypoint authoring with live pose capture, IK-planned preview, and explicit SIM/LIVE execution.
            </div>
          </div>
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
            isSubmittingRun={isRunningPreview}
            isMotionActive={motionStatusActive}
            motionStatus={motionStatus}
            preview={previewPlan}
            plannerFailure={trajectoryPlannerFailure}
            plannerPoints={plannerPoints}
            savedTrajectories={savedTrajectories}
            selectedTrajectory={selectedTrajectory}
            isTrajectoryListLoading={isTrajectoryListLoading}
            isLoadingSavedTrajectory={isLoadingSavedTrajectory}
            trajectoryProgramName={trajectoryProgramName}
            onTrajectoryProgramNameChange={setTrajectoryProgramName}
            onSaveTrajectory={handleSaveTrajectoryProgram}
            isSavingTrajectory={isSavingTrajectoryProgram}
            runtimeMode={runtimeMode}
            onPlanToggle={handlePlanToggle}
            onSimulate={handleSimulatePreview}
            onRunLive={handleRunPreviewLive}
            loopEnabled={trajectoryLoopEnabled}
            onLoopEnabledChange={setTrajectoryLoopEnabled}
            onCapturePose={handleCaptureTrajectoryPose}
            onAddWaypoint={handleAddTrajectoryWaypoint}
            onAddHomeWaypoint={handleAddTrajectoryHomeWaypoint}
            onClear={handleClearPreview}
            onRefreshTrajectories={refreshTrajectoryList}
            onSelectTrajectory={handleSelectTrajectory}
            onLoadTrajectory={handleLoadTrajectory}
            onRegenerateTrajectory={handleRegenerateTrajectory}
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
          isSubmittingRun={isRunningPreview}
          isMotionActive={motionStatusActive}
          motionStatus={motionStatus}
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
              runtimeMode={runtimeMode}
              onSimulate={handleSimulatePreview}
              onRunLive={handleRunPreviewLive}
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
          ? <TelemetryWorkspace />
        : null;
  const liveStateValue = useMemo(
    () => ({
      apiHost: normalizedApiHost,
      latest,
      motionStatus,
      setMotionStatus,
      isConnected,
      monitorLastMessageAtMs,
      monitorFreshnessMs,
      isMonitorFresh,
      monitorError: error,
      apiHealthError,
    }),
    [
      apiHealthError,
      error,
      isConnected,
      isMonitorFresh,
      latest,
      monitorFreshnessMs,
      monitorLastMessageAtMs,
      motionStatus,
      normalizedApiHost,
    ],
  );

  return (
    <LiveStateProvider value={liveStateValue}>
      <div className="flex h-[100dvh] min-h-0 flex-col overflow-hidden bg-gradient-to-b from-slate-900/80 via-slate-950 to-black text-slate-100">
      <header className={`relative shrink-0 flex flex-col border-b border-slate-800/40 bg-slate-950/60 px-6 pt-2 pb-0 shadow-inner shadow-slate-900/40 backdrop-blur ${
        hasHeaderAlert ? "gap-1 pb-1" : "gap-0"
      }`}>
        <div className={`relative flex flex-col gap-2 xl:flex-row xl:items-stretch ${
          hasHeaderAlert ? "min-h-[3.25rem]" : "min-h-[3rem]"
        }`}>
          <div className="flex shrink-0 flex-col justify-center">
            <div className="flex justify-end text-lg font-semibold tracking-tight text-cyan-300 sm:text-xl w-full leading-tight">
              Control Center
            </div>
            <div className="flex justify-end text-xs tracking-tight text-cyan-300/90 sm:text-sm w-full leading-tight">
              Gradient Robotics
            </div>
          </div>
          <div className="flex h-full min-w-0 justify-center xl:pointer-events-none xl:absolute xl:inset-y-0 xl:left-1/2 xl:w-[min(52rem,calc(100%-28rem))] xl:-translate-x-1/2">
            <div className="flex h-full min-w-0 items-stretch justify-center gap-2">
              <div className="pointer-events-auto flex min-w-0 items-center gap-2 rounded-xl border border-slate-700/70 bg-slate-950/80 px-2 py-1 shadow-[0_0_24px_rgba(15,23,42,0.28)]">
                <div className="hidden min-w-0 flex-col justify-center sm:flex">
                  <div className="text-[9px] font-semibold uppercase tracking-[0.18em] text-cyan-200/75">
                    Runtime Mode
                  </div>
                  <div className={`text-[10px] font-semibold ${runtimeModeChangePending ? "text-amber-200" : "text-slate-200"}`}>
                    {runtimeModeChangePending
                      ? `Current ${activeRuntimeModeLabel} · desired ${desiredRuntimeModeLabel}`
                      : `${activeRuntimeModeLabel} active`}
                  </div>
                  <div className="text-[10px] text-slate-400">
                    Tap LIVE or SIM to hot-switch without restarting.
                  </div>
                </div>
                <div className="inline-flex shrink-0 items-stretch rounded-lg border border-slate-700/80 bg-slate-950/90 p-1 shadow-inner shadow-black/30">
                  <button
                    type="button"
                    onClick={() => {
                      void handleSelectRuntimeMode("live");
                    }}
                    disabled={isRuntimeConfigBusy || isRestartingController}
                    title="Switch controller into LIVE mode now without restarting."
                    className={`rounded-md px-3 py-1.5 text-[11px] font-semibold uppercase tracking-[0.16em] transition ${
                      selectedRuntimeMode === "live"
                        ? "border border-rose-300/70 bg-rose-400/20 text-rose-50 shadow-[0_0_18px_rgba(251,113,133,0.24)]"
                        : "border border-transparent text-slate-300 hover:border-slate-500/60 hover:text-slate-100"
                    } ${(isRuntimeConfigBusy || isRestartingController) ? "cursor-not-allowed opacity-60" : ""}`}
                  >
                    LIVE
                  </button>
                  <button
                    type="button"
                    onClick={() => {
                      void handleSelectRuntimeMode("simulate");
                    }}
                    disabled={isRuntimeConfigBusy || isRestartingController}
                    title="Switch controller into SIM mode now without restarting."
                    className={`rounded-md px-3 py-1.5 text-[11px] font-semibold uppercase tracking-[0.16em] transition ${
                      selectedRuntimeMode === "simulate"
                        ? "border border-cyan-300/70 bg-cyan-400/20 text-cyan-50 shadow-[0_0_18px_rgba(34,211,238,0.24)]"
                        : "border border-transparent text-slate-300 hover:border-slate-500/60 hover:text-slate-100"
                    } ${(isRuntimeConfigBusy || isRestartingController) ? "cursor-not-allowed opacity-60" : ""}`}
                  >
                    SIM
                  </button>
                </div>
                {isRuntimeConfigBusy ? (
                  <span className="hidden shrink-0 rounded-lg border border-cyan-400/40 bg-cyan-500/10 px-3 py-1.5 text-[11px] font-semibold uppercase tracking-[0.16em] text-cyan-100 lg:inline-flex">
                    Switching...
                  </span>
                ) : null}
              </div>
              <ControlPanelRuntimeHeader
                apiHost={normalizedApiHost}
                driveFaults={latest?.drive_faults ?? null}
                activeServoBackend={runtimeConfigSnapshot?.active?.servo_backend?.effective_backend ?? null}
                onJointFeedback={handleFallbackJointFeedback}
                onError={(message) => setError(message)}
                className="min-w-0 h-full flex-wrap justify-center xl:pointer-events-auto xl:flex-nowrap"
              />
            </div>
          </div>
          <div className="flex items-center gap-2 xl:ml-auto">
            {apiHealthError ? (
              <span
                className={`inline-flex items-center gap-1.5 rounded-full px-2.5 py-0.5 text-[11px] font-medium ${
                  controllerStatusTone === "rose"
                    ? "bg-rose-500/10 text-rose-200 ring-1 ring-inset ring-rose-400/20"
                    : "bg-amber-500/10 text-amber-200 ring-1 ring-inset ring-amber-400/25"
                }`}
                title={apiHealthError}
              >
                <span
                  className={`h-1.5 w-1.5 rounded-full ${
                    controllerStatusTone === "rose" ? "bg-rose-400" : "bg-amber-400"
                  }`}
                />
                {controllerStatusTone === "rose" ? "API Offline" : "Controller Offline"}
              </span>
            ) : null}
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
              aria-label={isVisionActive ? "Show 3D workspace" : "Show vision workspace"}
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
        {hasHeaderAlert ? (
          <div
            className={`flex h-7 w-full items-center overflow-hidden rounded-lg border px-2 text-[11px] sm:text-xs ${
              alertTone === "rose"
                ? "border-rose-500/40 bg-rose-500/10 text-rose-200"
                : "border-amber-500/40 bg-amber-500/10 text-amber-200"
            }`}
            role="alert"
            aria-live="polite"
            title={headerAlert}
          >
            <span className="w-full truncate">{headerAlert}</span>
          </div>
        ) : null}
      </header>
      <main className="min-h-0 h-full flex-1 overflow-hidden p-4">
        <div className="grid h-full min-h-0 gap-0" style={shellGridStyle}>
          <div className="col-start-1 row-start-1 grid min-h-0 gap-0" style={workspaceColumnsStyle}>
              <div className="grid min-h-0 grid-cols-[4.75rem_minmax(0,1fr)] grid-rows-[minmax(0,1.15fr)_minmax(0,1fr)] gap-4">
                <div className="relative z-30 row-span-2 min-h-0 overflow-visible">
                  <SidebarRail
                    items={sidebarItems}
                    activeItemId={activePanel}
                    onSelect={handleSelectSidebarPanel}
                  />
                </div>
                {activePanel && activeDrawerContent ? (
                  <SidebarDrawer
                    onClose={() => updateSettings({ activePanel: null })}
                    headerContent={activeDrawerHeader}
                    widthClassName="row-span-2 w-full"
                    heightMode="full"
                  >
                    {activeDrawerContent}
                  </SidebarDrawer>
                ) : (
                  <section className="row-span-2 flex min-h-0 flex-col justify-center rounded-2xl border border-slate-800/80 bg-slate-950/72 p-5 shadow-xl shadow-black/20">
                    <div className="text-[11px] font-semibold uppercase tracking-[0.24em] text-cyan-200/80">
                      Authoring Surface
                    </div>
                    <h2 className="mt-2 text-xl font-semibold tracking-tight text-slate-50">
                      Select a workspace from the left rail
                    </h2>
                    <p className="mt-2 text-sm leading-6 text-slate-400">
                      The menu is back on the left-hand side so the shell feels closer to the earlier
                      layout, while the authoring panes remain docked and non-overlapping.
                    </p>
                    <div className="mt-4 rounded-2xl border border-slate-700/70 bg-slate-900/55 px-4 py-3">
                      <div className="text-[12px] font-semibold text-slate-100">Available sections</div>
                      <div className="mt-2 flex flex-wrap gap-2">
                        {sidebarItems.map((item) => (
                          <button
                            key={`workspace-chip-${item.id}`}
                            type="button"
                            onClick={() => handleSelectSidebarPanel(item.id)}
                            className="rounded-full border border-slate-700/70 bg-slate-950/70 px-3 py-1.5 text-[11px] font-semibold text-slate-300 transition hover:border-cyan-400/40 hover:text-cyan-100"
                          >
                            {item.label}
                          </button>
                        ))}
                      </div>
                    </div>
                  </section>
                )}
              </div>
              <PaneResizeHandle
                orientation="vertical"
                active={activeShellDrag?.kind === "left"}
                onPointerDown={startShellDrag("left")}
              />
              <section className="relative min-h-0 overflow-hidden rounded-[1.75rem] border border-slate-800/80 bg-slate-950 shadow-2xl shadow-black/30">
              {isVisualizerEnabled ? (
                <Suspense
                  fallback={
                    <div className="absolute inset-0 flex items-center justify-center bg-slate-950/80">
                      <div className="rounded-2xl border border-cyan-500/20 bg-slate-950/85 px-5 py-4 text-sm text-slate-200 shadow-2xl shadow-black/40">
                        Loading 3D workspace...
                      </div>
                    </div>
                  }
                >
                  <LazyArmVisualizer
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
                    highlightPathRange={activeSelectionPathRange}
                    highlightWaypointIndices={activeSelectionWaypointIndices}
                    stepFile={stepFile}
                    stepTransform={stepTransform}
                    onStepStatusChange={setStepLoadStatus}
                  />
                </Suspense>
              ) : (
                <div className="absolute inset-0 bg-[radial-gradient(circle_at_top,rgba(34,211,238,0.12),transparent_32%),linear-gradient(180deg,rgba(2,6,23,0.88),rgba(2,6,23,1))]">
                  <div className="pointer-events-none flex h-full items-center justify-center px-6">
                    <div className="pointer-events-auto max-w-xl rounded-2xl border border-slate-700/70 bg-slate-950/88 p-6 shadow-2xl shadow-black/40 backdrop-blur">
                      <div className="text-xs font-semibold uppercase tracking-[0.22em] text-cyan-300/80">
                        Lightweight startup
                      </div>
                      <h2 className="mt-2 text-2xl font-semibold tracking-tight text-slate-50">
                        Control UI loaded with 3D paused
                      </h2>
                      <p className="mt-3 max-w-lg text-sm leading-6 text-slate-300">
                        The robot visualizer is the heaviest part of the page on this device. It now
                        stays paused until you ask for it, so the rest of the UI can open reliably.
                      </p>
                      <div className="mt-5 flex flex-wrap items-center gap-3">
                        <button
                          type="button"
                          onClick={() => setIsVisualizerEnabled(true)}
                          className="rounded-lg border border-cyan-400/40 bg-cyan-500/15 px-4 py-2 text-sm font-semibold text-cyan-100 transition hover:border-cyan-300/60 hover:bg-cyan-400/20"
                        >
                          Load 3D Workspace
                        </button>
                        <span className="text-xs text-slate-400">
                          Jog, telemetry, settings, and the control panel stay available without it.
                        </span>
                      </div>
                    </div>
                  </div>
                </div>
              )}
              {isVisionActive ? (
                <div className="absolute inset-0 z-10 bg-[radial-gradient(circle_at_top,rgba(34,211,238,0.1),transparent_34%),linear-gradient(180deg,rgba(2,6,23,0.86),rgba(2,6,23,1))]">
                  {visionError ? (
                    <div className="pointer-events-none flex h-full items-center justify-center px-6">
                      <div className="pointer-events-auto max-w-xl rounded-2xl border border-slate-700/70 bg-slate-950/88 p-6 shadow-2xl shadow-black/40 backdrop-blur">
                        <div className="text-xs font-semibold uppercase tracking-[0.22em] text-cyan-300/80">
                          Vision Feed
                        </div>
                        <h2 className="mt-2 text-2xl font-semibold tracking-tight text-slate-50">
                          Camera stream unavailable
                        </h2>
                        <p className="mt-3 max-w-lg text-sm leading-6 text-slate-300">
                          {visionError}
                        </p>
                        <div className="mt-5 flex flex-wrap items-center gap-3">
                          <button
                            type="button"
                            onClick={showVisionStage}
                            className="rounded-lg border border-cyan-400/40 bg-cyan-500/15 px-4 py-2 text-sm font-semibold text-cyan-100 transition hover:border-cyan-300/60 hover:bg-cyan-400/20"
                          >
                            Retry Camera Feed
                          </button>
                          <button
                            type="button"
                            onClick={showStage3d}
                            className="rounded-lg border border-slate-700/70 bg-slate-900/80 px-4 py-2 text-sm font-semibold text-slate-100 transition hover:border-slate-500/70 hover:text-white"
                          >
                            Show 3D Workspace
                          </button>
                        </div>
                      </div>
                    </div>
                  ) : (
                    <img
                      src={visionStreamUrl}
                      alt="Gradient Vision stream"
                      className="h-full w-full object-contain bg-slate-950"
                      onLoad={() => setVisionError(null)}
                      onError={() => {
                        setVisionError(
                          "Unable to load the vision stream. Ensure Gradient Vision is running and accessible.",
                        );
                      }}
                    />
                  )}
                </div>
              ) : null}
              <div className="pointer-events-none absolute inset-x-4 top-4 z-20 flex justify-center">
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
              <div className="pointer-events-none absolute left-4 top-4 z-20">
                <div className="pointer-events-auto inline-flex max-w-[calc(100%-17rem)] items-center gap-1.5 border border-slate-700/70 bg-slate-950/82 px-2 py-1.5 text-xs text-slate-200">
                  <div className="flex min-w-0 items-center gap-2 border-r border-slate-700/70 pr-2">
                    <span className="max-w-[10rem] truncate text-sm font-semibold tracking-tight text-slate-50">
                      {stageSurfaceHeading}
                    </span>
                  </div>
                  <span className="shrink-0 border border-slate-700/70 bg-slate-900/70 px-1.5 py-0.5 text-[10px] font-semibold uppercase tracking-[0.16em] text-slate-200">
                    {activePanelTitle}
                  </span>
                  <span className="shrink-0 border border-slate-700/70 bg-slate-900/70 px-1.5 py-0.5 text-[10px] font-semibold uppercase tracking-[0.16em] text-slate-200">
                    {runtimeMode === "simulate" ? "SIM" : runtimeMode === "live" ? "LIVE" : "UNKNOWN"}
                  </span>
                  {showCurrentProgramLabel ? (
                    <span className="max-w-[10rem] truncate border border-slate-700/70 bg-slate-900/70 px-1.5 py-0.5 text-[10px] font-semibold uppercase tracking-[0.16em] text-slate-200">
                      {currentProgramLabel}
                    </span>
                  ) : null}
                </div>
              </div>
              {showTrajectoryRunPreparationOverlay ? (
                <div className="pointer-events-none absolute inset-x-0 top-16 z-20 flex justify-center px-6">
                  <div className="pointer-events-auto flex w-full max-w-xl items-start gap-3 rounded-2xl border border-cyan-400/35 bg-slate-950/92 px-4 py-3 text-left shadow-xl shadow-black/30 backdrop-blur">
                    <RefreshCcw
                      size={16}
                      strokeWidth={2.2}
                      className="mt-0.5 shrink-0 animate-spin text-cyan-300"
                    />
                    <div className="min-w-0">
                      <div className="text-[11px] font-semibold uppercase tracking-[0.18em] text-cyan-200/90">
                        {trajectoryRunPreparationTitle}
                      </div>
                      <div className="mt-1 text-sm text-slate-100">
                        {trajectoryRunPreparationMessage}
                      </div>
                    </div>
                  </div>
                </div>
              ) : null}
              <div className="pointer-events-none absolute right-4 top-4 z-20">
                <div className="pointer-events-auto inline-flex items-center gap-1 rounded-2xl border border-slate-700/70 bg-slate-950/82 p-1.5 shadow-xl shadow-black/20 backdrop-blur">
                  <button
                    type="button"
                    onClick={showStage3d}
                    className={`inline-flex items-center rounded-xl px-3 py-2 text-xs font-semibold uppercase tracking-[0.18em] transition ${
                      !isVisionActive
                        ? "bg-cyan-400 text-slate-950 shadow-sm shadow-cyan-400/30"
                        : "text-slate-300 hover:bg-slate-800/80 hover:text-slate-100"
                    }`}
                  >
                    3D Stage
                  </button>
                  <button
                    type="button"
                    onClick={showVisionStage}
                    className={`inline-flex items-center rounded-xl px-3 py-2 text-xs font-semibold uppercase tracking-[0.18em] transition ${
                      isVisionActive
                        ? "bg-cyan-400 text-slate-950 shadow-sm shadow-cyan-400/30"
                        : "text-slate-300 hover:bg-slate-800/80 hover:text-slate-100"
                    }`}
                  >
                    Vision Feed
                  </button>
                </div>
              </div>
              <div className="pointer-events-none absolute bottom-4 left-4 z-20">
                <div className="pointer-events-auto rounded-2xl border border-slate-700/70 bg-slate-950/82 px-4 py-3 text-sm text-slate-300 shadow-xl shadow-black/20 backdrop-blur">
                  <span>{stageGuidance}</span>
                </div>
              </div>
              </section>
          </div>
          <div className="col-start-1 row-start-2 min-h-0">
            <PaneResizeHandle
              orientation="horizontal"
              active={activeShellDrag?.kind === "timeline"}
              onPointerDown={startShellDrag("timeline")}
            />
          </div>
          <div className="col-start-1 row-start-3 min-h-0 h-full overflow-hidden pt-1">
            <ProgramTimeline
              lanes={timelineLanes}
              playheadLabel={currentPlayheadLabel}
              playheadDetail={currentPlayheadDetail}
              onSelectItem={handleSelectTimelineItem}
            />
          </div>
          <div className="col-start-2 row-start-1 row-span-3 min-h-0">
            <PaneResizeHandle
              orientation="vertical"
              active={activeShellDrag?.kind === "right"}
              onPointerDown={startShellDrag("right")}
            />
          </div>
          <div className="col-start-3 row-start-1 row-span-3 flex h-full min-h-0 flex-col overflow-hidden pl-2 pr-1">
            <div ref={rightDockContainerRef} className="grid min-h-0 flex-1" style={rightDockStyle}>
              <div className="min-h-0 overflow-hidden pb-1">
                {showProgramTree ? (
                  <ProgramFeatureTree
                    root={programTreeRoot}
                    expandedNodeIds={expandedProgramTreeNodeIds}
                    selectedNodeId={selectedProgramNodeId}
                    viewMode={programTreeViewMode}
                    editableControlPoint={selectedProgramControlPoint}
                    editableMove={visibleSelectedProgramMove}
                    canEditWaypointValues={canTreeWaypointValueEdit}
                    canEditMoveType={canTreeWaypointValueEdit}
                    canAddWaypoint={canTreeWaypointAdd}
                    canRemoveWaypoint={canTreeWaypointRemove}
                    canMoveWaypointUp={canTreeWaypointMoveUp}
                    canMoveWaypointDown={canTreeWaypointMoveDown}
                    canApplyWaypointEdits={canTreeWaypointApply}
                    showApplyWaypointEdits={Boolean(weldDraft)}
                    isTrajectoryRecalculating={!weldDraft && isPlanLoading}
                    trajectoryDraftActions={
                      weldDraft
                        ? null
                        : {
                            targetName: trajectoryDraftName,
                            hasUnsavedChanges: hasTrajectoryUnsavedChanges,
                            canUndoLast: canUndoTrajectoryDraftEdit,
                            canUndoAll: canRevertTrajectoryDraft,
                            canSave: canSaveTrajectoryDraft,
                            isSaving: isSavingTrajectoryProgram,
                          }
                    }
                    onToggleExpand={handleToggleProgramTreeNode}
                    onSelectNode={handleSelectProgramTreeNode}
                    onChangeViewMode={handleChangeProgramTreeViewMode}
                    onWaypointChange={handleTreeWaypointChange}
                    onWaypointMoveTypeChange={handleTreeWaypointMoveTypeChange}
                    onWaypointSpeedChange={handleTreeWaypointSpeedChange}
                    onWaypointSpeedReset={handleTreeWaypointSpeedReset}
                    onWaypointAccelerationChange={handleTreeWaypointAccelerationChange}
                    onWaypointAccelerationReset={handleTreeWaypointAccelerationReset}
                    onWaypointPauseChange={handleTreeWaypointPauseChange}
                    onWaypointPauseReset={handleTreeWaypointPauseReset}
                    onAddWaypoint={handleTreeAddWaypoint}
                    onRemoveWaypoint={handleTreeRemoveWaypoint}
                    onMoveWaypointUp={() => handleTreeMoveWaypoint(-1)}
                    onMoveWaypointDown={() => handleTreeMoveWaypoint(1)}
                    onApplyWaypointEdits={handleApplyTreeWaypointEdits}
                    onUndoTrajectoryDraft={handleUndoTrajectoryDraftEdit}
                    onRevertTrajectoryDraft={handleRevertTrajectoryDraft}
                    onSaveTrajectoryDraft={handleSaveTrajectoryProgram}
                  />
                ) : (
                  <section className="flex h-full min-h-0 flex-col items-center justify-center rounded-2xl border border-dashed border-slate-800/80 bg-slate-950/50 px-6 py-5 text-center">
                    <div className="text-[11px] font-semibold uppercase tracking-[0.24em] text-slate-400">
                      Program Tree Hidden
                    </div>
                    <div className="mt-2 max-w-sm text-sm leading-6 text-slate-400">
                      Reopen the docked program tree to edit waypoints and keep the stage and timeline in sync.
                    </div>
                    <button
                      type="button"
                      onClick={() => updateSettings({ showProgramTree: true })}
                      className="mt-4 rounded-xl border border-cyan-400/40 bg-cyan-500/10 px-4 py-2 text-sm font-semibold text-cyan-100 transition hover:border-cyan-300/60 hover:bg-cyan-500/18"
                    >
                      Show Program Tree
                    </button>
                  </section>
                )}
              </div>
              <div className="min-h-0">
                <PaneResizeHandle
                  orientation="horizontal"
                  active={activeShellDrag?.kind === "rightDock"}
                  ariaLabel="Resize program tree and robot controls"
                  onPointerDown={startShellDrag("rightDock")}
                />
              </div>
              <section className="gradient-scrollbar min-h-0 overflow-y-auto rounded-2xl border border-slate-800/80 bg-slate-950/72 p-4 shadow-xl shadow-black/20">
                <div className="text-[11px] font-semibold uppercase tracking-[0.24em] text-cyan-200/80">
                  Robot Control
                </div>
                <div className="mt-1 text-sm text-slate-400">
                  Runtime controls stay persistently docked on the right so authoring never collides with execution tools.
                </div>
                <div className="mt-4">
                  <div className="pr-1">
                    <ControlPanel
                      apiHost={normalizedApiHost}
                      driveFaults={latest?.drive_faults ?? null}
                      activeServoBackend={runtimeConfigSnapshot?.active?.servo_backend?.effective_backend ?? null}
                      onJointFeedback={handleFallbackJointFeedback}
                      onError={(m) => setError(m)}
                      splitStatusSections
                      showStatusSections={false}
                      showGripperPanel={showGripperPanel}
                      controlsCollapsed={isRobotControlCollapsed}
                      onToggleControlsCollapsed={() =>
                        updateSettings({ collapseRobotControl: !isRobotControlCollapsed })
                      }
                    />
                  </div>
                </div>
              </section>
            </div>
          </div>
        </div>
      </main>
      <SettingsDialog
        isOpen={isSettingsOpen}
        initialTab={settingsInitialTab}
        apiHost={apiHost}
        visionHost={visionHost}
        showBoundingBox={showBoundingBox}
        showGripperPanel={showGripperPanel}
        robots={robotOptions}
        selectedRobotName={selectedRobotName}
        selectedRtMaxRpmInput={selectedRtMaxRpmInput}
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
        onShowGripperPanelChange={(value) => updateSettings({ showGripperPanel: value })}
        onSelectedRobotNameChange={setSelectedRobotName}
        onSelectedRtMaxRpmInputChange={setSelectedRtMaxRpmInput}
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
    </LiveStateProvider>
  );
}
