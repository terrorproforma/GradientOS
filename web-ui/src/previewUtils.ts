export type Point3 = { x: number; y: number; z: number };

export type WaypointMoveType = "linear" | "joint" | "home";

export type PoseWaypoint = Point3 & {
  rollDeg: number | null;
  pitchDeg: number | null;
  yawDeg: number | null;
  moveType: WaypointMoveType;
};

export type RobotProgramKind = "trajectory" | "weld";

export type SavedRobotProgramRecord = {
  name: string;
  kind: RobotProgramKind;
  saved_at?: string;
  authoring: Record<string, unknown>;
  planned_trajectory?: PreviewPlan | null;
  metadata?: Record<string, unknown>;
};

export type TrajectoryMove = {
  command: string;
  vector?: number[] | null;
  orientation_euler_deg?: number[] | null;
  [key: string]: unknown;
};

export type TrajectoryFile = {
  description?: string;
  loop?: boolean;
  orientation_euler_angles_deg?: number[] | null;
  weld?: Record<string, unknown>;
  moves: TrajectoryMove[];
};

export type PreviewPlan = {
  name: string;
  trajectory: TrajectoryFile;
  pathPoints: Point3[];
  waypoints: PoseWaypoint[];
  planningWarnings?: string[];
  plannerDiagnostics?: PlannerDiagnostics;
  useCache?: boolean;
  isStale?: boolean;
  sourcePlanName?: string;
};

export type PlannerDiagnosticsResiduals = {
  maxJointStepRad?: number;
  jointLimitMarginRad?: number | null;
  cartesianResidualM?: number;
  orientationResidualDeg?: number;
  violatingJointIndex?: number;
  violatingJointMarginRad?: number;
  violatingPoseIndex?: number;
  violatingJointCount?: number;
  jumpPoseIndex?: number;
  jumpJointIndex?: number;
  jumpJointPreviousRad?: number | null;
  jumpJointCurrentRad?: number | null;
  jumpJointRawStepRad?: number | null;
  jumpJointWrappedStepRad?: number | null;
  jumpContext?: string;
  stepSource?: string;
  [key: string]: unknown;
};

export type PlannerRecoveryAttempt = {
  strategy?: string;
  suffixStartIndex?: number;
  seedJointSample?: Array<number | null>;
  solvedPoints?: number;
  elapsedMs?: number | null;
  reasonCode?: string;
  accepted?: boolean;
  residuals?: PlannerDiagnosticsResiduals;
  rawJump?: PlannerDiagnosticsResiduals;
  [key: string]: unknown;
};

export type PlannerRecoveryDiagnostics = {
  used?: boolean;
  kind?: string;
  strategy?: string;
  sourceAttempt?: string;
  jumpPoseIndex?: number;
  stepSource?: string;
  splitCount?: number;
  distanceM?: number | null;
  failureSegmentIndex?: number;
  reasonCode?: string;
  attempts?: PlannerRecoveryAttempt[];
  segments?: Array<Record<string, unknown>>;
  [key: string]: unknown;
};

export type PlannerDiagnostics = {
  reasonCode?: string;
  seedUsed?: string;
  fallbackLevel?: number;
  attempt?: string;
  smoothingApplied?: boolean;
  profileRevision?: number;
  branchAnchorAvailable?: boolean;
  residuals?: PlannerDiagnosticsResiduals;
  rawJump?: PlannerDiagnosticsResiduals;
  recovery?: PlannerRecoveryDiagnostics;
  splitRecovery?: PlannerRecoveryDiagnostics;
  [key: string]: unknown;
};

export type ProgramNodeType =
  | "program"
  | "setupGroup"
  | "operationGroup"
  | "move"
  | "waypoint"
  | "weldSegment";

export type ProgramNodeFocus = {
  openPanel?: "trajectory" | "weld";
  moveIndex?: number;
  waypointIndices?: number[];
  pathRange?: { start: number; end: number };
  weldSegmentEdgeId?: string;
};

export type ProgramNode = {
  id: string;
  type: ProgramNodeType;
  label: string;
  subtitle?: string;
  badge?: string;
  focus?: ProgramNodeFocus;
  children: ProgramNode[];
};

export type ProgramTreeViewMode = "chronological" | "grouped";

type BuildProgramTreeInput = {
  plan: PreviewPlan | null;
  weldSegments?: Array<{
    edgeId: string;
    startS: number;
    endS: number;
    weldType?: string;
  }>;
  weldType?: string;
  viewMode?: ProgramTreeViewMode;
};

function toNodeId(value: string): string {
  return value.replace(/[^a-zA-Z0-9_-]+/g, "_");
}

function poseSubtitle(point: PoseWaypoint): string {
  const xyz = `${point.x.toFixed(4)}, ${point.y.toFixed(4)}, ${point.z.toFixed(4)}`;
  const moveLabel =
    point.moveType === "joint" ? "joint move" : point.moveType === "home" ? "home move" : "linear move";
  if (
    point.rollDeg === null ||
    point.pitchDeg === null ||
    point.yawDeg === null
  ) {
    return `${xyz} | ${moveLabel} | pose inherited`;
  }
  return `${xyz} | ${moveLabel} | R ${point.rollDeg.toFixed(1)} P ${point.pitchDeg.toFixed(1)} Y ${point.yawDeg.toFixed(1)}`;
}

function toPoint3(value: unknown): Point3 | null {
  if (Array.isArray(value) && value.length >= 3) {
    const [x, y, z] = value;
    const nx = Number(x);
    const ny = Number(y);
    const nz = Number(z);
    if (Number.isFinite(nx) && Number.isFinite(ny) && Number.isFinite(nz)) {
      return { x: nx, y: ny, z: nz };
    }
  } else if (
    typeof value === "object" &&
    value !== null &&
    "x" in value &&
    "y" in value &&
    "z" in value
  ) {
    const vector = value as { x: unknown; y: unknown; z: unknown };
    const nx = Number(vector.x);
    const ny = Number(vector.y);
    const nz = Number(vector.z);
    if (Number.isFinite(nx) && Number.isFinite(ny) && Number.isFinite(nz)) {
      return { x: nx, y: ny, z: nz };
    }
  }
  return null;
}

function coercePointList(value: unknown): Point3[] {
  if (!Array.isArray(value)) {
    return [];
  }
  return value
    .map((entry) => toPoint3(entry))
    .filter((point): point is Point3 => point !== null);
}

function pointDistanceSquared(a: Point3, b: Point3): number {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  const dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

function trimCartesianPathToAuthoredRange(
  cartesianPath: Point3[],
  waypoints: PoseWaypoint[],
): Point3[] {
  if (cartesianPath.length < 2 || waypoints.length === 0) {
    return cartesianPath;
  }
  const firstWaypoint = waypoints[0];
  const lastWaypoint = waypoints[waypoints.length - 1];

  let startIndex = 0;
  let bestStartDistance = Number.POSITIVE_INFINITY;
  for (let index = 0; index < cartesianPath.length; index += 1) {
    const distance = pointDistanceSquared(cartesianPath[index], firstWaypoint);
    if (distance < bestStartDistance) {
      bestStartDistance = distance;
      startIndex = index;
    }
  }

  let endIndex = cartesianPath.length - 1;
  let bestEndDistance = Number.POSITIVE_INFINITY;
  for (let index = startIndex; index < cartesianPath.length; index += 1) {
    const distance = pointDistanceSquared(cartesianPath[index], lastWaypoint);
    if (distance <= bestEndDistance) {
      bestEndDistance = distance;
      endIndex = index;
    }
  }

  if (startIndex === 0 && endIndex === cartesianPath.length - 1) {
    return cartesianPath;
  }
  const trimmedPath = cartesianPath.slice(startIndex, endIndex + 1);
  return trimmedPath.length > 0 ? trimmedPath : cartesianPath;
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

function coerceOrientationDeg(value: unknown): {
  rollDeg: number | null;
  pitchDeg: number | null;
  yawDeg: number | null;
} {
  let rollRaw: unknown = null;
  let pitchRaw: unknown = null;
  let yawRaw: unknown = null;
  if (Array.isArray(value) && value.length >= 3) {
    [rollRaw, pitchRaw, yawRaw] = value;
  } else if (typeof value === "object" && value !== null) {
    const orientation = value as Record<string, unknown>;
    rollRaw = orientation.roll ?? orientation.x ?? null;
    pitchRaw = orientation.pitch ?? orientation.y ?? null;
    yawRaw = orientation.yaw ?? orientation.z ?? null;
  }
  const rollDeg = rollRaw == null ? null : Number(rollRaw);
  const pitchDeg = pitchRaw == null ? null : Number(pitchRaw);
  const yawDeg = yawRaw == null ? null : Number(yawRaw);
  return {
    rollDeg: Number.isFinite(rollDeg) ? rollDeg : null,
    pitchDeg: Number.isFinite(pitchDeg) ? pitchDeg : null,
    yawDeg: Number.isFinite(yawDeg) ? yawDeg : null,
  };
}

function toPoseWaypoint(value: unknown): PoseWaypoint | null {
  const point = toPoint3(value);
  if (!point) {
    return null;
  }
  const record = typeof value === "object" && value !== null ? (value as Record<string, unknown>) : null;
  const orientation = coerceOrientationDeg(
    record?.orientation_euler_deg ??
      record?.orientationEulerDeg ??
      (record
        ? {
            roll: record.rollDeg ?? record.roll_deg ?? null,
            pitch: record.pitchDeg ?? record.pitch_deg ?? null,
            yaw: record.yawDeg ?? record.yaw_deg ?? null,
          }
        : null),
  );
  const moveTypeRaw =
    record?.move_type ??
    record?.moveType ??
    record?.motion_type ??
    record?.motionType;
  const moveType: WaypointMoveType =
    moveTypeRaw === "joint" || moveTypeRaw === "home" ? moveTypeRaw : "linear";
  return {
    ...point,
    ...orientation,
    moveType,
  };
}

export function coercePoseWaypointList(value: unknown): PoseWaypoint[] {
  if (!Array.isArray(value)) {
    return [];
  }
  return value
    .map((entry) => toPoseWaypoint(entry))
    .filter((point): point is PoseWaypoint => point !== null);
}

function coerceStringList(value: unknown): string[] {
  if (!Array.isArray(value)) {
    return [];
  }
  return value
    .map((entry) => (typeof entry === "string" ? entry.trim() : ""))
    .filter((entry) => entry.length > 0);
}

function coerceFiniteNumber(value: unknown): number | undefined {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : undefined;
}

function coerceResiduals(value: unknown): PlannerDiagnosticsResiduals | undefined {
  if (!value || typeof value !== "object") {
    return undefined;
  }
  const record = value as Record<string, unknown>;
  const residuals: PlannerDiagnosticsResiduals = {};
  const maxJointStepRad = coerceFiniteNumber(record.max_joint_step_rad ?? record.maxJointStepRad);
  const jointLimitMarginRad =
    record.joint_limit_margin_rad === null
      ? null
      : coerceFiniteNumber(record.joint_limit_margin_rad ?? record.jointLimitMarginRad);
  const cartesianResidualM = coerceFiniteNumber(
    record.cartesian_residual_m ?? record.cartesianResidualM,
  );
  const orientationResidualDeg = coerceFiniteNumber(
    record.orientation_residual_deg ?? record.orientationResidualDeg,
  );
  const violatingJointIndex = coerceFiniteNumber(
    record.violating_joint_index ?? record.violatingJointIndex,
  );
  const violatingJointMarginRad = coerceFiniteNumber(
    record.violating_joint_margin_rad ?? record.violatingJointMarginRad,
  );
  const violatingPoseIndex = coerceFiniteNumber(
    record.violating_pose_index ?? record.violatingPoseIndex,
  );
  const violatingJointCount = coerceFiniteNumber(
    record.violating_joint_count ?? record.violatingJointCount,
  );
  const jumpPoseIndex = coerceFiniteNumber(record.jump_pose_index ?? record.jumpPoseIndex);
  const jumpJointIndex = coerceFiniteNumber(record.jump_joint_index ?? record.jumpJointIndex);
  const jumpJointPreviousRad =
    record.jump_joint_previous_rad === null
      ? null
      : coerceFiniteNumber(record.jump_joint_previous_rad ?? record.jumpJointPreviousRad);
  const jumpJointCurrentRad =
    record.jump_joint_current_rad === null
      ? null
      : coerceFiniteNumber(record.jump_joint_current_rad ?? record.jumpJointCurrentRad);
  const jumpJointRawStepRad =
    record.jump_joint_raw_step_rad === null
      ? null
      : coerceFiniteNumber(record.jump_joint_raw_step_rad ?? record.jumpJointRawStepRad);
  const jumpJointWrappedStepRad =
    record.jump_joint_wrapped_step_rad === null
      ? null
      : coerceFiniteNumber(record.jump_joint_wrapped_step_rad ?? record.jumpJointWrappedStepRad);
  const jumpContext =
    typeof record.jump_context === "string"
      ? record.jump_context.trim()
      : typeof record.jumpContext === "string"
        ? record.jumpContext.trim()
        : "";
  const stepSource =
    typeof record.step_source === "string"
      ? record.step_source.trim()
      : typeof record.stepSource === "string"
        ? record.stepSource.trim()
        : "";
  if (maxJointStepRad !== undefined) residuals.maxJointStepRad = maxJointStepRad;
  if (jointLimitMarginRad !== undefined || record.joint_limit_margin_rad === null) {
    residuals.jointLimitMarginRad = jointLimitMarginRad ?? null;
  }
  if (cartesianResidualM !== undefined) residuals.cartesianResidualM = cartesianResidualM;
  if (orientationResidualDeg !== undefined) residuals.orientationResidualDeg = orientationResidualDeg;
  if (violatingJointIndex !== undefined) residuals.violatingJointIndex = violatingJointIndex;
  if (violatingJointMarginRad !== undefined) residuals.violatingJointMarginRad = violatingJointMarginRad;
  if (violatingPoseIndex !== undefined) residuals.violatingPoseIndex = violatingPoseIndex;
  if (violatingJointCount !== undefined) residuals.violatingJointCount = violatingJointCount;
  if (jumpPoseIndex !== undefined) residuals.jumpPoseIndex = jumpPoseIndex;
  if (jumpJointIndex !== undefined) residuals.jumpJointIndex = jumpJointIndex;
  if (jumpJointPreviousRad !== undefined || record.jump_joint_previous_rad === null) {
    residuals.jumpJointPreviousRad = jumpJointPreviousRad ?? null;
  }
  if (jumpJointCurrentRad !== undefined || record.jump_joint_current_rad === null) {
    residuals.jumpJointCurrentRad = jumpJointCurrentRad ?? null;
  }
  if (jumpJointRawStepRad !== undefined || record.jump_joint_raw_step_rad === null) {
    residuals.jumpJointRawStepRad = jumpJointRawStepRad ?? null;
  }
  if (jumpJointWrappedStepRad !== undefined || record.jump_joint_wrapped_step_rad === null) {
    residuals.jumpJointWrappedStepRad = jumpJointWrappedStepRad ?? null;
  }
  if (jumpContext) residuals.jumpContext = jumpContext;
  if (stepSource) residuals.stepSource = stepSource;
  return Object.keys(residuals).length > 0 ? residuals : undefined;
}

function coerceRecoveryAttempt(value: unknown): PlannerRecoveryAttempt | null {
  if (!value || typeof value !== "object") {
    return null;
  }
  const record = value as Record<string, unknown>;
  const strategy = typeof record.strategy === "string" ? record.strategy.trim() : "";
  const reasonCode = typeof record.reason_code === "string" ? record.reason_code.trim() : "";
  const attempt: PlannerRecoveryAttempt = {};
  const suffixStartIndex = coerceFiniteNumber(
    record.suffix_start_index ?? record.suffixStartIndex,
  );
  const solvedPoints = coerceFiniteNumber(record.solved_points ?? record.solvedPoints);
  const elapsedMs =
    record.elapsed_ms === null
      ? null
      : coerceFiniteNumber(record.elapsed_ms ?? record.elapsedMs) ?? undefined;
  if (strategy) attempt.strategy = strategy;
  if (suffixStartIndex !== undefined) attempt.suffixStartIndex = suffixStartIndex;
  if (Array.isArray(record.seed_joint_sample)) {
    attempt.seedJointSample = record.seed_joint_sample.map((entry) => {
      if (entry === null) {
        return null;
      }
      const numeric = Number(entry);
      return Number.isFinite(numeric) ? numeric : null;
    });
  }
  if (solvedPoints !== undefined) attempt.solvedPoints = solvedPoints;
  if (elapsedMs !== undefined || record.elapsed_ms === null) attempt.elapsedMs = elapsedMs ?? null;
  if (reasonCode) attempt.reasonCode = reasonCode;
  if (typeof record.accepted === "boolean") attempt.accepted = record.accepted;
  const residuals = coerceResiduals(record.residuals);
  if (residuals) attempt.residuals = residuals;
  const rawJump = coerceResiduals(record.raw_jump ?? record.rawJump);
  if (rawJump) attempt.rawJump = rawJump;
  return Object.keys(attempt).length > 0 ? attempt : null;
}

function coerceRecoveryDiagnostics(value: unknown): PlannerRecoveryDiagnostics | undefined {
  if (!value || typeof value !== "object") {
    return undefined;
  }
  const record = value as Record<string, unknown>;
  const diagnostics: PlannerRecoveryDiagnostics = {};
  const kind = typeof record.kind === "string" ? record.kind.trim() : "";
  const strategy = typeof record.strategy === "string" ? record.strategy.trim() : "";
  const sourceAttempt =
    typeof record.source_attempt === "string"
      ? record.source_attempt.trim()
      : typeof record.sourceAttempt === "string"
        ? record.sourceAttempt.trim()
        : "";
  const stepSource =
    typeof record.step_source === "string"
      ? record.step_source.trim()
      : typeof record.stepSource === "string"
        ? record.stepSource.trim()
        : "";
  const reasonCode =
    typeof record.reason_code === "string"
      ? record.reason_code.trim()
      : typeof record.reasonCode === "string"
        ? record.reasonCode.trim()
        : "";
  const jumpPoseIndex = coerceFiniteNumber(record.jump_pose_index ?? record.jumpPoseIndex);
  const splitCount = coerceFiniteNumber(record.split_count ?? record.splitCount);
  const distanceM =
    record.distance_m === null
      ? null
      : coerceFiniteNumber(record.distance_m ?? record.distanceM) ?? undefined;
  const failureSegmentIndex = coerceFiniteNumber(
    record.failure_segment_index ?? record.failureSegmentIndex,
  );
  if (typeof record.used === "boolean") diagnostics.used = record.used;
  if (kind) diagnostics.kind = kind;
  if (strategy) diagnostics.strategy = strategy;
  if (sourceAttempt) diagnostics.sourceAttempt = sourceAttempt;
  if (stepSource) diagnostics.stepSource = stepSource;
  if (reasonCode) diagnostics.reasonCode = reasonCode;
  if (jumpPoseIndex !== undefined) diagnostics.jumpPoseIndex = jumpPoseIndex;
  if (splitCount !== undefined) diagnostics.splitCount = splitCount;
  if (distanceM !== undefined || record.distance_m === null) diagnostics.distanceM = distanceM ?? null;
  if (failureSegmentIndex !== undefined) diagnostics.failureSegmentIndex = failureSegmentIndex;
  if (Array.isArray(record.attempts)) {
    diagnostics.attempts = record.attempts
      .map((entry) => coerceRecoveryAttempt(entry))
      .filter((entry): entry is PlannerRecoveryAttempt => entry !== null);
  }
  if (Array.isArray(record.segments)) {
    diagnostics.segments = record.segments.filter(
      (entry): entry is Record<string, unknown> => typeof entry === "object" && entry !== null,
    );
  }
  return Object.keys(diagnostics).length > 0 ? diagnostics : undefined;
}

export function coercePlannerDiagnostics(value: unknown): PlannerDiagnostics | undefined {
  if (!value || typeof value !== "object") {
    return undefined;
  }
  const record = value as Record<string, unknown>;
  const diagnostics: PlannerDiagnostics = {};
  const reasonCode =
    typeof record.reason_code === "string"
      ? record.reason_code.trim()
      : typeof record.reasonCode === "string"
        ? record.reasonCode.trim()
        : "";
  const seedUsed =
    typeof record.seed_used === "string"
      ? record.seed_used.trim()
      : typeof record.seedUsed === "string"
        ? record.seedUsed.trim()
        : "";
  const attempt =
    typeof record.attempt === "string"
      ? record.attempt.trim()
      : "";
  const fallbackLevel = coerceFiniteNumber(record.fallback_level ?? record.fallbackLevel);
  const profileRevision = coerceFiniteNumber(record.profile_revision ?? record.profileRevision);
  if (reasonCode) diagnostics.reasonCode = reasonCode;
  if (seedUsed) diagnostics.seedUsed = seedUsed;
  if (attempt) diagnostics.attempt = attempt;
  if (fallbackLevel !== undefined) diagnostics.fallbackLevel = fallbackLevel;
  if (profileRevision !== undefined) diagnostics.profileRevision = profileRevision;
  if (typeof record.smoothing_applied === "boolean") {
    diagnostics.smoothingApplied = record.smoothing_applied;
  } else if (typeof record.smoothingApplied === "boolean") {
    diagnostics.smoothingApplied = record.smoothingApplied;
  }
  if (typeof record.branch_anchor_available === "boolean") {
    diagnostics.branchAnchorAvailable = record.branch_anchor_available;
  } else if (typeof record.branchAnchorAvailable === "boolean") {
    diagnostics.branchAnchorAvailable = record.branchAnchorAvailable;
  }
  const residuals = coerceResiduals(record.residuals);
  if (residuals) diagnostics.residuals = residuals;
  const rawJump = coerceResiduals(record.raw_jump ?? record.rawJump);
  if (rawJump) diagnostics.rawJump = rawJump;
  const recovery = coerceRecoveryDiagnostics(record.recovery);
  if (recovery) diagnostics.recovery = recovery;
  const splitRecovery = coerceRecoveryDiagnostics(record.split_recovery ?? record.splitRecovery);
  if (splitRecovery) diagnostics.splitRecovery = splitRecovery;
  return Object.keys(diagnostics).length > 0 ? diagnostics : undefined;
}

function deriveWaypointsFromTrajectory(trajectory: TrajectoryFile): PoseWaypoint[] {
  if (!trajectory?.moves || !Array.isArray(trajectory.moves)) {
    return [];
  }
  const points: PoseWaypoint[] = [];
  trajectory.moves.forEach((move) => {
    if (move?.command === "move_absolute" || move?.command === "move") {
      const point = toPoint3(move.vector ?? null);
      if (point) {
        const orientation = coerceOrientationDeg(move.orientation_euler_deg ?? null);
        points.push({
          ...point,
          ...orientation,
          moveType: move.command === "move" ? "joint" : "linear",
        });
      }
    } else if (move?.command === "home") {
      const point = toPoint3(move.vector ?? null) ?? { x: 0, y: 0, z: 0 };
      points.push({
        ...point,
        rollDeg: null,
        pitchDeg: null,
        yawDeg: null,
        moveType: "home",
      });
    }
  });
  return points;
}

function buildPreviewPlan(
  name: string,
  trajectory: TrajectoryFile,
  explicitPath?: unknown,
  explicitWaypoints?: unknown,
  explicitWarnings?: unknown,
  explicitPlannerDiagnostics?: unknown,
): PreviewPlan {
  const fallbackWaypoints = deriveWaypointsFromTrajectory(trajectory);
  const explicitWaypointsList = coercePoseWaypointList(explicitWaypoints);
  const waypoints =
    explicitWaypointsList.length > 0
      ? explicitWaypointsList
      : fallbackWaypoints;
  const cartesianPath = trimCartesianPathToAuthoredRange(
    coercePointList(explicitPath),
    waypoints,
  );
  const pathPoints =
    cartesianPath.length > 0
      ? cartesianPath
      : waypoints.length > 0
        ? waypoints
        : fallbackWaypoints;
  const planningWarnings = coerceStringList(explicitWarnings);
  const plannerDiagnostics = coercePlannerDiagnostics(explicitPlannerDiagnostics);

  return {
    name,
    trajectory,
    pathPoints: pathPoints.map(({ x, y, z }) => transformToScenePoint({ x, y, z })),
    waypoints: (waypoints.length > 0 ? waypoints : fallbackWaypoints).map(
      ({ x, y, z, rollDeg, pitchDeg, yawDeg, moveType }) => ({
        ...transformToScenePoint({ x, y, z }),
        rollDeg,
        pitchDeg,
        yawDeg,
        moveType,
      }),
    ),
    planningWarnings,
    plannerDiagnostics,
  };
}

export function previewFromPlannerPayload(payload: any): {
  plan: PreviewPlan;
  waypoints: PoseWaypoint[];
} {
  const name =
    typeof payload?.name === "string" && payload.name.trim()
      ? payload.name.trim()
      : "__planner_preview__";
  const trajectory =
    payload && typeof payload.trajectory === "object" && payload.trajectory
      ? (payload.trajectory as TrajectoryFile)
      : ({ moves: [] } as TrajectoryFile);
  const plan = buildPreviewPlan(
    name,
    trajectory,
    payload?.cartesian_path,
    payload?.waypoints,
    payload?.planning_warnings,
    payload?.planner_diagnostics,
  );
  const waypoints = plan.waypoints;
  return { plan, waypoints };
}

export function previewFromTrajectoryDetail(
  name: string,
  trajectory: TrajectoryFile,
): PreviewPlan {
  return buildPreviewPlan(name, trajectory);
}

export function buildProgramTree({
  plan,
  weldSegments = [],
  weldType,
  viewMode = "grouped",
}: BuildProgramTreeInput): ProgramNode | null {
  if (!plan && (!Array.isArray(weldSegments) || weldSegments.length === 0)) {
    return null;
  }

  const trajectory = plan?.trajectory;
  const moves = Array.isArray(trajectory?.moves) ? trajectory.moves : [];
  const defaultFocusPanel: "trajectory" | "weld" =
    trajectory && typeof trajectory.weld === "object" && trajectory.weld !== null
      ? "weld"
      : "trajectory";
  const pathPoints = Array.isArray(plan?.pathPoints) ? plan.pathPoints : [];
  const controlPoints = Array.isArray(plan?.waypoints) ? plan.waypoints : [];
  const pathPointCount = pathPoints.length;

  const exactPathNodes: ProgramNode[] = pathPoints.map((point, index) => ({
    id: `path_sample_${index}`,
    type: "waypoint",
    label: `Path Sample ${index + 1}`,
    subtitle: `${point.x.toFixed(4)}, ${point.y.toFixed(4)}, ${point.z.toFixed(4)}`,
    badge: `${index + 1}`,
    focus: {
      openPanel: defaultFocusPanel,
      pathRange: {
        start: index,
        end: Math.min(pathPointCount - 1, index + 1),
      },
    },
    children: [],
  }));

  const controlPointNodes: ProgramNode[] = controlPoints.map((point, index) => ({
    id: `control_point_${index}`,
    type: "waypoint",
    label: `Control Point ${index + 1}`,
    subtitle: poseSubtitle(point),
    badge: `${index + 1}`,
    focus: {
      openPanel: defaultFocusPanel,
      waypointIndices: [index],
    },
    children: [],
  }));

  const commandNodes: ProgramNode[] = moves.map((move, moveIndex) => {
    const command = String(move?.command ?? "").trim() || "unknown";
    const focus: ProgramNodeFocus = {
      openPanel: defaultFocusPanel,
      moveIndex,
    };
    const commandNode: ProgramNode = {
      id: `move_${moveIndex}`,
      type: "move",
      label: command,
      subtitle: `Move ${moveIndex + 1}`,
      badge: `#${moveIndex + 1}`,
      focus,
      children: [],
    };
    if (command === "move_absolute") {
      const movePoint = toPoint3(move.vector ?? null);
      if (movePoint) {
        const moveOrientation = coerceOrientationDeg(move.orientation_euler_deg ?? null);
        commandNode.children.push({
          id: `move_${moveIndex}_endpoint`,
          type: "waypoint",
          label: "Move Endpoint",
          subtitle: poseSubtitle({ ...movePoint, ...moveOrientation, moveType: "linear" }),
          focus: {
            openPanel: defaultFocusPanel,
            moveIndex,
          },
          children: [],
        });
      }
    } else if (command === "move" || command === "home") {
      const movePoint = toPoint3(move.vector ?? null);
      const moveOrientation = coerceOrientationDeg(move.orientation_euler_deg ?? null);
      if (movePoint) {
        commandNode.children.push({
          id: `move_${moveIndex}_endpoint`,
          type: "waypoint",
          label: command === "home" ? "Home Endpoint" : "Joint Endpoint",
          subtitle: poseSubtitle({
            ...movePoint,
            ...moveOrientation,
            moveType: command === "home" ? "home" : "joint",
          }),
          focus: {
            openPanel: defaultFocusPanel,
            moveIndex,
          },
          children: [],
        });
      }
    }
    return commandNode;
  });

  const groupedOperationNodes: ProgramNode[] = [];
  const waypointPathIndices = buildOrderedWaypointPathIndices(pathPoints, controlPoints);
  if (controlPointNodes.length > 0) {
    const groupedTrajectoryNodes: ProgramNode[] = [controlPointNodes[0]];
    const moveCountMatchesWaypoints = moves.length === controlPoints.length;
    for (let segmentIndex = 0; segmentIndex < controlPoints.length - 1; segmentIndex += 1) {
      const endControlPoint = controlPoints[segmentIndex + 1];
      const relatedMoveIndex = moveCountMatchesWaypoints
        ? segmentIndex + 1
        : segmentIndex < moves.length
          ? segmentIndex
          : null;
      const relatedCommand =
        relatedMoveIndex !== null && relatedMoveIndex >= 0 && relatedMoveIndex < moves.length
          ? moves[relatedMoveIndex]
          : null;
      const commandLabel = String(relatedCommand?.command ?? "")
        .trim()
        .replace(/_/g, " ");
      const rawStart = waypointPathIndices[segmentIndex] ?? 0;
      const rawEnd = waypointPathIndices[segmentIndex + 1] ?? rawStart;
      let start = Math.max(0, Math.min(pathPointCount - 1, rawStart));
      let end = Math.max(start, Math.min(pathPointCount - 1, rawEnd));
      if (pathPointCount >= 2 && start === end) {
        if (end < pathPointCount - 1) {
          end += 1;
        } else if (start > 0) {
          start -= 1;
        }
      }
      const pathSampleNodes =
        pathPointCount > 0
          ? pathPoints.slice(start, end + 1).map((point, sampleOffset) => {
              const globalIndex = start + sampleOffset;
              return {
                id: `move_group_${segmentIndex}_path_sample_${globalIndex}`,
                type: "waypoint" as const,
                label: `Path Sample ${globalIndex + 1}`,
                subtitle: `${point.x.toFixed(4)}, ${point.y.toFixed(4)}, ${point.z.toFixed(4)}`,
                badge: `${sampleOffset + 1}`,
                focus: {
                  openPanel: defaultFocusPanel,
                  pathRange: {
                    start: globalIndex,
                    end: Math.min(end, globalIndex + 1),
                  },
                },
                children: [],
              };
            })
          : [];
      const moveGroupFocus: ProgramNodeFocus = {
        openPanel: defaultFocusPanel,
        waypointIndices: [segmentIndex, segmentIndex + 1],
        pathRange:
          pathPointCount >= 2
            ? {
                start,
                end,
              }
            : undefined,
        ...(typeof relatedMoveIndex === "number" ? { moveIndex: relatedMoveIndex } : {}),
      };
      groupedTrajectoryNodes.push({
        id: `move_group_${segmentIndex}`,
        type: "operationGroup",
        label: `Move ${segmentIndex + 1}`,
        subtitle: `P${segmentIndex + 1} -> P${segmentIndex + 2} | ${endControlPoint.moveType} move${commandLabel ? ` | ${commandLabel}` : ""}`,
        badge:
          pathSampleNodes.length > 0
            ? `${pathSampleNodes.length}`
            : commandLabel || `${segmentIndex + 1}`,
        focus: moveGroupFocus,
        children:
          pathSampleNodes.length > 0
            ? pathSampleNodes
            : typeof relatedMoveIndex === "number" && commandNodes[relatedMoveIndex]
              ? [commandNodes[relatedMoveIndex]]
              : [],
      });
      groupedTrajectoryNodes.push(controlPointNodes[segmentIndex + 1]);
    }
    groupedOperationNodes.push({
      id: "op_motion",
      type: "operationGroup",
      label: "Trajectory Sequence",
      subtitle: `${controlPointNodes.length} control point(s) • ${Math.max(0, controlPointNodes.length - 1)} move group(s)`,
      badge: `${groupedTrajectoryNodes.length}`,
      children: groupedTrajectoryNodes,
    });
  } else if (commandNodes.length > 0) {
    groupedOperationNodes.push({
      id: "op_motion",
      type: "operationGroup",
      label:
        exactPathNodes.length > 0
          ? "Controller Commands (Reference)"
          : "Controller Commands",
      badge: `${commandNodes.length}`,
      children: commandNodes,
    });
  }

  const chronologicalNodes: ProgramNode[] = [];
  if (exactPathNodes.length > 0) {
    chronologicalNodes.push({
      id: "op_chronological",
      type: "operationGroup",
      label: "Execution Path (Exact)",
      badge: `${exactPathNodes.length}`,
      children: exactPathNodes,
    });
  } else if (commandNodes.length > 0) {
    chronologicalNodes.push({
      id: "op_chronological",
      type: "operationGroup",
      label: "Execution Order",
      badge: `${commandNodes.length}`,
      children: commandNodes,
    });
  }

  if (Array.isArray(weldSegments) && weldSegments.length > 0) {
    const weldNodes: ProgramNode[] = weldSegments.map((segment, index) => {
      const segmentWeldType = typeof segment.weldType === "string" ? segment.weldType : weldType;
      return {
        id: `weld_segment_${index}_${toNodeId(segment.edgeId)}`,
        type: "weldSegment",
        label: `Segment ${index + 1}`,
        subtitle: segment.edgeId,
        badge: segmentWeldType ?? "weld",
        focus: {
          openPanel: "weld",
          weldSegmentEdgeId: segment.edgeId,
        },
        children: [],
      };
    });
    const weldGroup: ProgramNode = {
      id: "op_weld",
      type: "operationGroup",
      label: "Weld Features",
      badge: `${weldNodes.length}`,
      children: weldNodes,
    };
    groupedOperationNodes.push(weldGroup);
    chronologicalNodes.push(weldGroup);
  }

  const distinctWeldTypes = Array.isArray(weldSegments)
    ? Array.from(
        new Set(
          weldSegments
            .map((segment) => (typeof segment.weldType === "string" ? segment.weldType.trim() : ""))
            .filter((value) => value.length > 0),
        ),
      )
    : [];
  const setupSubtitle =
    distinctWeldTypes.length > 1
      ? "mixed weld workflow"
      : distinctWeldTypes.length === 1
        ? `${distinctWeldTypes[0]} workflow`
        : weldType
          ? `${weldType} workflow`
          : "Robot sequence";

  const operationGroups =
    viewMode === "chronological" ? chronologicalNodes : groupedOperationNodes;

  return {
    id: "program_root",
    type: "program",
    label: plan?.name ?? "Program",
    subtitle:
      pathPointCount > 0
        ? `${moves.length} move(s) • ${pathPointCount} path sample(s)`
        : `${moves.length} move(s)`,
    children: [
      {
        id: "setup_primary",
        type: "setupGroup",
        label: "Setup",
        subtitle: setupSubtitle,
        children: operationGroups,
      },
    ],
  };
}

export function encodePointsForApi(points: Point3[]): any[] {
  return points.map((point) => {
    const world = transformFromScenePoint(point);
    return {
      x: Number(world.x),
      y: Number(world.y),
      z: Number(world.z),
    };
  });
}

export function encodePoseWaypointsForApi(waypoints: PoseWaypoint[]): any[] {
  return waypoints.map((waypoint) => {
    const world = transformFromScenePoint(waypoint);
    const payload: Record<string, unknown> = {
      x: Number(world.x),
      y: Number(world.y),
      z: Number(world.z),
    };
    if (
      waypoint.rollDeg !== null &&
      waypoint.pitchDeg !== null &&
      waypoint.yawDeg !== null
    ) {
      payload.orientation_euler_deg = {
        roll: Number(waypoint.rollDeg),
        pitch: Number(waypoint.pitchDeg),
        yaw: Number(waypoint.yawDeg),
      };
    }
    payload.move_type = waypoint.moveType;
    return payload;
  });
}

export function transformToScenePoint(point: Point3): Point3 {
  return {
    x: point.x,
    y: point.y,
    z: point.z,
  };
}

export function transformFromScenePoint(point: Point3): Point3 {
  return {
    x: point.x,
    y: point.y,
    z: point.z,
  };
}
