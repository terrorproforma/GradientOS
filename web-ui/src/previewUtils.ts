export type Point3 = { x: number; y: number; z: number };

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
  waypoints: Point3[];
  planningWarnings?: string[];
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

function coerceStringList(value: unknown): string[] {
  if (!Array.isArray(value)) {
    return [];
  }
  return value
    .map((entry) => (typeof entry === "string" ? entry.trim() : ""))
    .filter((entry) => entry.length > 0);
}

function deriveWaypointsFromTrajectory(trajectory: TrajectoryFile): Point3[] {
  if (!trajectory?.moves || !Array.isArray(trajectory.moves)) {
    return [];
  }
  const points: Point3[] = [];
  trajectory.moves.forEach((move) => {
    if (move?.command === "move_absolute") {
      const point = toPoint3(move.vector ?? null);
      if (point) {
        points.push(point);
      }
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
): PreviewPlan {
  const fallbackWaypoints = deriveWaypointsFromTrajectory(trajectory);
  const explicitWaypointsList = coercePointList(explicitWaypoints);
  const waypoints =
    explicitWaypointsList.length > 0
      ? explicitWaypointsList
      : fallbackWaypoints;
  const cartesianPath = coercePointList(explicitPath);
  const pathPoints =
    cartesianPath.length > 0
      ? cartesianPath
      : waypoints.length > 0
        ? waypoints
        : fallbackWaypoints;
  const planningWarnings = coerceStringList(explicitWarnings);

  return {
    name,
    trajectory,
    pathPoints: pathPoints.map(({ x, y, z }) => transformToScenePoint({ x, y, z })),
    waypoints: (waypoints.length > 0 ? waypoints : fallbackWaypoints).map(
      ({ x, y, z }) => transformToScenePoint({ x, y, z }),
    ),
    planningWarnings,
  };
}

export function previewFromPlannerPayload(payload: any): {
  plan: PreviewPlan;
  waypoints: Point3[];
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
    subtitle: `${point.x.toFixed(4)}, ${point.y.toFixed(4)}, ${point.z.toFixed(4)}`,
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
        commandNode.children.push({
          id: `move_${moveIndex}_endpoint`,
          type: "waypoint",
          label: "Move Endpoint",
          subtitle: `${movePoint.x.toFixed(4)}, ${movePoint.y.toFixed(4)}, ${movePoint.z.toFixed(4)}`,
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
  if (exactPathNodes.length > 0) {
    groupedOperationNodes.push({
      id: "op_path_exact",
      type: "operationGroup",
      label: "Exact Path Samples",
      badge: `${exactPathNodes.length}`,
      children: exactPathNodes,
    });
  }
  if (controlPointNodes.length > 0) {
    groupedOperationNodes.push({
      id: "op_control_points",
      type: "operationGroup",
      label: "Control Points",
      badge: `${controlPointNodes.length}`,
      children: controlPointNodes,
    });
  }
  if (commandNodes.length > 0) {
    groupedOperationNodes.push({
      id: exactPathNodes.length > 0 ? "op_commands_meta" : "op_commands",
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
