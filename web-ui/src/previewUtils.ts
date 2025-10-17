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
  moves: TrajectoryMove[];
};

export type PreviewPlan = {
  name: string;
  trajectory: TrajectoryFile;
  pathPoints: Point3[];
  waypoints: Point3[];
};

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

  return {
    name,
    trajectory,
    pathPoints: pathPoints.map(({ x, y, z }) => transformToScenePoint({ x, y, z })),
    waypoints: (waypoints.length > 0 ? waypoints : fallbackWaypoints).map(
      ({ x, y, z }) => transformToScenePoint({ x, y, z }),
    ),
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
    y: point.z,
    z: -point.y,
  };
}

export function transformFromScenePoint(point: Point3): Point3 {
  return {
    x: point.x,
    y: -point.z,
    z: point.y,
  };
}
