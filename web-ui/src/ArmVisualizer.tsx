/*
 * ARM DISPLAY GUARDRAILS: if the robot disappears, assume it is a regression in this viewer or its
 * surrounding stage/dev-server wiring first, not "missing files." The Gradient-05 source STL files live at
 * `.../GradientOS/robots/gradient-05/stl-files/`, and the synced web-served copies live at
 * `.../GradientOS/web-ui/public/assets/robots/gradient-05/stl-files/`; the URDF served by this UI points
 * at those STL names. Common ways to break the arm display are: unmounting/replacing `ArmVisualizer` during
 * stage/view toggles, changing `resolveRobotUrdfConfig()` or Vite public-asset serving so URDF/STLs resolve to
 * the SPA shell instead of real bytes, changing `robotId` selection/hydration, or changing camera grounding /
 * focus behavior so the grid renders but the arm is effectively out of frame. Before making layout or asset-path
 * changes here, preserve the existing visualizer lifecycle, treat a missing arm as a regression you introduced,
 * and verify live URDF/STL HTTP responses plus actual arm visibility after the change.
 */
import {
  forwardRef,
  type ForwardedRef,
  useEffect,
  useImperativeHandle,
  useRef,
  useState,
} from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import occtWasmUrl from "occt-import-js/dist/occt-import-js.wasm?url";
import type { Point3 } from "./previewUtils";

declare const __GRADIENT_PUBLIC_DIR_FS__: string;

type ArmVisualizerProps = {
  robotId?: string | null;
  activeTool?: {
    tool_id: string;
    display_name: string;
    offset: {
      position_mm: { x: number; y: number; z: number };
      rotation_deg: { x: number; y: number; z: number };
    };
    mesh?: {
      asset_path: string;
      scale: number;
      position_mm?: { x: number; y: number; z: number };
      rotation_deg?: { x: number; y: number; z: number };
    } | null;
  } | null;
  joints?: number[];
  showBoundingBox: boolean;
  selectionMode: boolean;
  onPointSelected?: (point: { x: number; y: number; z: number }) => void;
  weldSelectionMode?: boolean;
  topologyEdges?: TopologyEdgeOverlay[];
  selectedTopologyEdgeId?: string | null;
  selectedTopologyEdgeIds?: string[];
  onTopologyEdgeSelected?: (edgeId: string) => void;
  weldActive?: boolean;
  weldIndicatorPoint?: Point3 | null;
  weldStartPoint?: Point3 | null;
  weldStopPoint?: Point3 | null;
  weldSegmentPoints?: Point3[];
  showEndEffectorFrame?: boolean;
  weldAnglePreview?: {
    workAngleDeg: number;
    travelAngleDeg: number;
    spinAngleDeg: number;
  } | null;
  weldGhostJoints?: number[] | null;
  pathPoints?: Point3[];
  waypoints?: Point3[];
  highlightPathRange?: { start: number; end: number } | null;
  highlightWaypointIndices?: number[];
  stepFile?: File | null;
  stepTransform?: StepTransform;
  onStepStatusChange?: (status: StepLoadStatus) => void;
};

const GRID_CELL_SIZE = 0.05; // 10 cm per square
const GRID_CELLS_PER_SIDE = 80; // spans 4 m total (enough workspace)
const GRID_SIZE = GRID_CELL_SIZE * GRID_CELLS_PER_SIDE;
const ROBOT_SCALE = 1; // make the robot look bit bigger
const BOUNDING_MARKER_COLORS = [
  0xf87171,
  0xfbbf24,
  0x34d399,
  0x38bdf8,
  0xa855f7,
  0xf472b6,
  0x22d3ee,
  0xf97316,
] as const;
const STEP_FALLBACK_COLOR = 0x94a3b8;
const WORLD_AXIS_LENGTH = 0.26;
const WORLD_AXIS_RADIUS = 0.0025; // 2.5 mm
const WORLD_AXIS_HEAD_LENGTH = 0.03;
const WORLD_AXIS_HEAD_RADIUS = 0.008;
const STEP_LOCAL_AXIS_LENGTH = 0.18;
const STEP_LOCAL_AXIS_RADIUS = 0.002;
const STEP_LOCAL_AXIS_HEAD_LENGTH = 0.022;
const STEP_LOCAL_AXIS_HEAD_RADIUS = 0.006;
const ORIENTATION_WIDGET_SIZE_PX = 120;
const ORIENTATION_WIDGET_MIN_SIZE_PX = 84;
const ORIENTATION_WIDGET_MARGIN_PX = 16;
const DEFAULT_STEP_TRANSFORM: StepTransform = {
  position: { x: 0, y: 0, z: 0 },
  rotationDeg: { x: 0, y: 0, z: 0 },
  scale: 1,
};
const TOPOLOGY_EDGE_DEFAULT_COLOR = 0x22d3ee;
const TOPOLOGY_EDGE_HOVER_COLOR = 0xfacc15;
const TOPOLOGY_EDGE_SELECTED_COLOR = 0x22c55e;
const TOPOLOGY_EDGE_PICK_RADIUS_M = 0.0005; // 0.5 mm
const TOPOLOGY_EDGE_SELECTED_RADIUS_M = 0.0006;
const WELD_ANGLE_PREVIEW_WORK_LIMIT_DEG = 89;
const WELD_ANGLE_PREVIEW_TRAVEL_LIMIT_DEG = 89;
const WELD_ANGLE_PREVIEW_ROLL_LIMIT_DEG = 180;
const WELD_ANGLE_PREVIEW_NORMAL_LIMIT_DEG = 180;
const WELD_ANGLE_PREVIEW_VECTOR_LENGTH_M = 0.065;
const WELD_ANGLE_PREVIEW_ARC_RADIUS_BASE_M = 0.018;
const WELD_ANGLE_PREVIEW_LABEL_SCALE = 0.032;
const WELD_ANGLE_PREVIEW_LABEL_OFFSET_M = 0.014;
const LIVE_BOUNDS_REFRESH_INTERVAL_MS = 20;
const PREVIEW_CP_LABEL_SCALE = 0.018;
const PREVIEW_MOVE_LABEL_SCALE = 0.017;
const PREVIEW_CP_LABEL_OFFSET = new THREE.Vector3(0.012, 0.006, 0.012);
const PREVIEW_MOVE_LABEL_OFFSET = new THREE.Vector3(0.0, 0.0, 0.016);

function clampNumber(value: number, min: number, max: number): number {
  const safeValue = Number.isFinite(value) ? value : 0;
  return Math.max(min, Math.min(max, safeValue));
}

function pointDistanceSquared(a: THREE.Vector3, b: THREE.Vector3): number {
  return a.distanceToSquared(b);
}

function buildOrderedWaypointPathIndices(pathPoints: THREE.Vector3[], waypoints: THREE.Vector3[]): number[] {
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

function normalizeVector(
  vector: THREE.Vector3,
  fallback: THREE.Vector3,
): THREE.Vector3 {
  if (vector.lengthSq() <= 1e-12) {
    return fallback.clone().normalize();
  }
  return vector.clone().normalize();
}

function createAngleArc(
  origin: THREE.Vector3,
  startDirection: THREE.Vector3,
  axis: THREE.Vector3,
  angleRad: number,
  radius: number,
  color: number,
): THREE.Line | null {
  if (!Number.isFinite(angleRad) || Math.abs(angleRad) <= 1e-4) {
    return null;
  }
  const arcAxis = normalizeVector(axis, new THREE.Vector3(0, 0, 1));
  const start = normalizeVector(startDirection, new THREE.Vector3(1, 0, 0));
  const stepCount = Math.max(8, Math.min(64, Math.ceil(Math.abs(angleRad) / (Math.PI / 24))));
  const points: THREE.Vector3[] = [];
  for (let i = 0; i <= stepCount; i += 1) {
    const t = i / stepCount;
    const direction = start.clone().applyAxisAngle(arcAxis, angleRad * t).normalize();
    points.push(origin.clone().addScaledVector(direction, radius));
  }
  const geometry = new THREE.BufferGeometry().setFromPoints(points);
  const material = new THREE.LineBasicMaterial({
    color,
    transparent: true,
    opacity: 0.95,
    depthTest: false,
    depthWrite: false,
  });
  const line = new THREE.Line(geometry, material);
  line.renderOrder = 36;
  return line;
}

type OcctImporter = {
  ReadStepFile?: (
    content: Uint8Array,
    params?: Record<string, unknown> | null,
  ) => {
    success?: boolean;
    meshes?: Array<{
      name?: string;
      color?: number[];
      attributes?: {
        position?: { array?: number[] };
        normal?: { array?: number[] };
      };
      index?: { array?: number[] };
    }>;
  };
};

let occtImporterPromise: Promise<OcctImporter> | null = null;

export type StepTransform = {
  position: { x: number; y: number; z: number };
  rotationDeg: { x: number; y: number; z: number };
  scale: number;
};

export type StepLoadStatus = {
  state: "idle" | "loading" | "loaded" | "error";
  message: string;
};

export type TopologyEdgeOverlay = {
  id: string;
  partId?: string;
  points: Point3[];
};

type RobotAssetIndex = {
  defaultRobotId: string;
  robots: Record<string, { urdfPath: string }>;
};

function buildDevServedPublicAssetUrl(assetPath: string): string | null {
  const publicDirFs =
    typeof __GRADIENT_PUBLIC_DIR_FS__ === "string" ? __GRADIENT_PUBLIC_DIR_FS__.trim() : "";
  if (!publicDirFs) {
    return null;
  }
  const normalizedRoot = publicDirFs.replace(/\\/g, "/").replace(/\/+$/, "");
  const normalizedAssetPath = assetPath.startsWith("/") ? assetPath : `/${assetPath}`;
  return `/@fs${normalizedRoot}${normalizedAssetPath}`;
}

function buildActiveToolSignature(tool: ArmVisualizerProps["activeTool"]): string {
  if (!tool) {
    return "none";
  }
  const offsetPos = tool.offset?.position_mm ?? { x: 0, y: 0, z: 0 };
  const offsetRot = tool.offset?.rotation_deg ?? { x: 0, y: 0, z: 0 };
  const mesh = tool.mesh ?? null;
  const meshPos = mesh?.position_mm ?? { x: 0, y: 0, z: 0 };
  const meshRot = mesh?.rotation_deg ?? { x: 0, y: 0, z: 0 };
  return JSON.stringify({
    id: tool.tool_id,
    offsetPos,
    offsetRot,
    meshPath: mesh?.asset_path ?? "",
    meshScale: mesh?.scale ?? 1,
    meshPos,
    meshRot,
  });
}

async function loadOcctImporter(): Promise<OcctImporter> {
  if (!occtImporterPromise) {
    occtImporterPromise = import("occt-import-js").then(async (module) => {
      const init = (module as { default?: unknown }).default ?? module;
      if (typeof init !== "function") {
        throw new Error("STEP importer module is not callable.");
      }
      return (await (init as (opts?: Record<string, unknown>) => Promise<OcctImporter>)({
        locateFile: (path: string) => (path.endsWith(".wasm") ? occtWasmUrl : path),
      })) as OcctImporter;
    });
  }
  return occtImporterPromise;
}

function toThreeColor(color?: number[]): number {
  if (!Array.isArray(color) || color.length < 3) {
    return STEP_FALLBACK_COLOR;
  }
  const [r, g, b] = color;
  const max = Math.max(r, g, b);
  const normalized =
    max > 1
      ? [r / 255, g / 255, b / 255]
      : [Math.max(r, 0), Math.max(g, 0), Math.max(b, 0)];
  return new THREE.Color(normalized[0], normalized[1], normalized[2]).getHex();
}

function buildStepMesh(meshData: {
  name?: string;
  color?: number[];
  attributes?: {
    position?: { array?: number[] };
    normal?: { array?: number[] };
  };
  index?: { array?: number[] };
}): THREE.Mesh | null {
  const positions = meshData.attributes?.position?.array;
  if (!Array.isArray(positions) || positions.length < 9) {
    return null;
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(positions, 3),
  );
  const normals = meshData.attributes?.normal?.array;
  if (Array.isArray(normals) && normals.length === positions.length) {
    geometry.setAttribute(
      "normal",
      new THREE.Float32BufferAttribute(normals, 3),
    );
  } else {
    geometry.computeVertexNormals();
  }
  const indices = meshData.index?.array;
  if (Array.isArray(indices) && indices.length > 0) {
    geometry.setIndex(indices);
  }
  geometry.computeBoundingBox();
  const material = new THREE.MeshStandardMaterial({
    color: toThreeColor(meshData.color),
    metalness: 0.25,
    roughness: 0.6,
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.name = meshData.name ?? "step-mesh";
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function disposeObject3D(object: THREE.Object3D) {
  object.traverse((child) => {
    if ((child as THREE.Mesh).isMesh) {
      const mesh = child as THREE.Mesh;
      mesh.geometry.dispose();
      if (Array.isArray(mesh.material)) {
        mesh.material.forEach((material) => material.dispose());
      } else if (mesh.material) {
        mesh.material.dispose();
      }
      return;
    }
    if (child instanceof THREE.Line || child instanceof THREE.LineSegments) {
      child.geometry.dispose();
      if (Array.isArray(child.material)) {
        child.material.forEach((material) => material.dispose());
      } else {
        child.material.dispose();
      }
      return;
    }
    if (child instanceof THREE.Sprite) {
      const spriteMaterial = child.material as THREE.SpriteMaterial;
      spriteMaterial.map?.dispose();
      spriteMaterial.dispose();
      return;
    }
  });
}

function computeGroundingBoxFromBase(
  baseSource: THREE.Object3D,
  excludedSubtree?: THREE.Object3D | null,
): THREE.Box3 | null {
  const bounds = new THREE.Box3();
  const worldGeometryBounds = new THREE.Box3();
  const pending: THREE.Object3D[] = [baseSource];
  let hasGeometry = false;

  while (pending.length > 0) {
    const current = pending.pop();
    if (!current) {
      continue;
    }
    if (excludedSubtree && current === excludedSubtree) {
      continue;
    }

    const asMesh = current as THREE.Mesh;
    if (asMesh.isMesh) {
      const geometry = asMesh.geometry as THREE.BufferGeometry | undefined;
      if (geometry) {
        if (!geometry.boundingBox) {
          geometry.computeBoundingBox();
        }
        if (geometry.boundingBox) {
          worldGeometryBounds.copy(geometry.boundingBox).applyMatrix4(asMesh.matrixWorld);
          bounds.union(worldGeometryBounds);
          hasGeometry = true;
        }
      }
    }

    current.children.forEach((child) => {
      if (excludedSubtree && child === excludedSubtree) {
        return;
      }
      const childRecord = child as unknown as {
        isURDFJoint?: boolean;
        jointType?: unknown;
        type?: string;
      };
      const childType = typeof childRecord.type === "string" ? childRecord.type.toLowerCase() : "";
      const childName = child.name.toLowerCase();
      const isJointNode =
        childRecord.isURDFJoint === true ||
        Object.prototype.hasOwnProperty.call(childRecord, "jointType") ||
        childType.includes("urdfjoint") ||
        childName === "joint" ||
        childName.startsWith("joint");
      if (!isJointNode) {
        pending.push(child);
      }
    });
  }

  if (hasGeometry) {
    return bounds;
  }

  const baseOrigin = baseSource.getWorldPosition(new THREE.Vector3());
  if (!Number.isFinite(baseOrigin.z)) {
    return null;
  }
  return new THREE.Box3(baseOrigin.clone(), baseOrigin.clone());
}

function computeVisibleWorldBounds(root: THREE.Object3D): THREE.Box3 | null {
  const bounds = new THREE.Box3();
  const worldGeometryBounds = new THREE.Box3();
  let hasGeometry = false;

  root.traverseVisible((object) => {
    const typed = object as unknown as {
      isMesh?: boolean;
      isLine?: boolean;
      isLineSegments?: boolean;
      isPoints?: boolean;
      geometry?: THREE.BufferGeometry;
    };
    const hasRenderableGeometry =
      typed.isMesh === true ||
      typed.isLine === true ||
      typed.isLineSegments === true ||
      typed.isPoints === true;
    if (!hasRenderableGeometry || !typed.geometry) {
      return;
    }
    if (!typed.geometry.boundingBox) {
      typed.geometry.computeBoundingBox();
    }
    if (!typed.geometry.boundingBox) {
      return;
    }
    worldGeometryBounds.copy(typed.geometry.boundingBox).applyMatrix4(object.matrixWorld);
    bounds.union(worldGeometryBounds);
    hasGeometry = true;
  });

  return hasGeometry ? bounds : null;
}

function createAxisLabelSprite(
  label: string,
  color: number,
  scale: number,
): THREE.Sprite {
  const canvas = document.createElement("canvas");
  const canvasHeight = 128;
  const fontPx = 64;
  const horizontalPaddingPx = 14;
  canvas.width = canvasHeight;
  canvas.height = canvasHeight;
  const ctx = canvas.getContext("2d");
  if (ctx) {
    const text = label.trim().length > 0 ? label : "?";
    ctx.font = `700 ${fontPx}px sans-serif`;
    const measuredWidth = ctx.measureText(text).width;
    const desiredWidth = Math.ceil(
      Math.max(
        canvasHeight,
        measuredWidth + horizontalPaddingPx * 2,
      ),
    );
    canvas.width = Math.min(512, desiredWidth);
    canvas.height = canvasHeight;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = `#${new THREE.Color(color).getHexString()}`;
    ctx.font = `700 ${fontPx}px sans-serif`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, canvas.width / 2, canvas.height / 2);
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const material = new THREE.SpriteMaterial({
    map: texture,
    transparent: true,
    depthTest: false,
    depthWrite: false,
  });
  const sprite = new THREE.Sprite(material);
  const aspect = canvas.width / canvas.height;
  sprite.scale.set(scale * aspect, scale, scale);
  sprite.renderOrder = 50;
  return sprite;
}

function createAxisArrow(
  direction: THREE.Vector3,
  color: number,
  options: {
    length: number;
    radius: number;
    headLength: number;
    headRadius: number;
  },
): THREE.Group {
  const { length, radius, headLength, headRadius } = options;
  const shaftLength = Math.max(length - headLength, length * 0.65);
  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius, shaftLength, 16),
    new THREE.MeshBasicMaterial({ color }),
  );
  shaft.position.y = shaftLength * 0.5;

  const tip = new THREE.Mesh(
    new THREE.ConeGeometry(headRadius, headLength, 16),
    new THREE.MeshBasicMaterial({ color }),
  );
  tip.position.y = shaftLength + headLength * 0.5;

  const group = new THREE.Group();
  group.add(shaft);
  group.add(tip);
  group.quaternion.setFromUnitVectors(
    new THREE.Vector3(0, 1, 0),
    direction.clone().normalize(),
  );
  return group;
}

function createAxisTripod(options: {
  length: number;
  radius: number;
  headLength: number;
  headRadius: number;
  includeLabels: boolean;
  labelScale?: number;
  labelOffset?: number;
}): THREE.Group {
  const group = new THREE.Group();
  const axisDefs = [
    { axis: "x", dir: new THREE.Vector3(1, 0, 0), color: 0xef4444 },
    { axis: "y", dir: new THREE.Vector3(0, 1, 0), color: 0x22c55e },
    { axis: "z", dir: new THREE.Vector3(0, 0, 1), color: 0x3b82f6 },
  ] as const;

  axisDefs.forEach(({ axis, dir, color }) => {
    const arrow = createAxisArrow(dir, color, options);
    group.add(arrow);
    if (options.includeLabels) {
      const label = createAxisLabelSprite(
        axis.toUpperCase(),
        color,
        options.labelScale ?? 0.12,
      );
      const distance = options.length + (options.labelOffset ?? 0.07);
      label.position.copy(dir.clone().multiplyScalar(distance));
      group.add(label);
    }
  });
function resolveEndEffectorFrameNode(root: THREE.Object3D): THREE.Object3D {
  const findFirst = (predicate: (nameLower: string) => boolean): THREE.Object3D | null => {
    let match: THREE.Object3D | null = null;
    root.traverse((obj) => {
      if (match) {
        return;
      }
      const nameLower = (obj.name ?? "").toLowerCase();
      if (nameLower && predicate(nameLower)) {
        match = obj;
      }
    });
    return match;
  };

  const tcpNode = findFirst((name) => name.startsWith("active-tool-tcp-"));
  if (tcpNode) {
    return tcpNode;
  }
  const toolNode = findFirst(
    (name) => name === "tool0" || name === "tool_0" || name.includes("tcp"),
  );
  if (toolNode) {
    return toolNode;
  }
  const flangeNode = findFirst((name) => name.includes("flange") || name.includes("wrist"));
  if (flangeNode) {
    return flangeNode;
  }
  const joint6Node = findFirst(
    (name) => name === "joint6" || name === "joint_6" || name === "j6",
  );
  if (joint6Node) {
    return joint6Node;
  }
  return root;
}

function findActiveToolTcpNode(root: THREE.Object3D): THREE.Object3D | null {
  let match: THREE.Object3D | null = null;
  root.traverse((obj) => {
    if (match) {
      return;
    }
    const nameLower = (obj.name ?? "").toLowerCase();
    if (nameLower.startsWith("active-tool-tcp-")) {
      match = obj;
    }
  });
  return match;
}


  return group;
}

function isToolTcpNodeForTool(
  node: THREE.Object3D | null | undefined,
  tool: ArmVisualizerProps["activeTool"],
): node is THREE.Object3D {
  if (!node) {
    return false;
  }
  const nameLower = (node.name ?? "").toLowerCase();
  if (!nameLower.startsWith("active-tool-tcp-")) {
    return false;
  }
  if (!tool?.tool_id) {
    return true;
  }
  const toolToken = tool.tool_id.trim().toLowerCase();
  if (!toolToken) {
    return true;
  }
  return nameLower === `active-tool-tcp-${toolToken}` || nameLower === `active-tool-tcp-synth-${toolToken}`;
}

function findToolTcpNodeForTool(
  root: THREE.Object3D,
  tool: ArmVisualizerProps["activeTool"],
): THREE.Object3D | null {
  if (!tool?.tool_id) {
    return null;
  }
  const exact = root.getObjectByName(`active-tool-tcp-${tool.tool_id}`);
  if (exact) {
    return exact;
  }
  const synth = root.getObjectByName(`active-tool-tcp-synth-${tool.tool_id}`);
  if (synth) {
    return synth;
  }
  return null;
}

function applySolverToolOffsetToNode(
  node: THREE.Object3D,
  offset:
    | {
        position_mm: { x: number; y: number; z: number };
        rotation_deg: { x: number; y: number; z: number };
      }
    | undefined
    | null,
) {
  const mm = offset?.position_mm ?? { x: 0, y: 0, z: 0 };
  const deg = offset?.rotation_deg ?? { x: 0, y: 0, z: 0 };
  node.position.set(Number(mm.x) / 1000, Number(mm.y) / 1000, Number(mm.z) / 1000);
  const rx = THREE.MathUtils.degToRad(Number(deg.x));
  const ry = THREE.MathUtils.degToRad(Number(deg.y));
  const rz = THREE.MathUtils.degToRad(Number(deg.z));
  // Match backend runtime._matrix_from_offset (SciPy as lower-case xyz):
  // R = Rz * Ry * Rx (extrinsic XYZ convention).
  const rot = new THREE.Matrix4()
    .makeRotationZ(rz)
    .multiply(new THREE.Matrix4().makeRotationY(ry))
    .multiply(new THREE.Matrix4().makeRotationX(rx));
  node.quaternion.setFromRotationMatrix(rot);
}

function buildSolverToolOffsetMatrix(
  offset:
    | {
        position_mm: { x: number; y: number; z: number };
        rotation_deg: { x: number; y: number; z: number };
      }
    | undefined
    | null,
): THREE.Matrix4 {
  const probe = new THREE.Object3D();
  applySolverToolOffsetToNode(probe, offset);
  probe.updateMatrix();
  return probe.matrix.clone();
}

function findJoint6Anchor(robot: URDFRobot): THREE.Object3D | null {
  const jointsRecord = robot.joints as unknown as Record<string, THREE.Object3D | undefined>;
  return (
    jointsRecord?.joint6 ??
    Object.entries(jointsRecord ?? {}).find(([name]) => {
      const token = name.toLowerCase();
      return token === "joint6" || token === "joint_6" || token === "j6";
    })?.[1] ??
    robot.getObjectByName("joint6") ??
    null
  );
}

function applyJointValuesToRobotHierarchy(root: THREE.Object3D, joints: number[]) {
  const jointSetters = new Map<string, (value: number) => void>();
  root.traverse((obj) => {
    const asJoint = obj as THREE.Object3D & {
      setJointValue?: (value: number) => void;
    };
    if (typeof asJoint.setJointValue !== "function") {
      return;
    }
    const nameToken = (obj.name ?? "").toLowerCase();
    const urdfToken = String((obj as { urdfName?: unknown }).urdfName ?? "").toLowerCase();
    const token = nameToken || urdfToken;
    if (!token) {
      return;
    }
    jointSetters.set(token, asJoint.setJointValue.bind(asJoint));
  });

  joints.forEach((value, index) => {
    if (!Number.isFinite(value)) {
      return;
    }
    const candidates = [`joint${index + 1}`, `joint_${index + 1}`, `j${index + 1}`];
    const setter = candidates
      .map((candidate) => jointSetters.get(candidate))
      .find((candidate): candidate is (value: number) => void => Boolean(candidate));
    if (setter) {
      setter(value);
    }
  });
}

function createEndEffectorFrameMarker(
  label: string,
  opacity: number,
): THREE.Group {
  const group = new THREE.Group();
  group.name = `ee-frame-${label.toLowerCase().replace(/[^a-z0-9]+/g, "-")}`;
  const tripod = createAxisTripod({
    length: 0.05,
    radius: 0.0018,
    headLength: 0.011,
    headRadius: 0.0044,
    includeLabels: true,
    labelScale: 0.02,
    labelOffset: 0.014,
  });
  tripod.traverse((obj) => {
    if (obj instanceof THREE.Mesh) {
      const mat = obj.material as THREE.MeshBasicMaterial;
      mat.transparent = true;
      mat.opacity = opacity;
      mat.depthTest = false;
      mat.depthWrite = false;
      obj.renderOrder = 44;
      return;
    }
    if (obj instanceof THREE.Sprite) {
      const mat = obj.material as THREE.SpriteMaterial;
      mat.transparent = true;
      mat.opacity = opacity;
      mat.depthTest = false;
      mat.depthWrite = false;
      obj.renderOrder = 45;
    }
  });
  group.add(tripod);

  const origin = new THREE.Mesh(
    new THREE.SphereGeometry(0.0038, 14, 14),
    new THREE.MeshBasicMaterial({
      color: 0xe2e8f0,
      transparent: true,
      opacity,
      depthTest: false,
      depthWrite: false,
    }),
  );
  origin.renderOrder = 46;
  group.add(origin);

  const caption = createAxisLabelSprite(label, 0xe2e8f0, 0.024);
  caption.position.set(0, 0, 0.065);
  const captionMaterial = caption.material as THREE.SpriteMaterial;
  captionMaterial.transparent = true;
  captionMaterial.opacity = opacity;
  captionMaterial.depthTest = false;
  captionMaterial.depthWrite = false;
  caption.renderOrder = 46;
  group.add(caption);
  return group;
}

function applyStepTransform(root: THREE.Group, transform?: StepTransform) {
  const next = transform ?? DEFAULT_STEP_TRANSFORM;
  // Apply STEP transform directly in scene/world axes.
  root.position.set(next.position.x, next.position.y, next.position.z);
  root.rotation.set(
    THREE.MathUtils.degToRad(next.rotationDeg.x),
    THREE.MathUtils.degToRad(next.rotationDeg.y),
    THREE.MathUtils.degToRad(next.rotationDeg.z),
  );
  const safeScale = Number.isFinite(next.scale)
    ? Math.max(1e-4, next.scale)
    : 1;
  root.scale.setScalar(safeScale);
}

function createEdgeOverlayMesh(
  points: Point3[],
  color: number,
  radius: number,
): THREE.Mesh | null {
  if (!Array.isArray(points) || points.length < 2) {
    return null;
  }
  const vectors = points.map((point) => new THREE.Vector3(point.x, point.y, point.z));
  const curve = new THREE.CatmullRomCurve3(vectors, false, "centripetal");
  const tubularSegments = Math.max(24, points.length * 4);
  const geometry = new THREE.TubeGeometry(curve, tubularSegments, radius, 10, false);
  const material = new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity: 0.95,
    depthTest: false,
    depthWrite: false,
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.renderOrder = 24;
  return mesh;
}

async function resolveRobotUrdfConfig(selectedRobotId?: string | null): Promise<{
  robotId: string;
  urdfPath: string;
  assetBasePath: string;
}> {
  const candidateSources = [
    {
      indexUrl: "/assets/robots/index.json",
      resolveAssetUrl: (assetPath: string) => assetPath,
    },
    {
      indexUrl: buildDevServedPublicAssetUrl("/assets/robots/index.json"),
      resolveAssetUrl: (assetPath: string) => buildDevServedPublicAssetUrl(assetPath) ?? assetPath,
    },
    {
      indexUrl: "/public/assets/robots/index.json",
      resolveAssetUrl: (assetPath: string) => `/public${assetPath}`,
    },
  ].filter(
    (candidate): candidate is {
      indexUrl: string;
      resolveAssetUrl: (assetPath: string) => string;
    } => typeof candidate.indexUrl === "string" && candidate.indexUrl.length > 0,
  );
  let parsed: Partial<RobotAssetIndex> | null = null;
  let resolvedAssetUrl: ((assetPath: string) => string) | null = null;
  let lastError: Error | null = null;

  for (const candidate of candidateSources) {
    try {
      const response = await fetch(candidate.indexUrl, { cache: "no-store" });
      if (!response.ok) {
        lastError = new Error(
          `Failed to load robot asset index from ${candidate.indexUrl}: HTTP ${response.status}`,
        );
        continue;
      }
      const contentType = response.headers.get("content-type") ?? "";
      if (!contentType.includes("json")) {
        lastError = new Error(
          `Robot asset index at ${candidate.indexUrl} returned '${contentType || "unknown"}' instead of JSON.`,
        );
        continue;
      }
      parsed = (await response.json()) as Partial<RobotAssetIndex>;
      resolvedAssetUrl = candidate.resolveAssetUrl;
      break;
    } catch (error) {
      lastError = error instanceof Error ? error : new Error(String(error));
    }
  }

  if (!parsed) {
    throw lastError ?? new Error("Failed to load robot asset index.");
  }

  if (!parsed || typeof parsed.defaultRobotId !== "string" || !parsed.robots) {
    throw new Error("Invalid robot asset index format");
  }
  const requestedRobotId =
    typeof selectedRobotId === "string" && selectedRobotId.trim().length > 0
      ? selectedRobotId.trim()
      : parsed.defaultRobotId;
  const resolvedRobotId =
    parsed.robots[requestedRobotId] ? requestedRobotId : parsed.defaultRobotId;
  const selected = parsed.robots[resolvedRobotId];
  if (!selected || typeof selected.urdfPath !== "string") {
    throw new Error(`Robot '${resolvedRobotId}' missing urdfPath in index`);
  }

  const urdfPath = resolvedAssetUrl ? resolvedAssetUrl(selected.urdfPath) : selected.urdfPath;
  const slashIndex = urdfPath.lastIndexOf("/");
  if (slashIndex < 0) {
    throw new Error(`Invalid URDF path in robot asset index: ${urdfPath}`);
  }
  const assetBasePath = urdfPath.slice(0, slashIndex + 1);
  return {
    robotId: resolvedRobotId,
    urdfPath,
    assetBasePath,
  };
}

function _resolveToolAssetPath(rawPath: string): string {
  const trimmed = rawPath.trim();
  if (!trimmed) {
    return "";
  }
  if (trimmed.startsWith("/")) {
    return trimmed;
  }
  return `/assets/tools/${trimmed.replace(/^\.\/+/, "")}`;
}

function _createFallbackToolMarker(): THREE.Group {
  const group = new THREE.Group();
  group.name = "active-tool-fallback";

  const body = new THREE.Mesh(
    new THREE.CylinderGeometry(0.008, 0.008, 0.12, 20),
    new THREE.MeshStandardMaterial({
      color: 0xf97316,
      metalness: 0.15,
      roughness: 0.55,
      transparent: true,
      opacity: 0.9,
    }),
  );
  body.rotation.x = Math.PI / 2;
  body.position.z = 0.06;
  body.castShadow = true;
  body.receiveShadow = true;
  group.add(body);

  const tip = new THREE.Mesh(
    new THREE.ConeGeometry(0.011, 0.03, 24),
    new THREE.MeshStandardMaterial({
      color: 0xfacc15,
      metalness: 0.2,
      roughness: 0.45,
    }),
  );
  tip.rotation.x = Math.PI / 2;
  tip.position.z = 0.135;
  tip.castShadow = true;
  tip.receiveShadow = true;
  group.add(tip);
  return group;
}

export type ArmVisualizerHandle = {
  resetView: () => void;
  focusOnPoints: (points: Point3[]) => void;
};

export const ArmVisualizer = forwardRef(function ArmVisualizer(
  {
    robotId,
    activeTool,
    joints,
    showBoundingBox,
    selectionMode,
    onPointSelected,
    weldSelectionMode = false,
    topologyEdges,
    selectedTopologyEdgeId,
    selectedTopologyEdgeIds,
    onTopologyEdgeSelected,
    weldActive = false,
    weldIndicatorPoint,
    weldStartPoint,
    weldStopPoint,
    weldSegmentPoints,
    showEndEffectorFrame = false,
    weldAnglePreview,
    weldGhostJoints,
    pathPoints,
    waypoints,
    highlightPathRange,
    highlightWaypointIndices,
    stepFile,
    stepTransform,
    onStepStatusChange,
  }: ArmVisualizerProps,
  ref: ForwardedRef<ArmVisualizerHandle>,
) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const robotRef = useRef<URDFRobot | null>(null);
  const targetAnglesRef = useRef<number[] | null>(null);
  const currentAnglesRef = useRef<number[] | null>(null);
  const previousTimeRef = useRef<number | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const initialControlsTarget = useRef(new THREE.Vector3(0.25, 0.15, 0));
  const defaultCameraOffset = useRef(new THREE.Vector3(1.15, 0.85, 1.6));
  const defaultCameraOffsetCaptured = useRef(false);
  const isGroundedRef = useRef(false);
  const boundingCenterRef = useRef(new THREE.Vector3());
  const liveToolTcpNodeRef = useRef<THREE.Object3D | null>(null);
  const [toolTcpVersion, setToolTcpVersion] = useState(0);
  const pendingDynamicBoundsRef = useRef(false);
  const lastLiveBoundsRefreshMsRef = useRef(0);
  const lastContainerSizeRef = useRef<{ width: number; height: number } | null>(null);
  const boundingWallsRef = useRef<THREE.Mesh[]>([]);
  const boundingEdgesRef = useRef<THREE.LineSegments | null>(null);
  const boundingMarkersRef = useRef<THREE.Object3D[]>([]);
  const setBoundsVisibilityRef = useRef<((visible: boolean) => void) | null>(null);
  const showBoundingBoxRef = useRef(showBoundingBox);
  const selectionModeRef = useRef(selectionMode);
  const weldSelectionModeRef = useRef(weldSelectionMode);
  const selectedTopologyEdgeIdRef = useRef<string | null>(selectedTopologyEdgeId ?? null);
  const selectedTopologyEdgeIdsRef = useRef<Set<string>>(
    new Set(selectedTopologyEdgeIds ?? []),
  );
  const hoveredTopologyEdgeIdRef = useRef<string | null>(null);
  const onPointSelectedRef = useRef(onPointSelected);
  const onTopologyEdgeSelectedRef = useRef(onTopologyEdgeSelected);
  const previewGroupRef = useRef<THREE.Group | null>(null);
  const activeToolGroupRef = useRef<THREE.Group | null>(null);
  const stepRootRef = useRef<THREE.Group | null>(null);
  const topologyEdgesGroupRef = useRef<THREE.Group | null>(null);
  const topologyEdgeObjectsRef = useRef<THREE.Line[]>([]);
  const topologyEdgePickObjectsRef = useRef<THREE.Mesh[]>([]);
  const topologyEdgePointsByIdRef = useRef<Map<string, Point3[]>>(new Map());
  const topologyEdgeLinesByIdRef = useRef<Map<string, THREE.Line>>(new Map());
  const topologyHoverOverlayRef = useRef<THREE.Mesh | null>(null);
  const topologySelectionOverlayRef = useRef<THREE.Mesh | null>(null);
  const refreshTopologyEdgeVisualsRef = useRef<(() => void) | null>(null);
  const weldEndpointsGroupRef = useRef<THREE.Group | null>(null);
  const weldIndicatorRef = useRef<THREE.Group | null>(null);
  const weldAnglePreviewGroupRef = useRef<THREE.Group | null>(null);
  const weldEeFrameRef = useRef<THREE.Group | null>(null);
  const weldGhostRobotRef = useRef<THREE.Object3D | null>(null);
  const onStepStatusChangeRef = useRef(onStepStatusChange);
  const activeToolRef = useRef<ArmVisualizerProps["activeTool"]>(activeTool);
  const activeToolSignatureRef = useRef<string>(buildActiveToolSignature(activeTool));
  const attachActiveToolVisualRef = useRef<
    ((robot: URDFRobot, tool: ArmVisualizerProps["activeTool"]) => void) | null
  >(null);
  const raycasterRef = useRef(new THREE.Raycaster());
  const pointerRef = useRef(new THREE.Vector2());
  const groundPlaneRef = useRef(new THREE.Plane(new THREE.Vector3(0, 0, 1), 0));

  useEffect(() => {
    const container = containerRef.current;
    if (!container) {
      return;
    }

    const scene = new THREE.Scene();
    scene.background = new THREE.Color("#020617");
    sceneRef.current = scene;
    const raycaster = raycasterRef.current;
    const pointer = pointerRef.current;
    const groundPlane = groundPlaneRef.current;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.autoClear = false;
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    const camera = new THREE.PerspectiveCamera(
      45,
      container.clientWidth / Math.max(1, container.clientHeight),
      0.05,
      50,
    );
    camera.up.set(0, 0, 1);
    camera.position
      .copy(initialControlsTarget.current)
      .add(defaultCameraOffset.current);
    cameraRef.current = camera;

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.copy(initialControlsTarget.current);
    controlsRef.current = controls;

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const keyLight = new THREE.DirectionalLight(0xffffff, 1.2);
    keyLight.position.set(1.5, 2, 1.5);
    scene.add(keyLight);

    const fillLight = new THREE.DirectionalLight(0x89cff0, 0.4);
    fillLight.position.set(-1.5, 1, -1.5);
    scene.add(fillLight);

    const hemiLight = new THREE.HemisphereLight(0x9be7ff, 0x0f172a, 0.3);
    scene.add(hemiLight);

    const grid = new THREE.GridHelper(
      GRID_SIZE,
      GRID_CELLS_PER_SIDE,
      0x38bdf8,
      0x1e293b,
    );
    // GridHelper is XZ by default (Y-up). Rotate into XY so Z becomes up.
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);
    const worldAxes = createAxisTripod({
      length: WORLD_AXIS_LENGTH,
      radius: WORLD_AXIS_RADIUS,
      headLength: WORLD_AXIS_HEAD_LENGTH,
      headRadius: WORLD_AXIS_HEAD_RADIUS,
      includeLabels: false,
    });
    worldAxes.position.set(0, 0, 0);
    scene.add(worldAxes);
    const orientationScene = new THREE.Scene();
    const orientationAxes = createAxisTripod({
      length: 0.4,
      radius: 0.03,
      headLength: 0.09,
      headRadius: 0.06,
      includeLabels: true,
      labelScale: 0.13,
      labelOffset: 0.1,
    });
    orientationScene.add(orientationAxes);
    const orientationCamera = new THREE.OrthographicCamera(-0.8, 0.8, 0.8, -0.8, 0.01, 10);
    orientationCamera.position.set(0, 0, 2);
    orientationCamera.lookAt(0, 0, 0);

    const handlePointerDown = (event: PointerEvent) => {
      if (event.button !== 0) {
        return;
      }
      const camera = cameraRef.current;
      if (!camera) {
        return;
      }
      event.preventDefault();
      const rect = renderer.domElement.getBoundingClientRect();
      pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointer, camera);
      if (weldSelectionModeRef.current) {
        const hit = raycaster.intersectObjects(topologyEdgePickObjectsRef.current, true)[0];
        if (hit && hit.object) {
          const edgeId = (hit.object.userData?.edgeId as string | undefined) ?? "";
          if (edgeId) {
            event.preventDefault();
            hoveredTopologyEdgeIdRef.current = edgeId;
            refreshTopologyEdgeVisualsRef.current?.();
            const callback = onTopologyEdgeSelectedRef.current;
            if (callback) {
              callback(edgeId);
            }
            return;
          }
        }
      }
      if (!selectionModeRef.current || !event.shiftKey) {
        return;
      }
      const intersection = new THREE.Vector3();
      if (raycaster.ray.intersectPlane(groundPlane, intersection)) {
        const callback = onPointSelectedRef.current;
        if (callback) {
          callback({
            x: intersection.x,
            y: intersection.y,
            z: Math.max(intersection.z, 0),
          });
        }
      }
    };

    const handlePointerMove = (event: PointerEvent) => {
      if (!weldSelectionModeRef.current) {
        if (hoveredTopologyEdgeIdRef.current !== null) {
          hoveredTopologyEdgeIdRef.current = null;
          refreshTopologyEdgeVisualsRef.current?.();
        }
        return;
      }
      const camera = cameraRef.current;
      if (!camera) {
        return;
      }
      const rect = renderer.domElement.getBoundingClientRect();
      pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointer, camera);
      const hit = raycaster.intersectObjects(topologyEdgePickObjectsRef.current, true)[0];
      const nextHovered =
        hit && hit.object ? ((hit.object.userData?.edgeId as string | undefined) ?? null) : null;
      if (hoveredTopologyEdgeIdRef.current !== nextHovered) {
        hoveredTopologyEdgeIdRef.current = nextHovered;
        refreshTopologyEdgeVisualsRef.current?.();
      }
    };

    const handlePointerLeave = () => {
      if (hoveredTopologyEdgeIdRef.current !== null) {
        hoveredTopologyEdgeIdRef.current = null;
        refreshTopologyEdgeVisualsRef.current?.();
      }
    };

    const loader = new URDFLoader();
    let disposed = false;
    let attemptedUrdfPath = "(not resolved)";
    const debugMarkers = boundingMarkersRef.current;
    const isFiniteBox = (box: THREE.Box3) =>
      Number.isFinite(box.min.x) &&
      Number.isFinite(box.min.y) &&
      Number.isFinite(box.min.z) &&
      Number.isFinite(box.max.x) &&
      Number.isFinite(box.max.y) &&
      Number.isFinite(box.max.z);

    const updateBoundingMarkers = (box: THREE.Box3) => {
      const corners = [
        new THREE.Vector3(box.min.x, box.min.y, box.min.z),
        new THREE.Vector3(box.min.x, box.min.y, box.max.z),
        new THREE.Vector3(box.min.x, box.max.y, box.min.z),
        new THREE.Vector3(box.min.x, box.max.y, box.max.z),
        new THREE.Vector3(box.max.x, box.min.y, box.min.z),
        new THREE.Vector3(box.max.x, box.min.y, box.max.z),
        new THREE.Vector3(box.max.x, box.max.y, box.min.z),
        new THREE.Vector3(box.max.x, box.max.y, box.max.z),
      ];

      if (debugMarkers.length === 0) {
        corners.forEach((corner, index) => {
          const marker = new THREE.Mesh(
            new THREE.SphereGeometry(0.005, 12, 12),
            new THREE.MeshBasicMaterial({
              color: BOUNDING_MARKER_COLORS[index % BOUNDING_MARKER_COLORS.length],
            }),
          );
          marker.position.copy(corner);
          scene.add(marker);
          debugMarkers.push(marker);
        });
      } else {
        debugMarkers.forEach((marker, index) => {
          marker.position.copy(corners[index]);
        });
      }
    };

    const wallConfigs = [
      {
        // Left wall (x = min)
        axis: "x" as const,
        sign: -1,
        rotation: new THREE.Euler(0, Math.PI / 2, 0),
        size: (size: THREE.Vector3) => new THREE.Vector2(size.z, size.y),
        position: (box: THREE.Box3, center: THREE.Vector3) =>
          new THREE.Vector3(box.min.x, center.y, center.z),
      },
      {
        // Right wall (x = max)
        axis: "x" as const,
        sign: 1,
        rotation: new THREE.Euler(0, -Math.PI / 2, 0),
        size: (size: THREE.Vector3) => new THREE.Vector2(size.z, size.y),
        position: (box: THREE.Box3, center: THREE.Vector3) =>
          new THREE.Vector3(box.max.x, center.y, center.z),
      },
      {
        // Front wall (z = max)
        axis: "z" as const,
        sign: 1,
        rotation: new THREE.Euler(0, 0, 0),
        size: (size: THREE.Vector3) => new THREE.Vector2(size.x, size.y),
        position: (box: THREE.Box3, center: THREE.Vector3) =>
          new THREE.Vector3(center.x, center.y, box.max.z),
      },
      {
        // Back wall (z = min)
        axis: "z" as const,
        sign: -1,
        rotation: new THREE.Euler(Math.PI, 0, 0),
        size: (size: THREE.Vector3) => new THREE.Vector2(size.x, size.y),
        position: (box: THREE.Box3, center: THREE.Vector3) =>
          new THREE.Vector3(center.x, center.y, box.min.z),
      },
      {
        // Ceiling (y = max)
        axis: "y" as const,
        sign: 1,
        rotation: new THREE.Euler(-Math.PI / 2, 0, 0),
        size: (size: THREE.Vector3) => new THREE.Vector2(size.x, size.z),
        position: (box: THREE.Box3, center: THREE.Vector3) =>
          new THREE.Vector3(center.x, box.max.y, center.z),
      },
      {
        // Floor hint (y = min)
        axis: "y" as const,
        sign: -1,
        rotation: new THREE.Euler(Math.PI / 2, 0, 0),
        size: (size: THREE.Vector3) => new THREE.Vector2(size.x, size.z),
        position: (box: THREE.Box3, center: THREE.Vector3) =>
          new THREE.Vector3(center.x, box.min.y, center.z),
      },
    ];

    const ensureBoundingWalls = (box: THREE.Box3) => {
      const size = box.getSize(new THREE.Vector3());
      const center = box.getCenter(new THREE.Vector3());
      const walls = boundingWallsRef.current;

      wallConfigs.forEach((config, index) => {
        let wall = walls[index];
        if (!wall) {
          const geometry = new THREE.PlaneGeometry(1, 1);
          const material = new THREE.MeshBasicMaterial({
            color: 0x38bdf8,
            transparent: true,
            opacity: 0.05,
            depthWrite: false,
            side: THREE.DoubleSide,
          });
          wall = new THREE.Mesh(geometry, material);
          wall.renderOrder = 5;
          walls[index] = wall;
          scene.add(wall);
        }

        const extent = config.size(size);
        wall.scale.set(Math.max(extent.x, 1e-6), Math.max(extent.y, 1e-6), 1);
        wall.position.copy(config.position(box, center));
        wall.rotation.copy(config.rotation);
      });
    };

    const ensureBoundingEdges = (box: THREE.Box3) => {
      const corners = [
        new THREE.Vector3(box.min.x, box.min.y, box.min.z),
        new THREE.Vector3(box.max.x, box.min.y, box.min.z),
        new THREE.Vector3(box.max.x, box.max.y, box.min.z),
        new THREE.Vector3(box.min.x, box.max.y, box.min.z),
        new THREE.Vector3(box.min.x, box.min.y, box.max.z),
        new THREE.Vector3(box.max.x, box.min.y, box.max.z),
        new THREE.Vector3(box.max.x, box.max.y, box.max.z),
        new THREE.Vector3(box.min.x, box.max.y, box.max.z),
      ];

      const indices = [
        0, 1, 1, 2, 2, 3, 3, 0, // bottom
        4, 5, 5, 6, 6, 7, 7, 4, // top
        0, 4, 1, 5, 2, 6, 3, 7, // verticals
      ];

      let edges = boundingEdgesRef.current;
      if (!edges) {
        const geometry = new THREE.BufferGeometry();
        const material = new THREE.LineBasicMaterial({
          color: 0x38bdf8,
          transparent: true,
          opacity: 0.25,
        });
        edges = new THREE.LineSegments(geometry, material);
        edges.renderOrder = 6;
        boundingEdgesRef.current = edges;
        scene.add(edges);
      }

      const positions = new Float32Array(indices.length * 3);
      indices.forEach((idx, arrayIndex) => {
        const vertex = corners[idx];
        positions[arrayIndex * 3] = vertex.x;
        positions[arrayIndex * 3 + 1] = vertex.y;
        positions[arrayIndex * 3 + 2] = vertex.z;
      });
      edges.geometry.setAttribute(
        "position",
        new THREE.BufferAttribute(positions, 3),
      );
      edges.geometry.computeBoundingSphere();
    };

    const updateBoundsVisibility = (visible: boolean) => {
      boundingMarkersRef.current.forEach((marker) => {
        marker.visible = visible;
      });
      boundingWallsRef.current.forEach((wall) => {
        wall.visible = visible;
      });
      if (boundingEdgesRef.current) {
        boundingEdgesRef.current.visible = visible;
      }
    };
    setBoundsVisibilityRef.current = updateBoundsVisibility;

    const alignToGroundAndUpdateBounds = (options?: {
      snapCamera?: boolean;
      applySnapshot?: boolean;
    }) => {
      const robot = robotRef.current;
      if (!robot) {
        return;
      }
      robot.updateMatrixWorld(true);

      const candidateNames = ["base", "base_link", "link0", "base_link_inertia"];
      let baseObject: THREE.Object3D | null = null;
      const linksRecord = robot.links as unknown as Record<
        string,
        THREE.Object3D | undefined
      >;
      for (const name of candidateNames) {
        const fromLinks = linksRecord?.[name];
        if (fromLinks) {
          baseObject = fromLinks;
          break;
        }
        const found = robot.getObjectByName(name);
        if (found) {
          baseObject = found;
          break;
        }
      }

      const baseSource =
        baseObject ?? linksRecord?.base ?? linksRecord?.base_link ?? robot;
      const activeToolGroup = activeToolGroupRef.current;
      // Grounding must be based on base-link geometry only and never track tool meshes.
      const baseBox = computeGroundingBoxFromBase(baseSource, activeToolGroup);
      if (!baseBox) {
        return;
      }
      if (!isFiniteBox(baseBox)) {
        return;
      }

      const shouldApplyGrounding = !isGroundedRef.current;
      if (shouldApplyGrounding) {
        const deltaZ = baseBox.min.z;
        if (Number.isFinite(deltaZ) && Math.abs(deltaZ) > 1e-5) {
          robot.position.z -= deltaZ;
          robot.updateMatrixWorld(true);
        }
      }

      const groundedBBox = computeVisibleWorldBounds(robot);
      if (!groundedBBox || !isFiniteBox(groundedBBox)) {
        return;
      }

      const center = groundedBBox.getCenter(boundingCenterRef.current);
      initialControlsTarget.current.copy(center);

      updateBoundingMarkers(groundedBBox);
      ensureBoundingWalls(groundedBBox);
      ensureBoundingEdges(groundedBBox);
      updateBoundsVisibility(showBoundingBoxRef.current);

      if (options?.applySnapshot) {
        const values = targetAnglesRef.current;
        if (values) {
          currentAnglesRef.current = values.slice();
          values.forEach((value, index) => {
            const joint = robot.joints[`joint${index + 1}`];
            if (joint) {
              joint.setJointValue(value);
            }
          });
        }
      }

      const controlsInstance = controlsRef.current;
      const cameraInstance = cameraRef.current;
      if ((options?.snapCamera || shouldApplyGrounding) && controlsInstance && cameraInstance) {
        const offset = cameraInstance.position
          .clone()
          .sub(controlsInstance.target);
        controlsInstance.target.copy(initialControlsTarget.current);
        cameraInstance.position
          .copy(initialControlsTarget.current)
          .add(offset);
        cameraInstance.updateProjectionMatrix();
        controlsInstance.update();
        if (!defaultCameraOffsetCaptured.current) {
          defaultCameraOffset.current.copy(offset);
          defaultCameraOffsetCaptured.current = true;
        }
      }
    };

    const attachActiveToolVisual = (
      robot: URDFRobot,
      tool: ArmVisualizerProps["activeTool"],
    ) => {
      const previous = activeToolGroupRef.current;
      if (previous) {
        previous.parent?.remove(previous);
        disposeObject3D(previous);
        activeToolGroupRef.current = null;
      }
      liveToolTcpNodeRef.current = null;
      if (!tool) {
        setToolTcpVersion((value) => value + 1);
        return;
      }
      const joint6Anchor = findJoint6Anchor(robot);
      if (!joint6Anchor) {
        console.error("[ArmVisualizer] joint6 anchor not found; skipping active tool attach");
        setToolTcpVersion((value) => value + 1);
        return;
      }
      // Mesh origin must be at J6 flange/joint frame (never TCP/tool tip).
      const anchor = joint6Anchor;
      const toolRootGroup = new THREE.Group();
      toolRootGroup.name = `active-tool-${tool.tool_id}`;
      
      // Solver defines Z as the tool approach axis (DH convention).
      // URDF might define joint axis differently (e.g. X for gradient-05).
      // Rotate the solver's Z axis to align with the actual physical URDF joint axis.
      const toolTcpMappingGroup = new THREE.Group();
      toolTcpMappingGroup.name = `active-tool-tcp-map-${tool.tool_id}`;
      const jointAxis = (anchor as any).axis instanceof THREE.Vector3 
        ? (anchor as any).axis 
        : new THREE.Vector3(0, 0, 1);
      
      if (Math.abs(jointAxis.x - 1.0) < 1e-5) {
        // Map Solver DH frame to URDF frame where Joint Axis is X:
        // Solver Z -> URDF X
        // Solver X -> URDF -Z
        // Solver Y -> URDF Y
        const mappingMatrix = new THREE.Matrix4().set(
          0,  0, 1, 0,
          0,  1, 0, 0,
         -1,  0, 0, 0,
          0,  0, 0, 1
        );
        toolTcpMappingGroup.quaternion.setFromRotationMatrix(mappingMatrix);
      } else if (Math.abs(jointAxis.y - 1.0) < 1e-5) {
        // Map Solver DH frame to URDF frame where Joint Axis is Y:
        // Solver Z -> URDF Y
        // Solver X -> URDF X
        // Solver Y -> URDF Z
        const mappingMatrix = new THREE.Matrix4().set(
          1, 0,  0, 0,
          0, 0,  1, 0,
          0, 1,  0, 0,
          0, 0,  0, 1
        );
        toolTcpMappingGroup.quaternion.setFromRotationMatrix(mappingMatrix);
      } else if (Math.abs(jointAxis.z - 1.0) > 1e-5) {
        // Fallback for arbitrary axes
        toolTcpMappingGroup.quaternion.setFromUnitVectors(
          new THREE.Vector3(0, 0, 1),
          jointAxis.clone().normalize()
        );
      }
      toolRootGroup.add(toolTcpMappingGroup);

      // TCP/tool-tip frame (used by kinematic tool offset semantics).
      const toolTcpGroup = new THREE.Group();
      toolTcpGroup.name = `active-tool-tcp-${tool.tool_id}`;
      applySolverToolOffsetToNode(toolTcpGroup, tool.offset);

      // Tool offset is TCP-only. Mesh stays in J6 frame via toolRootGroup.
      const fallbackMarker = _createFallbackToolMarker();
      const hasMeshAsset =
        typeof tool.mesh?.asset_path === "string" &&
        tool.mesh.asset_path.trim().length > 0;
      // Only show fallback when no mesh is configured or mesh loading fails.
      fallbackMarker.visible = !hasMeshAsset;
      toolTcpGroup.add(fallbackMarker);
      toolTcpMappingGroup.add(toolTcpGroup);
      if (tool.mesh?.asset_path) {
        const meshPath = _resolveToolAssetPath(tool.mesh.asset_path);
        if (meshPath.toLowerCase().endsWith(".stl")) {
          const stlLoader = new STLLoader();
          stlLoader.load(
            meshPath,
            (geometry) => {
              if (disposed) {
                geometry.dispose();
                return;
              }
              if (activeToolGroupRef.current !== toolRootGroup) {
                geometry.dispose();
                return;
              }
              const mesh = new THREE.Mesh(
                geometry,
                new THREE.MeshStandardMaterial({
                  color: 0x38bdf8,
                  metalness: 0.15,
                  roughness: 0.6,
                }),
              );
              const scale = Number(tool.mesh?.scale ?? 1);
              mesh.scale.setScalar(Number.isFinite(scale) ? Math.max(1e-5, scale) : 1);
              const meshPosMm = tool.mesh?.position_mm ?? { x: 0, y: 0, z: 0 };
              const meshRotDeg = tool.mesh?.rotation_deg ?? { x: 0, y: 0, z: 0 };
              mesh.position.set(
                Number(meshPosMm.x) / 1000,
                Number(meshPosMm.y) / 1000,
                Number(meshPosMm.z) / 1000,
              );
              mesh.rotation.set(
                (Number(meshRotDeg.x) * Math.PI) / 180,
                (Number(meshRotDeg.y) * Math.PI) / 180,
                (Number(meshRotDeg.z) * Math.PI) / 180,
              );
              mesh.castShadow = true;
              mesh.receiveShadow = true;
              mesh.name = `tool-mesh-${tool.tool_id}`;
              // Mesh transform is relative to the J6/flange anchor frame, not TCP.
              toolRootGroup.add(mesh);
              fallbackMarker.visible = false;
            },
            undefined,
            (error) => {
              fallbackMarker.visible = true;
              console.warn("[ArmVisualizer] Failed to load active tool STL", {
                toolId: tool.tool_id,
                meshPath,
                error,
              });
            },
          );
        } else {
          fallbackMarker.visible = true;
        }
      }
      anchor.add(toolRootGroup);
      activeToolGroupRef.current = toolRootGroup;
      liveToolTcpNodeRef.current = toolTcpGroup;
      setToolTcpVersion((value) => value + 1);
    };
    attachActiveToolVisualRef.current = attachActiveToolVisual;

    const mountRobotInstance = (robot: URDFRobot, metadata: {
      robotId: string;
      urdfPath: string;
    }) => {
      if (disposed) {
        return;
      }
      console.info("[ArmVisualizer] URDF loaded", {
        robotId: metadata.robotId,
        urdfPath: metadata.urdfPath,
        jointNames: Object.keys(robot.joints),
        linkNames: Object.keys(robot.links),
      });

      robot.scale.setScalar(ROBOT_SCALE);
      const defaultMaterial = new THREE.MeshStandardMaterial({
        color: 0x1e293b,
        metalness: 0.1,
        roughness: 0.8,
      });
      const accentMaterial = new THREE.MeshStandardMaterial({
        color: 0x38bdf8,
        metalness: 0.2,
        roughness: 0.5,
      });

      robot.traverse((obj) => {
        if ((obj as THREE.Mesh).isMesh) {
          const mesh = obj as THREE.Mesh;
          mesh.castShadow = true;
          mesh.receiveShadow = true;
          const material =
            mesh.name.toLowerCase().includes("wrist") ||
            mesh.name.toLowerCase().includes("tool")
              ? accentMaterial
              : defaultMaterial;

          if (Array.isArray(mesh.material)) {
            mesh.material.forEach((mat) => {
              (mat as THREE.Material).dispose();
            });
          } else if (mesh.material) {
            (mesh.material as THREE.Material).dispose();
          }
          mesh.material = material;
        }
      });

      scene.add(robot);
      attachActiveToolVisual(robot, activeToolRef.current);

      robotRef.current = robot;
      if (!currentAnglesRef.current) {
        currentAnglesRef.current = new Array(6).fill(0);
      }
      if (!targetAnglesRef.current) {
        targetAnglesRef.current = new Array(6).fill(0);
      }

      const initialiseScene = () => {
        alignToGroundAndUpdateBounds({ snapCamera: true, applySnapshot: true });
        isGroundedRef.current = true;
      };

      initialiseScene();
      updateBoundsVisibility(showBoundingBoxRef.current);
      renderer.render(scene, camera);
    };

    const loadRobotModel = async () => {
      try {
        const urdfConfig = await resolveRobotUrdfConfig(robotId);
        if (disposed) {
          return;
        }
        attemptedUrdfPath = urdfConfig.urdfPath;
        loader.workingPath = urdfConfig.assetBasePath;

        loader.load(
          urdfConfig.urdfPath,
          (robot) => {
            if (disposed) {
              return;
            }
            mountRobotInstance(robot, {
              robotId: urdfConfig.robotId,
              urdfPath: urdfConfig.urdfPath,
            });
          },
          undefined,
          (error) => {
            console.error("[ArmVisualizer] Failed to load URDF", {
              error,
              attemptedPath: attemptedUrdfPath,
            });
          },
        );
      } catch (error) {
        console.error("[ArmVisualizer] Failed to resolve robot URDF config", { error });
      }
    };
    void loadRobotModel();

    const applyResize = (snapCamera = false) => {
      if (!container) {
        return;
      }
      const { clientWidth, clientHeight } = container;
      if (clientWidth <= 0 || clientHeight <= 0) {
        return;
      }
      renderer.setSize(clientWidth, clientHeight);
      camera.aspect = clientWidth / Math.max(clientHeight, 1);
      camera.updateProjectionMatrix();
      const previousSize = lastContainerSizeRef.current;
      const firstMeasuredSize = previousSize === null;
      lastContainerSizeRef.current = {
        width: clientWidth,
        height: clientHeight,
      };
      if (robotRef.current && (snapCamera || firstMeasuredSize)) {
        alignToGroundAndUpdateBounds({ snapCamera: true, applySnapshot: false });
      }
    };
    const handleResize = () => {
      applyResize(false);
    };
    const resizeObserver = new ResizeObserver(() => {
      handleResize();
    });
    resizeObserver.observe(container);
    const initialResizeFrame = window.requestAnimationFrame(() => {
      applyResize(true);
    });

    let animationFrameId: number;
    const animate = (time?: number) => {
      animationFrameId = requestAnimationFrame(animate);

      const deltaSeconds =
        previousTimeRef.current !== null && time !== undefined
          ? Math.min((time - previousTimeRef.current) / 1000, 0.05)
          : 0.016;
      previousTimeRef.current = time ?? null;

      if (pendingDynamicBoundsRef.current) {
        pendingDynamicBoundsRef.current = false;
        alignToGroundAndUpdateBounds({ snapCamera: false, applySnapshot: false });
      }

      controls.update();
      const canvasWidth = container.clientWidth;
      const canvasHeight = container.clientHeight;
      renderer.setViewport(0, 0, canvasWidth, canvasHeight);
      renderer.setScissorTest(false);
      renderer.clear();
      renderer.render(scene, camera);

      orientationAxes.quaternion.copy(camera.quaternion).invert();
      const maxWidgetSize = Math.round(
        Math.min(canvasWidth, canvasHeight) * 0.24,
      );
      const widgetSize = Math.max(
        ORIENTATION_WIDGET_MIN_SIZE_PX,
        Math.min(ORIENTATION_WIDGET_SIZE_PX, maxWidgetSize),
      );
      const widgetX = ORIENTATION_WIDGET_MARGIN_PX;
      const widgetY = ORIENTATION_WIDGET_MARGIN_PX;
      renderer.clearDepth();
      renderer.setScissor(widgetX, widgetY, widgetSize, widgetSize);
      renderer.setViewport(widgetX, widgetY, widgetSize, widgetSize);
      renderer.setScissorTest(true);
      renderer.render(orientationScene, orientationCamera);
      renderer.setScissorTest(false);
      renderer.setViewport(0, 0, canvasWidth, canvasHeight);
    };
    animate();

    renderer.domElement.addEventListener("pointerdown", handlePointerDown);
    renderer.domElement.addEventListener("pointermove", handlePointerMove);
    renderer.domElement.addEventListener("pointerleave", handlePointerLeave);
    window.addEventListener("resize", handleResize);

    return () => {
      disposed = true;
      renderer.domElement.removeEventListener("pointerdown", handlePointerDown);
      renderer.domElement.removeEventListener("pointermove", handlePointerMove);
      renderer.domElement.removeEventListener("pointerleave", handlePointerLeave);
      attachActiveToolVisualRef.current = null;
      window.removeEventListener("resize", handleResize);
      resizeObserver.disconnect();
      window.cancelAnimationFrame(initialResizeFrame);
      cancelAnimationFrame(animationFrameId);
      controls.dispose();
      controlsRef.current = null;
      renderer.dispose();
      grid.geometry.dispose();
      if (Array.isArray(grid.material)) {
        grid.material.forEach((material) => material.dispose());
      } else {
        grid.material.dispose();
      }
      scene.remove(worldAxes);
      disposeObject3D(worldAxes);
      disposeObject3D(orientationAxes);
      orientationScene.clear();
      if (container.contains(renderer.domElement)) {
        container.removeChild(renderer.domElement);
      }
      const previewGroup = previewGroupRef.current;
      if (previewGroup) {
        disposeObject3D(previewGroup);
        scene.remove(previewGroup);
        previewGroupRef.current = null;
      }
      const stepRoot = stepRootRef.current;
      if (stepRoot) {
        disposeObject3D(stepRoot);
        scene.remove(stepRoot);
        stepRootRef.current = null;
      }
      const activeToolGroup = activeToolGroupRef.current;
      if (activeToolGroup) {
        activeToolGroup.parent?.remove(activeToolGroup);
        disposeObject3D(activeToolGroup);
        activeToolGroupRef.current = null;
      }
      const topologyGroup = topologyEdgesGroupRef.current;
      if (topologyGroup) {
        disposeObject3D(topologyGroup);
        scene.remove(topologyGroup);
        topologyEdgesGroupRef.current = null;
      }
      topologyEdgeObjectsRef.current = [];
      topologyEdgePickObjectsRef.current = [];
      topologyEdgePointsByIdRef.current.clear();
      topologyEdgeLinesByIdRef.current.clear();
      refreshTopologyEdgeVisualsRef.current = null;
      hoveredTopologyEdgeIdRef.current = null;
      const weldIndicator = weldIndicatorRef.current;
      if (weldIndicator) {
        disposeObject3D(weldIndicator);
        scene.remove(weldIndicator);
        weldIndicatorRef.current = null;
      }
      const weldEndpointsGroup = weldEndpointsGroupRef.current;
      if (weldEndpointsGroup) {
        disposeObject3D(weldEndpointsGroup);
        scene.remove(weldEndpointsGroup);
        weldEndpointsGroupRef.current = null;
      }
      const weldAnglePreviewGroup = weldAnglePreviewGroupRef.current;
      if (weldAnglePreviewGroup) {
        disposeObject3D(weldAnglePreviewGroup);
        scene.remove(weldAnglePreviewGroup);
        weldAnglePreviewGroupRef.current = null;
      }
      const weldEeFrame = weldEeFrameRef.current;
      if (weldEeFrame) {
        weldEeFrame.parent?.remove(weldEeFrame);
        disposeObject3D(weldEeFrame);
        weldEeFrameRef.current = null;
      }
      const weldGhostRobot = weldGhostRobotRef.current;
      if (weldGhostRobot) {
        disposeObject3D(weldGhostRobot);
        scene.remove(weldGhostRobot);
        weldGhostRobotRef.current = null;
      }
      boundingMarkersRef.current.forEach((marker) => {
        scene.remove(marker);
        marker.traverse((child) => {
          if ((child as THREE.Mesh).isMesh) {
            const meshChild = child as THREE.Mesh;
            meshChild.geometry.dispose();
            if (Array.isArray(meshChild.material)) {
              meshChild.material.forEach((mat) => mat.dispose());
            } else if (meshChild.material) {
              (meshChild.material as THREE.Material).dispose();
            }
          }
        });
      });
      boundingMarkersRef.current = [];
      scene.clear();
      sceneRef.current = null;
      robotRef.current = null;
      targetAnglesRef.current = null;
      currentAnglesRef.current = null;
      previousTimeRef.current = null;
      cameraRef.current = null;
      isGroundedRef.current = false;
      pendingDynamicBoundsRef.current = false;
      lastContainerSizeRef.current = null;
      boundingWallsRef.current.forEach((wall) => {
        scene.remove(wall);
        if ((wall.material as THREE.Material | THREE.Material[])) {
          if (Array.isArray(wall.material)) {
            wall.material.forEach((mat) => mat.dispose());
          } else {
            (wall.material as THREE.Material).dispose();
          }
        }
        wall.geometry.dispose();
      });
      boundingWallsRef.current = [];
      const edges = boundingEdgesRef.current;
      if (edges) {
        scene.remove(edges);
        edges.geometry.dispose();
        (edges.material as THREE.Material).dispose();
        boundingEdgesRef.current = null;
      }
      setBoundsVisibilityRef.current = null;
    };
  }, [robotId]);

  useEffect(() => {
    activeToolRef.current = activeTool;
    const robot = robotRef.current;
    const attachTool = attachActiveToolVisualRef.current;
    if (!robot || !attachTool) {
      return;
    }
    const nextSignature = buildActiveToolSignature(activeTool);
    if (activeToolSignatureRef.current === nextSignature) {
      return;
    }
    activeToolSignatureRef.current = nextSignature;
    attachTool(robot, activeTool);
    pendingDynamicBoundsRef.current = true;
  }, [activeTool]);

  useImperativeHandle(
    ref,
    () => ({
      resetView: () => {
        const camera = cameraRef.current;
        const controls = controlsRef.current;
        if (!camera || !controls) {
          return;
        }
        controls.target.copy(initialControlsTarget.current);
        camera
          .position.copy(initialControlsTarget.current)
          .add(defaultCameraOffset.current);
        camera.updateProjectionMatrix();
        controls.update();
      },
      focusOnPoints: (points: Point3[]) => {
        const camera = cameraRef.current;
        const controls = controlsRef.current;
        if (!camera || !controls || !Array.isArray(points) || points.length === 0) {
          return;
        }
        const finitePoints = points.filter(
          (point) =>
            Number.isFinite(point.x) &&
            Number.isFinite(point.y) &&
            Number.isFinite(point.z),
        );
        if (finitePoints.length === 0) {
          return;
        }
        const bounds = new THREE.Box3();
        finitePoints.forEach((point) => {
          bounds.expandByPoint(new THREE.Vector3(point.x, point.y, point.z));
        });
        if (
          !Number.isFinite(bounds.min.x) ||
          !Number.isFinite(bounds.min.y) ||
          !Number.isFinite(bounds.min.z) ||
          !Number.isFinite(bounds.max.x) ||
          !Number.isFinite(bounds.max.y) ||
          !Number.isFinite(bounds.max.z)
        ) {
          return;
        }
        const center = bounds.getCenter(new THREE.Vector3());
        const size = bounds.getSize(new THREE.Vector3());
        const radius = Math.max(size.length() * 0.5, 0.05);
        const direction = camera.position.clone().sub(controls.target);
        if (direction.lengthSq() < 1e-8) {
          direction.copy(defaultCameraOffset.current);
        }
        direction.normalize();
        const distance = Math.max(radius * 2.8, 0.4);
        controls.target.copy(center);
        camera.position.copy(center).add(direction.multiplyScalar(distance));
        camera.near = 0.01;
        camera.far = Math.max(10, distance * 20);
        camera.updateProjectionMatrix();
        controls.update();
      },
    }),
    [],
  );

  useEffect(() => {
    selectionModeRef.current = selectionMode;
  }, [selectionMode]);

  useEffect(() => {
    weldSelectionModeRef.current = weldSelectionMode;
    if (!weldSelectionMode) {
      hoveredTopologyEdgeIdRef.current = null;
    }
    refreshTopologyEdgeVisualsRef.current?.();
  }, [weldSelectionMode]);

  useEffect(() => {
    selectedTopologyEdgeIdRef.current = selectedTopologyEdgeId ?? null;
    refreshTopologyEdgeVisualsRef.current?.();
  }, [selectedTopologyEdgeId]);

  useEffect(() => {
    selectedTopologyEdgeIdsRef.current = new Set(selectedTopologyEdgeIds ?? []);
    refreshTopologyEdgeVisualsRef.current?.();
  }, [selectedTopologyEdgeIds]);

  useEffect(() => {
    onPointSelectedRef.current = onPointSelected;
  }, [onPointSelected]);

  useEffect(() => {
    onTopologyEdgeSelectedRef.current = onTopologyEdgeSelected;
  }, [onTopologyEdgeSelected]);

  useEffect(() => {
    onStepStatusChangeRef.current = onStepStatusChange;
  }, [onStepStatusChange]);

  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) {
      return;
    }
    const notify = (status: StepLoadStatus) => {
      onStepStatusChangeRef.current?.(status);
    };
    const clearStepRoot = () => {
      const existing = stepRootRef.current;
      if (existing) {
        scene.remove(existing);
        disposeObject3D(existing);
        stepRootRef.current = null;
      }
    };
    clearStepRoot();
    if (!stepFile) {
      notify({ state: "idle", message: "No STEP model loaded." });
      return;
    }

    let disposed = false;
    notify({ state: "loading", message: `Loading ${stepFile.name}...` });

    const run = async () => {
      try {
        const importer = await loadOcctImporter();
        if (disposed) {
          return;
        }
        const readStep = importer.ReadStepFile;
        if (typeof readStep !== "function") {
          throw new Error("STEP importer does not expose ReadStepFile.");
        }
        const bytes = new Uint8Array(await stepFile.arrayBuffer());
        const result = readStep(bytes, {
          linearUnit: "meter",
        });
        if (!result?.success) {
          throw new Error("Failed to parse STEP model.");
        }
        const meshes = Array.isArray(result.meshes) ? result.meshes : [];
        const modelGroup = new THREE.Group();
        modelGroup.name = "step-model";
        meshes.forEach((meshData) => {
          const mesh = buildStepMesh(meshData);
          if (mesh) {
            modelGroup.add(mesh);
          }
        });
        if (modelGroup.children.length === 0) {
          throw new Error("The STEP file does not contain renderable meshes.");
        }

        const modelBounds = new THREE.Box3().setFromObject(modelGroup);
        const center = modelBounds.getCenter(new THREE.Vector3());
        modelGroup.position.set(-center.x, -center.y, -modelBounds.min.z);

        const stepRoot = new THREE.Group();
        stepRoot.name = "step-root";
        const stepLocalAxes = createAxisTripod({
          length: STEP_LOCAL_AXIS_LENGTH,
          radius: STEP_LOCAL_AXIS_RADIUS,
          headLength: STEP_LOCAL_AXIS_HEAD_LENGTH,
          headRadius: STEP_LOCAL_AXIS_HEAD_RADIUS,
          includeLabels: false,
        });
        stepLocalAxes.name = "step-local-axes";
        stepRoot.add(modelGroup);
        stepRoot.add(stepLocalAxes);
        applyStepTransform(stepRoot, stepTransform);

        if (disposed) {
          disposeObject3D(stepRoot);
          return;
        }
        scene.add(stepRoot);
        stepRootRef.current = stepRoot;
        notify({
          state: "loaded",
          message: `Loaded ${stepFile.name} (${modelGroup.children.length} mesh${modelGroup.children.length === 1 ? "" : "es"}).`,
        });
      } catch (error) {
        if (disposed) {
          return;
        }
        notify({
          state: "error",
          message: `STEP load failed: ${(error as Error).message}`,
        });
      }
    };

    run();
    return () => {
      disposed = true;
      clearStepRoot();
    };
  }, [stepFile]);

  useEffect(() => {
    const stepRoot = stepRootRef.current;
    if (!stepRoot) {
      return;
    }
    applyStepTransform(stepRoot, stepTransform);
  }, [stepTransform]);

  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) {
      return;
    }

    const existingGroup = topologyEdgesGroupRef.current;
    if (existingGroup) {
      scene.remove(existingGroup);
      disposeObject3D(existingGroup);
      topologyEdgesGroupRef.current = null;
      topologyEdgeObjectsRef.current = [];
      topologyEdgePickObjectsRef.current = [];
      topologyEdgePointsByIdRef.current.clear();
      topologyEdgeLinesByIdRef.current.clear();
      if (topologyHoverOverlayRef.current) {
        topologyHoverOverlayRef.current = null;
      }
      if (topologySelectionOverlayRef.current) {
        topologySelectionOverlayRef.current = null;
      }
    }

    if (!Array.isArray(topologyEdges) || topologyEdges.length === 0) {
      return;
    }

    const allPoints = topologyEdges.flatMap((edge) => edge.points ?? []);
    if (allPoints.length === 0) {
      return;
    }

    let minX = Number.POSITIVE_INFINITY;
    let minY = Number.POSITIVE_INFINITY;
    let minZ = Number.POSITIVE_INFINITY;
    let maxX = Number.NEGATIVE_INFINITY;
    let maxY = Number.NEGATIVE_INFINITY;
    let maxZ = Number.NEGATIVE_INFINITY;
    allPoints.forEach((p) => {
      minX = Math.min(minX, p.x);
      minY = Math.min(minY, p.y);
      minZ = Math.min(minZ, p.z);
      maxX = Math.max(maxX, p.x);
      maxY = Math.max(maxY, p.y);
      maxZ = Math.max(maxZ, p.z);
    });
    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;

    const root = new THREE.Group();
    root.name = "topology-edge-root";
    const topologyModelGroup = new THREE.Group();
    topologyModelGroup.name = "topology-edge-model";
    topologyModelGroup.position.set(-centerX, -centerY, -minZ);
    root.add(topologyModelGroup);
    applyStepTransform(root, stepTransform);
    root.userData.topologyOffset = { x: centerX, y: centerY, z: minZ };

    const lineObjects: THREE.Line[] = [];
    const edgePickObjects: THREE.Mesh[] = [];
    const edgePointsById = new Map<string, Point3[]>();
    const edgeLinesById = new Map<string, THREE.Line>();
    topologyEdges.forEach((edge) => {
      const points = Array.isArray(edge.points) ? edge.points : [];
      if (points.length < 2) {
        return;
      }
      const geometry = new THREE.BufferGeometry().setFromPoints(
        points.map((p) => new THREE.Vector3(p.x, p.y, p.z)),
      );
      const material = new THREE.LineBasicMaterial({
        color: TOPOLOGY_EDGE_DEFAULT_COLOR,
        transparent: true,
        opacity: 0.7,
        depthTest: false,
        depthWrite: false,
      });
      const line = new THREE.Line(geometry, material);
      line.renderOrder = 11;
      line.userData.edgeId = edge.id;
      topologyModelGroup.add(line);
      const pickMesh = createEdgeOverlayMesh(
        points,
        TOPOLOGY_EDGE_DEFAULT_COLOR,
        TOPOLOGY_EDGE_PICK_RADIUS_M,
      );
      if (pickMesh) {
        const pickMaterial = pickMesh.material as THREE.MeshBasicMaterial;
        pickMaterial.opacity = 0;
        pickMaterial.transparent = true;
        pickMaterial.depthTest = false;
        pickMaterial.depthWrite = false;
        pickMesh.renderOrder = 10;
        pickMesh.userData.edgeId = edge.id;
        topologyModelGroup.add(pickMesh);
        edgePickObjects.push(pickMesh);
      }
      lineObjects.push(line);
      edgePointsById.set(edge.id, points);
      edgeLinesById.set(edge.id, line);
    });

    const refreshVisuals = () => {
      const activeSelectedId = selectedTopologyEdgeIdRef.current;
      const selectedIds = selectedTopologyEdgeIdsRef.current;
      const hoveredId = hoveredTopologyEdgeIdRef.current;

      edgeLinesById.forEach((line, edgeId) => {
        const material = line.material as THREE.LineBasicMaterial;
        if (edgeId === activeSelectedId) {
          material.transparent = false;
          material.color.setHex(TOPOLOGY_EDGE_SELECTED_COLOR);
          material.opacity = 1.0;
          line.renderOrder = 18;
        } else if (selectedIds.has(edgeId)) {
          material.transparent = false;
          material.color.setHex(TOPOLOGY_EDGE_SELECTED_COLOR);
          material.opacity = 1.0;
          line.renderOrder = 16;
        } else if (edgeId === hoveredId && weldSelectionModeRef.current) {
          material.transparent = false;
          material.color.setHex(TOPOLOGY_EDGE_HOVER_COLOR);
          material.opacity = 1.0;
          line.renderOrder = 17;
        } else {
          material.transparent = true;
          material.color.setHex(TOPOLOGY_EDGE_DEFAULT_COLOR);
          material.opacity = 0.65;
          line.renderOrder = 11;
        }
      });

      if (topologyHoverOverlayRef.current) {
        topologyModelGroup.remove(topologyHoverOverlayRef.current);
        disposeObject3D(topologyHoverOverlayRef.current);
        topologyHoverOverlayRef.current = null;
      }
      if (
        weldSelectionModeRef.current &&
        hoveredId &&
        !selectedIds.has(hoveredId) &&
        edgePointsById.has(hoveredId)
      ) {
        const mesh = createEdgeOverlayMesh(
          edgePointsById.get(hoveredId)!,
          TOPOLOGY_EDGE_HOVER_COLOR,
          TOPOLOGY_EDGE_PICK_RADIUS_M,
        );
        if (mesh) {
          topologyModelGroup.add(mesh);
          topologyHoverOverlayRef.current = mesh;
        }
      }

      if (topologySelectionOverlayRef.current) {
        topologyModelGroup.remove(topologySelectionOverlayRef.current);
        disposeObject3D(topologySelectionOverlayRef.current);
        topologySelectionOverlayRef.current = null;
      }
      if (activeSelectedId && edgePointsById.has(activeSelectedId)) {
        const mesh = createEdgeOverlayMesh(
          edgePointsById.get(activeSelectedId)!,
          TOPOLOGY_EDGE_SELECTED_COLOR,
          TOPOLOGY_EDGE_SELECTED_RADIUS_M,
        );
        if (mesh) {
          topologyModelGroup.add(mesh);
          topologySelectionOverlayRef.current = mesh;
        }
      }
    };

    scene.add(root);
    topologyEdgesGroupRef.current = root;
    topologyEdgeObjectsRef.current = lineObjects;
    topologyEdgePickObjectsRef.current = edgePickObjects;
    topologyEdgePointsByIdRef.current = edgePointsById;
    topologyEdgeLinesByIdRef.current = edgeLinesById;
    refreshTopologyEdgeVisualsRef.current = refreshVisuals;
    refreshVisuals();

    return () => {
      scene.remove(root);
      disposeObject3D(root);
      if (topologyEdgesGroupRef.current === root) {
        topologyEdgesGroupRef.current = null;
      }
      topologyEdgeObjectsRef.current = [];
      topologyEdgePickObjectsRef.current = [];
      topologyEdgePointsByIdRef.current.clear();
      topologyEdgeLinesByIdRef.current.clear();
      if (refreshTopologyEdgeVisualsRef.current === refreshVisuals) {
        refreshTopologyEdgeVisualsRef.current = null;
      }
      topologyHoverOverlayRef.current = null;
      topologySelectionOverlayRef.current = null;
    };
  }, [topologyEdges, stepTransform]);

  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) {
      return;
    }
    const existing = weldIndicatorRef.current;
    if (existing) {
      scene.remove(existing);
      disposeObject3D(existing);
      weldIndicatorRef.current = null;
    }
    if (!weldActive || !weldIndicatorPoint) {
      return;
    }

    const group = new THREE.Group();
    const glow = new THREE.Mesh(
      new THREE.SphereGeometry(0.012, 16, 16),
      new THREE.MeshBasicMaterial({
        color: 0xfb923c,
        transparent: true,
        opacity: 0.85,
      }),
    );
    const core = new THREE.Mesh(
      new THREE.SphereGeometry(0.005, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0xfef3c7 }),
    );
    group.add(glow);
    group.add(core);
    group.position.set(weldIndicatorPoint.x, weldIndicatorPoint.y, weldIndicatorPoint.z);
    scene.add(group);
    weldIndicatorRef.current = group;

    return () => {
      scene.remove(group);
      disposeObject3D(group);
      if (weldIndicatorRef.current === group) {
        weldIndicatorRef.current = null;
      }
    };
  }, [weldActive, weldIndicatorPoint]);

  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) {
      return;
    }

    const existing = weldEndpointsGroupRef.current;
    if (existing) {
      if (topologyEdgesGroupRef.current) {
        topologyEdgesGroupRef.current.remove(existing);
      } else {
        scene.remove(existing);
      }
      disposeObject3D(existing);
      weldEndpointsGroupRef.current = null;
    }

    if (!weldStartPoint || !weldStopPoint) {
      return;
    }

    const parent = topologyEdgesGroupRef.current ?? scene;
    const group = new THREE.Group();
    group.name = "weld-start-stop-markers";

    const topologyOffset = topologyEdgesGroupRef.current?.userData?.topologyOffset as
      | { x: number; y: number; z: number }
      | undefined;
    const toLocal = (point: Point3): THREE.Vector3 => {
      if (!topologyOffset) {
        return new THREE.Vector3(point.x, point.y, point.z);
      }
      return new THREE.Vector3(
        point.x - topologyOffset.x,
        point.y - topologyOffset.y,
        point.z - topologyOffset.z,
      );
    };

    const startMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.0075, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0x22c55e, depthWrite: false }),
    );
    const stopMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.0075, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0xef4444, depthWrite: false }),
    );
    startMarker.position.copy(toLocal(weldStartPoint));
    stopMarker.position.copy(toLocal(weldStopPoint));
    startMarker.renderOrder = 30;
    stopMarker.renderOrder = 30;
    group.add(startMarker);
    group.add(stopMarker);

    const segmentSource =
      Array.isArray(weldSegmentPoints) && weldSegmentPoints.length >= 2
        ? weldSegmentPoints
        : [weldStartPoint, weldStopPoint];
    if (segmentSource.length >= 2) {
      const segmentGeometry = new THREE.BufferGeometry().setFromPoints(
        segmentSource.map((point) => toLocal(point)),
      );
      const segmentLine = new THREE.Line(
        segmentGeometry,
        new THREE.LineBasicMaterial({
          color: 0xf59e0b,
          transparent: true,
          opacity: 0.9,
          depthWrite: false,
        }),
      );
      segmentLine.renderOrder = 29;
      group.add(segmentLine);
    }

    parent.add(group);
    weldEndpointsGroupRef.current = group;

    return () => {
      if (parent === scene) {
        scene.remove(group);
      } else {
        (parent as THREE.Group).remove(group);
      }
      disposeObject3D(group);
      if (weldEndpointsGroupRef.current === group) {
        weldEndpointsGroupRef.current = null;
      }
    };
  }, [weldStartPoint, weldStopPoint, weldSegmentPoints, topologyEdges, stepTransform]);

  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) {
      return;
    }

    const existing = weldAnglePreviewGroupRef.current;
    if (existing) {
      existing.parent?.remove(existing);
      disposeObject3D(existing);
      weldAnglePreviewGroupRef.current = null;
    }

    if (!weldAnglePreview) {
      return;
    }

    const segmentSource =
      Array.isArray(weldSegmentPoints) && weldSegmentPoints.length >= 2
        ? weldSegmentPoints
        : weldStartPoint && weldStopPoint
          ? [weldStartPoint, weldStopPoint]
          : [];
    if (segmentSource.length < 2) {
      return;
    }

    const topologyRoot = topologyEdgesGroupRef.current;
    const topologyOffset = topologyRoot?.userData?.topologyOffset as
      | { x: number; y: number; z: number }
      | undefined;
    topologyRoot?.updateMatrixWorld(true);
    const toWorldPoint = (point: Point3): THREE.Vector3 => {
      const local = topologyOffset
        ? new THREE.Vector3(
            point.x - topologyOffset.x,
            point.y - topologyOffset.y,
            point.z - topologyOffset.z,
          )
        : new THREE.Vector3(point.x, point.y, point.z);
      if (!topologyRoot) {
        return local;
      }
      return local.applyMatrix4(topologyRoot.matrixWorld);
    };

    const worldPoints = segmentSource.map((point) => toWorldPoint(point));
    if (worldPoints.length < 2) {
      return;
    }

    const anchorIndex = Math.max(
      0,
      Math.min(worldPoints.length - 2, Math.floor(worldPoints.length * 0.3)),
    );
    const anchor = worldPoints[anchorIndex].clone();
    const nextPoint = worldPoints[anchorIndex + 1].clone();
    const forward = normalizeVector(
      nextPoint.sub(anchor),
      new THREE.Vector3(1, 0, 0),
    );
    const worldUp = new THREE.Vector3(0, 0, 1);
    const horizontalForward = normalizeVector(
      forward.clone().sub(worldUp.clone().multiplyScalar(forward.dot(worldUp))),
      forward,
    );
    const defaultNormal = normalizeVector(
      new THREE.Vector3().crossVectors(worldUp, horizontalForward),
      new THREE.Vector3(1, 0, 0),
    );

    const workRad = THREE.MathUtils.degToRad(
      clampNumber(weldAnglePreview.workAngleDeg, -180, 180)
    );
    const travelRad = THREE.MathUtils.degToRad(
      clampNumber(weldAnglePreview.travelAngleDeg, -180, 180)
    );
    const spinRad = THREE.MathUtils.degToRad(
      clampNumber(weldAnglePreview.spinAngleDeg, -180, 180)
    );

    const normalAxis = defaultNormal;

    // Apply the same 3-plane rotations as the backend
    // Base Torch Z points down
    const baseTorch = worldUp.clone().negate();
    
    // 1. Work Angle (Normal-Up plane, around Travel axis)
    const workTorch = baseTorch.clone().applyAxisAngle(horizontalForward, workRad).normalize();
    // 2. Travel Angle (Travel-Up plane, around Normal axis)
    const travelTorch = workTorch.clone().applyAxisAngle(normalAxis, travelRad).normalize();
    // 3. Spin Angle (Travel-Normal plane, around Up axis)
    const finalTorch = travelTorch.clone().applyAxisAngle(worldUp, spinRad).normalize();

    const group = new THREE.Group();
    group.name = "weld-angle-preview";

    const addArrow = (
      direction: THREE.Vector3,
      color: number,
      length = WELD_ANGLE_PREVIEW_VECTOR_LENGTH_M,
      radius = 0.0022,
      headLength = 0.014,
      headRadius = 0.0054,
    ) => {
      const arrow = createAxisArrow(direction, color, {
        length,
        radius,
        headLength,
        headRadius,
      });
      arrow.position.copy(anchor);
      arrow.traverse((obj) => {
        if (obj instanceof THREE.Mesh) {
          const material = obj.material as THREE.MeshBasicMaterial;
          material.depthTest = false;
          material.depthWrite = false;
          material.transparent = true;
          material.opacity = 0.96;
          obj.renderOrder = 35;
        }
      });
      group.add(arrow);
    };

    const addLabel = (text: string, direction: THREE.Vector3, color: number) => {
      const label = createAxisLabelSprite(text, color, WELD_ANGLE_PREVIEW_LABEL_SCALE);
      label.position
        .copy(anchor)
        .addScaledVector(direction.clone().normalize(), WELD_ANGLE_PREVIEW_VECTOR_LENGTH_M + WELD_ANGLE_PREVIEW_LABEL_OFFSET_M);
      const material = label.material as THREE.SpriteMaterial;
      material.depthTest = false;
      material.depthWrite = false;
      label.renderOrder = 37;
      group.add(label);
    };

    addArrow(forward, 0x22d3ee);
    addArrow(worldUp, 0x3b82f6, 0.052, 0.0021, 0.012, 0.0052);
    addArrow(normalAxis, 0x22c55e, 0.054, 0.0021, 0.012, 0.0052);
    addArrow(finalTorch, 0xf97316, 0.072, 0.0024, 0.015, 0.0059);
    addLabel("Travel", forward, 0x22d3ee);
    addLabel("Up", worldUp, 0x3b82f6);
    addLabel("Normal", normalAxis, 0x22c55e);
    addLabel("Torch", finalTorch, 0xf97316);

    const workArc = createAngleArc(
      anchor,
      baseTorch,
      horizontalForward,
      workRad,
      WELD_ANGLE_PREVIEW_ARC_RADIUS_BASE_M,
      0xfacc15, // yellow
    );
    if (workArc) {
      group.add(workArc);
    }
    const travelArc = createAngleArc(
      anchor,
      workTorch,
      normalAxis,
      travelRad,
      WELD_ANGLE_PREVIEW_ARC_RADIUS_BASE_M + 0.006,
      0xfb7185, // pink-red
    );
    if (travelArc) {
      group.add(travelArc);
    }
    const spinArc = createAngleArc(
      anchor,
      travelTorch,
      worldUp,
      spinRad,
      WELD_ANGLE_PREVIEW_ARC_RADIUS_BASE_M + 0.012,
      0x34d399, // green
    );
    if (spinArc) {
      group.add(spinArc);
    }

    scene.add(group);
    weldAnglePreviewGroupRef.current = group;

    return () => {
      scene.remove(group);
      disposeObject3D(group);
    const robot = robotRef.current;
      if (weldAnglePreviewGroupRef.current === group) {
        weldAnglePreviewGroupRef.current = null;
      }
    };
  }, [
    weldAnglePreview,
    weldSegmentPoints,
    weldStartPoint,
    weldStopPoint,
    topologyEdges,
    stepTransform,
  ]);

  useEffect(() => {
    const scene = sceneRef.current;
    const robot = robotRef.current;
    if (!scene || !robot) {
      return;
    }

    const staleMarkers: THREE.Object3D[] = [];
    scene.traverse((obj) => {
      if ((obj.name ?? "").toLowerCase() === "ee-frame-ee") {
        staleMarkers.push(obj);
      }
    });
    staleMarkers.forEach((marker) => {
      marker.parent?.remove(marker);
      disposeObject3D(marker);
    });
    weldEeFrameRef.current = null;

    if (!showEndEffectorFrame || !activeTool) {
      return;
    }

    const liveEeNode = liveToolTcpNodeRef.current;
    if (liveEeNode && isToolTcpNodeForTool(liveEeNode, activeTool)) {
      const frame = createEndEffectorFrameMarker("EE", 0.9);
      liveEeNode.add(frame);
      weldEeFrameRef.current = frame;
      return () => {
        liveEeNode.remove(frame);
        disposeObject3D(frame);
        if (weldEeFrameRef.current === frame) {
          weldEeFrameRef.current = null;
        }
      };
    }
  }, [showEndEffectorFrame, activeTool, robotId, toolTcpVersion]);

  useEffect(() => {
    const scene = sceneRef.current;
    const robot = robotRef.current;
    if (!scene || !robot) {
      return;
    }

    const existing = weldGhostRobotRef.current;
    if (existing) {
      scene.remove(existing);
      disposeObject3D(existing);
      weldGhostRobotRef.current = null;
    }

    if (!weldAnglePreview || !Array.isArray(weldGhostJoints) || weldGhostJoints.length === 0) {
      return;
    }

    const ghost = robot.clone(true) as THREE.Object3D;
    ghost.name = "weld-preview-robot-ghost";
    const staleEeFrames: THREE.Object3D[] = [];
    ghost.traverse((obj) => {
      const nameLower = (obj.name ?? "").toLowerCase();
      if (nameLower.startsWith("ee-frame-")) {
        staleEeFrames.push(obj);
      }
    });
    staleEeFrames.forEach((frame) => frame.parent?.remove(frame));
    applyJointValuesToRobotHierarchy(ghost, weldGhostJoints);
    const ghostToolGroup = activeTool?.tool_id
      ? ghost.getObjectByName(`active-tool-${activeTool.tool_id}`)
      : null;
    const ghostEeNode = ghostToolGroup
      ? findToolTcpNodeForTool(ghostToolGroup, activeTool)
      : null;
    if (showEndEffectorFrame && ghostEeNode && isToolTcpNodeForTool(ghostEeNode, activeTool)) {
      ghostEeNode.add(createEndEffectorFrameMarker("EE Ghost", 0.72));
    }

    ghost.traverse((obj: THREE.Object3D) => {
      if (!(obj instanceof THREE.Mesh)) {
        return;
      }
      const cloneMaterial = (material: THREE.Material) => {
        const cloned = material.clone();
        const stylable = cloned as THREE.Material & {
          transparent?: boolean;
          opacity?: number;
          depthWrite?: boolean;
          color?: THREE.Color;
          emissive?: THREE.Color;
        };
        stylable.transparent = true;
        stylable.opacity = 0.18;
        stylable.depthWrite = false;
        if (stylable.color) {
          stylable.color = stylable.color.clone().lerp(new THREE.Color(0x67e8f9), 0.45);
        }
        if (stylable.emissive) {
          stylable.emissive = new THREE.Color(0x0ea5e9).multiplyScalar(0.15);
        }
        return cloned;
      };
      if (Array.isArray(obj.material)) {
        obj.material = obj.material.map((material) => cloneMaterial(material));
      } else {
        obj.material = cloneMaterial(obj.material);
      }
      obj.renderOrder = 32;
      obj.castShadow = false;
      obj.receiveShadow = false;
    });

    scene.add(ghost);
    weldGhostRobotRef.current = ghost;

    return () => {
      scene.remove(ghost);
      disposeObject3D(ghost);
      if (weldGhostRobotRef.current === ghost) {
        weldGhostRobotRef.current = null;
      }
    };
  }, [weldGhostJoints, weldAnglePreview, robotId, activeTool, showEndEffectorFrame]);

  useEffect(() => {
    if (!joints || joints.length === 0) {
      return;
    }
    const nextAngles = joints.slice();
    targetAnglesRef.current = nextAngles;
    // The visualized robot should be a live digital twin of telemetry, not a
    // smoothed approximation that lags behind the hardware.
    currentAnglesRef.current = nextAngles.slice();
    if (!isGroundedRef.current) {
      return;
    }
    const robot = robotRef.current;
    if (robot) {
      nextAngles.forEach((value, index) => {
        const joint = robot.joints[`joint${index + 1}`];
        if (joint) {
          joint.setJointValue(value);
        }
      });
      // Recomputing scene bounds on every telemetry packet makes the live view
      // feel sluggish. Only refresh occasionally, and only if the bounds are visible.
      if (showBoundingBoxRef.current) {
        const nowMs = performance.now();
        if (nowMs - lastLiveBoundsRefreshMsRef.current >= LIVE_BOUNDS_REFRESH_INTERVAL_MS) {
          lastLiveBoundsRefreshMsRef.current = nowMs;
          pendingDynamicBoundsRef.current = true;
        }
      }
    }
  }, [joints]);

  useEffect(() => {
    showBoundingBoxRef.current = showBoundingBox;
    const setter = setBoundsVisibilityRef.current;
    if (setter) {
      setter(showBoundingBox);
    }
  }, [showBoundingBox]);

  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) {
      return;
    }

    const disposeGroup = (group: THREE.Group | null) => {
      if (!group) {
        return;
      }
      group.traverse((object) => {
        if ((object as THREE.Mesh).isMesh) {
          const mesh = object as THREE.Mesh;
          mesh.geometry.dispose();
          if (Array.isArray(mesh.material)) {
            mesh.material.forEach((mat) => mat.dispose());
          } else {
            (mesh.material as THREE.Material).dispose();
          }
        } else if (object instanceof THREE.Line) {
          object.geometry.dispose();
          (object.material as THREE.Material).dispose();
        }
      });
    };

    if (previewGroupRef.current) {
      scene.remove(previewGroupRef.current);
      disposeGroup(previewGroupRef.current);
      previewGroupRef.current = null;
    }

    const pathList = Array.isArray(pathPoints)
      ? pathPoints.map(({ x, y, z }) => new THREE.Vector3(x, y, z))
      : [];
    const waypointList =
      Array.isArray(waypoints) && waypoints.length > 0
        ? waypoints.map(({ x, y, z }) => new THREE.Vector3(x, y, z))
        : [];

    if (pathList.length === 0 && waypointList.length === 0) {
      return;
    }

    const group = new THREE.Group();

    if (pathList.length >= 2) {
      const geometry = new THREE.BufferGeometry().setFromPoints(pathList);
      const material = new THREE.LineBasicMaterial({
        color: 0xfacc15,
        transparent: true,
        opacity: 0.7,
      });
      const line = new THREE.Line(geometry, material);
      group.add(line);

      if (highlightPathRange) {
        const start = Math.max(
          0,
          Math.min(pathList.length - 1, Math.floor(highlightPathRange.start)),
        );
        const end = Math.max(
          start,
          Math.min(pathList.length - 1, Math.ceil(highlightPathRange.end)),
        );
        const highlightedPoints = pathList.slice(start, end + 1);
        if (highlightedPoints.length >= 2) {
          const highlightGeometry = new THREE.BufferGeometry().setFromPoints(highlightedPoints);
          const highlightLine = new THREE.Line(
            highlightGeometry,
            new THREE.LineBasicMaterial({
              color: 0xf97316,
              transparent: true,
              opacity: 0.95,
            }),
          );
          highlightLine.renderOrder = 41;
          group.add(highlightLine);
        }
      }
    }

    const markerSource =
      waypointList.length > 0 ? waypointList : pathList.length > 0 ? pathList : [];
    const highlightedWaypoints = new Set(
      Array.isArray(highlightWaypointIndices)
        ? highlightWaypointIndices.filter((value) => Number.isInteger(value))
        : [],
    );
    const waypointPathIndices =
      waypointList.length > 0 ? buildOrderedWaypointPathIndices(pathList, waypointList) : [];

    markerSource.forEach((point, index) => {
      const isHighlighted = highlightedWaypoints.has(index);
      const marker = new THREE.Mesh(
        new THREE.SphereGeometry(0.001, 12, 12),
        new THREE.MeshBasicMaterial({
          color: isHighlighted
            ? 0xf97316
            : index === markerSource.length - 1
              ? 0xfacc15
              : 0xfef08a,
        }),
      );
      marker.renderOrder = isHighlighted ? 42 : 40;
      marker.position.copy(point);
      group.add(marker);

      if (waypointList.length > 0 && index < waypointList.length) {
        const label = createAxisLabelSprite(
          `CP${index + 1}`,
          isHighlighted ? 0xf97316 : 0x67e8f9,
          PREVIEW_CP_LABEL_SCALE,
        );
        label.position.copy(point).add(PREVIEW_CP_LABEL_OFFSET);
        label.renderOrder = isHighlighted ? 52 : 50;
        group.add(label);
      }
    });

    if (waypointList.length >= 2) {
      for (let index = 0; index < waypointList.length - 1; index += 1) {
        const startPoint = waypointList[index];
        const endPoint = waypointList[index + 1];
        const midpoint = startPoint.clone().lerp(endPoint, 0.5);
        let isHighlighted = false;
        if (highlightPathRange && pathList.length > 1 && waypointPathIndices.length > index + 1) {
          const rawStart = waypointPathIndices[index] ?? 0;
          const rawEnd = waypointPathIndices[index + 1] ?? rawStart;
          let segmentStart = Math.max(0, Math.min(pathList.length - 1, rawStart));
          let segmentEnd = Math.max(segmentStart, Math.min(pathList.length - 1, rawEnd));
          if (pathList.length >= 2 && segmentStart === segmentEnd) {
            if (segmentEnd < pathList.length - 1) {
              segmentEnd += 1;
            } else if (segmentStart > 0) {
              segmentStart -= 1;
            }
          }
          const selectedStart = Math.max(
            0,
            Math.min(pathList.length - 1, Math.floor(highlightPathRange.start)),
          );
          const selectedEnd = Math.max(
            selectedStart,
            Math.min(pathList.length - 1, Math.ceil(highlightPathRange.end)),
          );
          isHighlighted = selectedStart === segmentStart && selectedEnd === segmentEnd;
        }
        const moveLabel = createAxisLabelSprite(
          `M${index + 1}`,
          isHighlighted ? 0xf97316 : 0xfacc15,
          PREVIEW_MOVE_LABEL_SCALE,
        );
        moveLabel.position.copy(midpoint).add(PREVIEW_MOVE_LABEL_OFFSET);
        moveLabel.renderOrder = isHighlighted ? 52 : 50;
        group.add(moveLabel);
      }
    }

    scene.add(group);
    previewGroupRef.current = group;

    return () => {
      scene.remove(group);
      disposeGroup(group);
      if (previewGroupRef.current === group) {
        previewGroupRef.current = null;
      }
    };
  }, [pathPoints, waypoints, highlightPathRange, highlightWaypointIndices]);

  useEffect(() => {
    const canvas = containerRef.current?.querySelector("canvas");
    if (canvas) {
      canvas.style.cursor = selectionMode || weldSelectionMode ? "crosshair" : "";
    }
  }, [selectionMode, weldSelectionMode]);

  return <div ref={containerRef} className="absolute inset-0" />;
});
