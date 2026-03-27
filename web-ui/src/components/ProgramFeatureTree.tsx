import { useEffect, useRef, useState } from "react";
import { ChevronDown, ChevronRight } from "lucide-react";
import type { PoseWaypoint, ProgramNode, ProgramTreeViewMode, WaypointMoveType } from "../previewUtils";

type ProgramFeatureTreeProps = {
  root: ProgramNode | null;
  expandedNodeIds: string[];
  selectedNodeId: string | null;
  viewMode: ProgramTreeViewMode;
  editableControlPoint: { index: number; point: PoseWaypoint } | null;
  editableMove: {
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
  } | null;
  canEditWaypointValues: boolean;
  canEditMoveType: boolean;
  canAddWaypoint: boolean;
  canRemoveWaypoint: boolean;
  canMoveWaypointUp: boolean;
  canMoveWaypointDown: boolean;
  canApplyWaypointEdits: boolean;
  showApplyWaypointEdits: boolean;
  isTrajectoryRecalculating: boolean;
  trajectoryDraftActions: {
    targetName: string;
    hasUnsavedChanges: boolean;
    canUndoLast: boolean;
    canUndoAll: boolean;
    canSave: boolean;
    isSaving: boolean;
  } | null;
  onToggleExpand: (id: string) => void;
  onSelectNode: (id: string) => void;
  onChangeViewMode: (value: ProgramTreeViewMode) => void;
  onWaypointChange: (
    index: number,
    axis: "x" | "y" | "z" | "rollDeg" | "pitchDeg" | "yawDeg",
    value: number,
  ) => void;
  onWaypointMoveTypeChange: (index: number, moveType: WaypointMoveType) => void;
  onWaypointSpeedChange: (index: number, speedMode: "linear" | "angular", value: number) => void;
  onWaypointSpeedReset: (index: number, speedMode: "linear" | "angular") => void;
  onWaypointAccelerationChange: (index: number, value: number) => void;
  onWaypointAccelerationReset: (index: number) => void;
  onWaypointPauseChange: (index: number, value: number) => void;
  onWaypointPauseReset: (index: number) => void;
  onAddWaypoint: () => void;
  onRemoveWaypoint: (index: number) => void;
  onMoveWaypointUp: () => void;
  onMoveWaypointDown: () => void;
  onApplyWaypointEdits: () => void;
  onUndoTrajectoryDraft: () => void;
  onRevertTrajectoryDraft: () => void;
  onSaveTrajectoryDraft: () => void;
};

type LinearMotionProfile = {
  distanceMm: number;
  setSpeedMmPerSec: number;
  actualPeakSpeedMmPerSec: number;
  accelerationMmPerSec2: number;
  accelDistanceMm: number;
  cruiseDistanceMm: number;
  decelStartMm: number;
  accelTimeSec: number;
  cruiseTimeSec: number;
  totalTimeSec: number;
  reachesSetSpeed: boolean;
};

function formatCompactNumber(value: number, fractionDigits = 1): string {
  return Number(value.toFixed(fractionDigits)).toLocaleString();
}

function parsePositiveDraft(rawValue: string): number | null {
  const parsed = Number.parseFloat(rawValue);
  return Number.isFinite(parsed) && parsed > 0 ? parsed : null;
}

function buildLinearMotionProfile(
  distanceMm: number,
  setSpeedMmPerSec: number | null,
  accelerationMmPerSec2: number | null,
): LinearMotionProfile | null {
  if (
    !Number.isFinite(distanceMm) ||
    distanceMm <= 0 ||
    setSpeedMmPerSec === null ||
    !Number.isFinite(setSpeedMmPerSec) ||
    setSpeedMmPerSec <= 0 ||
    accelerationMmPerSec2 === null ||
    !Number.isFinite(accelerationMmPerSec2) ||
    accelerationMmPerSec2 <= 0
  ) {
    return null;
  }
  const accelDistanceAtSetSpeedMm = (setSpeedMmPerSec * setSpeedMmPerSec) / (2 * accelerationMmPerSec2);
  const reachesSetSpeed = distanceMm >= accelDistanceAtSetSpeedMm * 2;
  const accelDistanceMm = reachesSetSpeed ? accelDistanceAtSetSpeedMm : distanceMm / 2;
  const cruiseDistanceMm = reachesSetSpeed ? Math.max(0, distanceMm - accelDistanceMm * 2) : 0;
  const actualPeakSpeedMmPerSec = reachesSetSpeed
    ? setSpeedMmPerSec
    : Math.sqrt(distanceMm * accelerationMmPerSec2);
  const accelTimeSec = actualPeakSpeedMmPerSec / accelerationMmPerSec2;
  const cruiseTimeSec = reachesSetSpeed ? cruiseDistanceMm / setSpeedMmPerSec : 0;
  return {
    distanceMm,
    setSpeedMmPerSec,
    actualPeakSpeedMmPerSec,
    accelerationMmPerSec2,
    accelDistanceMm,
    cruiseDistanceMm,
    decelStartMm: accelDistanceMm + cruiseDistanceMm,
    accelTimeSec,
    cruiseTimeSec,
    totalTimeSec: accelTimeSec + cruiseTimeSec + accelTimeSec,
    reachesSetSpeed,
  };
}

function MotionProfileChart({ profile }: { profile: LinearMotionProfile }) {
  const width = 320;
  const height = 168;
  const left = 36;
  const right = 10;
  const top = 10;
  const bottom = 24;
  const plotWidth = width - left - right;
  const plotHeight = height - top - bottom;
  const maxSpeedMmPerSec = Math.max(profile.setSpeedMmPerSec, profile.actualPeakSpeedMmPerSec, 1);
  const yMax = maxSpeedMmPerSec * 1.12;
  const xFor = (distanceMm: number) => left + (distanceMm / Math.max(profile.distanceMm, 1)) * plotWidth;
  const yFor = (speedMmPerSec: number) => top + (1 - speedMmPerSec / yMax) * plotHeight;
  const baselineY = yFor(0);
  const peakY = yFor(profile.actualPeakSpeedMmPerSec);
  const setSpeedY = yFor(profile.setSpeedMmPerSec);
  const startX = xFor(0);
  const accelEndX = xFor(profile.accelDistanceMm);
  const decelStartX = xFor(profile.decelStartMm);
  const endX = xFor(profile.distanceMm);
  const linePoints = profile.reachesSetSpeed
    ? `${startX},${baselineY} ${accelEndX},${peakY} ${decelStartX},${peakY} ${endX},${baselineY}`
    : `${startX},${baselineY} ${xFor(profile.distanceMm / 2)},${peakY} ${endX},${baselineY}`;
  const fillPoints = `${startX},${baselineY} ${linePoints} ${endX},${baselineY}`;
  const cruisePercent = profile.distanceMm > 0 ? (profile.cruiseDistanceMm / profile.distanceMm) * 100 : 0;

  return (
    <div className="mt-3 rounded-lg border border-slate-700/70 bg-slate-950/55 p-2">
      <div className="flex items-center justify-between gap-3">
        <div className="text-[10px] font-semibold uppercase tracking-[0.16em] text-cyan-100/85">
          Speed Profile
        </div>
        <div className="text-[10px] text-slate-400">
          {profile.reachesSetSpeed
            ? `${formatCompactNumber(cruisePercent, 0)}% at set speed`
            : "Too short to reach set speed"}
        </div>
      </div>
      <div className="mt-2 overflow-hidden rounded-md border border-slate-800/80 bg-[linear-gradient(180deg,rgba(14,24,39,0.9),rgba(4,8,14,0.95))]">
        <svg viewBox={`0 0 ${width} ${height}`} className="block h-44 w-full">
          <line x1={left} y1={top} x2={left} y2={baselineY} stroke="rgba(226,232,240,0.55)" strokeWidth="1.2" />
          <line x1={left} y1={baselineY} x2={width - right} y2={baselineY} stroke="rgba(226,232,240,0.55)" strokeWidth="1.2" />
          <line
            x1={left}
            y1={setSpeedY}
            x2={width - right}
            y2={setSpeedY}
            stroke="rgba(248,113,113,0.9)"
            strokeDasharray="5 4"
            strokeWidth="1"
          />
          <polyline
            points={fillPoints}
            fill="rgba(34,211,238,0.12)"
            stroke="none"
          />
          <polyline
            points={linePoints}
            fill="none"
            stroke="rgba(251,191,36,0.95)"
            strokeWidth="2.2"
            strokeLinejoin="round"
            strokeLinecap="round"
          />
          <line x1={accelEndX} y1={baselineY} x2={accelEndX} y2={height - 6} stroke="rgba(148,163,184,0.45)" strokeWidth="1" />
          <line x1={decelStartX} y1={baselineY} x2={decelStartX} y2={height - 6} stroke="rgba(148,163,184,0.45)" strokeWidth="1" />
          <text x={6} y={setSpeedY - 4} fill="rgba(248,250,252,0.82)" fontSize="9">
            {formatCompactNumber(profile.setSpeedMmPerSec, 0)} mm/s
          </text>
          <text x={left - 2} y={height - 4} textAnchor="start" fill="rgba(226,232,240,0.74)" fontSize="9">
            0
          </text>
          <text x={accelEndX} y={height - 4} textAnchor="middle" fill="rgba(226,232,240,0.74)" fontSize="9">
            {formatCompactNumber(profile.accelDistanceMm, 0)}
          </text>
          <text x={decelStartX} y={height - 4} textAnchor="middle" fill="rgba(226,232,240,0.74)" fontSize="9">
            {formatCompactNumber(profile.decelStartMm, 0)}
          </text>
          <text x={endX} y={height - 4} textAnchor="middle" fill="rgba(226,232,240,0.74)" fontSize="9">
            {formatCompactNumber(profile.distanceMm, 0)} mm
          </text>
        </svg>
      </div>
      <div className="mt-2 grid grid-cols-2 gap-2 text-[10px] leading-5 text-slate-300">
        <div className="rounded border border-slate-800/80 bg-slate-900/55 px-2 py-1.5">
          <span className="text-slate-500">Distance</span>
          <div>{formatCompactNumber(profile.distanceMm, 1)} mm</div>
        </div>
        <div className="rounded border border-slate-800/80 bg-slate-900/55 px-2 py-1.5">
          <span className="text-slate-500">Target / peak speed</span>
          <div>
            {formatCompactNumber(profile.setSpeedMmPerSec, 1)} / {formatCompactNumber(profile.actualPeakSpeedMmPerSec, 1)} mm/s
          </div>
        </div>
        <div className="rounded border border-slate-800/80 bg-slate-900/55 px-2 py-1.5">
          <span className="text-slate-500">Accel / cruise / decel</span>
          <div>
            {formatCompactNumber(profile.accelDistanceMm, 1)} / {formatCompactNumber(profile.cruiseDistanceMm, 1)} /{" "}
            {formatCompactNumber(profile.accelDistanceMm, 1)} mm
          </div>
        </div>
        <div className="rounded border border-slate-800/80 bg-slate-900/55 px-2 py-1.5">
          <span className="text-slate-500">Estimated move time</span>
          <div>{formatCompactNumber(profile.totalTimeSec, 3)} s</div>
        </div>
      </div>
      <div className="mt-2 text-[10px] leading-5 text-slate-400">
        {profile.reachesSetSpeed
          ? `Accelerates for ${formatCompactNumber(profile.accelTimeSec, 3)} s, cruises for ${formatCompactNumber(profile.cruiseTimeSec, 3)} s, then decelerates symmetrically.`
          : `This line is too short to reach the requested ${formatCompactNumber(profile.setSpeedMmPerSec, 1)} mm/s, so the move stays triangular and peaks at ${formatCompactNumber(profile.actualPeakSpeedMmPerSec, 1)} mm/s.`}
      </div>
    </div>
  );
}

function NodeRow({
  node,
  depth,
  expanded,
  selectedNodeId,
  onToggleExpand,
  onSelectNode,
}: {
  node: ProgramNode;
  depth: number;
  expanded: Set<string>;
  selectedNodeId: string | null;
  onToggleExpand: (id: string) => void;
  onSelectNode: (id: string) => void;
}) {
  const hasChildren = node.children.length > 0;
  const isExpanded = expanded.has(node.id);
  const isSelected = selectedNodeId === node.id;
  return (
    <>
      <div
        className={`flex items-center gap-1 rounded px-1 py-0.5 text-[11px] ${
          isSelected
            ? "bg-cyan-500/20 text-cyan-100"
            : "text-slate-200/90 hover:bg-slate-800/70"
        }`}
        style={{ paddingLeft: `${depth * 12 + 4}px` }}
      >
        <button
          type="button"
          onClick={() => (hasChildren ? onToggleExpand(node.id) : onSelectNode(node.id))}
          className="flex h-4 w-4 items-center justify-center text-slate-400 hover:text-slate-100"
          aria-label={hasChildren ? "Toggle node" : "Select node"}
        >
          {hasChildren ? (
            isExpanded ? (
              <ChevronDown size={12} />
            ) : (
              <ChevronRight size={12} />
            )
          ) : (
            <span className="h-1.5 w-1.5 rounded-full bg-slate-500/70" />
          )}
        </button>
        <button
          type="button"
          onClick={() => onSelectNode(node.id)}
          className="flex min-w-0 flex-1 items-center justify-between gap-2 text-left"
          title={node.subtitle ?? node.label}
        >
          <span className="flex min-w-0 items-center gap-1.5">
            <span className="truncate">{node.label}</span>
            {node.subtitle ? (
              <span className="truncate text-[10px] text-slate-500">- {node.subtitle}</span>
            ) : null}
          </span>
          {node.badge ? (
            <span className="rounded border border-slate-700/80 bg-slate-900/80 px-1.5 py-0.5 text-[10px] text-slate-300">
              {node.badge}
            </span>
          ) : null}
        </button>
      </div>
      {hasChildren && isExpanded
        ? node.children.map((child) => (
            <NodeRow
              key={child.id}
              node={child}
              depth={depth + 1}
              expanded={expanded}
              selectedNodeId={selectedNodeId}
              onToggleExpand={onToggleExpand}
              onSelectNode={onSelectNode}
            />
          ))
        : null}
    </>
  );
}

export function ProgramFeatureTree({
  root,
  expandedNodeIds,
  selectedNodeId,
  viewMode,
  editableControlPoint,
  editableMove,
  canEditWaypointValues,
  canEditMoveType,
  canAddWaypoint,
  canRemoveWaypoint,
  canMoveWaypointUp,
  canMoveWaypointDown,
  canApplyWaypointEdits,
  showApplyWaypointEdits,
  isTrajectoryRecalculating,
  trajectoryDraftActions,
  onToggleExpand,
  onSelectNode,
  onChangeViewMode,
  onWaypointChange,
  onWaypointMoveTypeChange,
  onWaypointSpeedChange,
  onWaypointSpeedReset,
  onWaypointAccelerationChange,
  onWaypointAccelerationReset,
  onWaypointPauseChange,
  onWaypointPauseReset,
  onAddWaypoint,
  onRemoveWaypoint,
  onMoveWaypointUp,
  onMoveWaypointDown,
  onApplyWaypointEdits,
  onUndoTrajectoryDraft,
  onRevertTrajectoryDraft,
  onSaveTrajectoryDraft,
}: ProgramFeatureTreeProps) {
  const expanded = new Set(expandedNodeIds);
  const lastEditableMoveRef = useRef<NonNullable<ProgramFeatureTreeProps["editableMove"]> | null>(null);
  const moveSpeedCommitTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const moveAccelerationCommitTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const movePauseCommitTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const [moveSpeedDraft, setMoveSpeedDraft] = useState("");
  const [moveAccelerationDraft, setMoveAccelerationDraft] = useState("");
  const [movePauseDraft, setMovePauseDraft] = useState("");
  if (editableMove) {
    lastEditableMoveRef.current = editableMove;
  } else if (!isTrajectoryRecalculating) {
    lastEditableMoveRef.current = null;
  }
  const visibleEditableMove =
    editableMove ?? (isTrajectoryRecalculating ? lastEditableMoveRef.current : null);
  const formatNumericDraft = (value: number | null, decimals: number) =>
    value === null ? "" : String(Number(value.toFixed(decimals)));
  const formatMoveSpeedValue = (value: number | null, speedMode: "linear" | "angular") =>
    formatNumericDraft(value, speedMode === "angular" ? 1 : 0);
  const parseOrKeep = (raw: string, fallback: number) => {
    const parsed = Number.parseFloat(raw);
    return Number.isFinite(parsed) ? parsed : fallback;
  };
  const clearPendingMoveSpeedCommit = () => {
    if (moveSpeedCommitTimeoutRef.current) {
      clearTimeout(moveSpeedCommitTimeoutRef.current);
      moveSpeedCommitTimeoutRef.current = null;
    }
  };
  const clearPendingMoveAccelerationCommit = () => {
    if (moveAccelerationCommitTimeoutRef.current) {
      clearTimeout(moveAccelerationCommitTimeoutRef.current);
      moveAccelerationCommitTimeoutRef.current = null;
    }
  };
  const clearPendingMovePauseCommit = () => {
    if (movePauseCommitTimeoutRef.current) {
      clearTimeout(movePauseCommitTimeoutRef.current);
      movePauseCommitTimeoutRef.current = null;
    }
  };
  const commitMoveSpeedDraft = (
    rawValue: string,
    targetMove = visibleEditableMove,
  ) => {
    clearPendingMoveSpeedCommit();
    if (!targetMove) {
      return;
    }
    const trimmed = rawValue.trim();
    if (!trimmed) {
      onWaypointSpeedReset(targetMove.waypointIndex, targetMove.speedMode);
      return;
    }
    const parsed = Number.parseFloat(trimmed);
    if (Number.isFinite(parsed) && parsed > 0) {
      onWaypointSpeedChange(targetMove.waypointIndex, targetMove.speedMode, parsed);
      return;
    }
    setMoveSpeedDraft(formatMoveSpeedValue(targetMove.speedValue, targetMove.speedMode));
  };
  const commitMoveAccelerationDraft = (
    rawValue: string,
    targetMove = visibleEditableMove,
  ) => {
    clearPendingMoveAccelerationCommit();
    if (!targetMove || !targetMove.supportsAccelerationEdit) {
      return;
    }
    const trimmed = rawValue.trim();
    if (!trimmed) {
      onWaypointAccelerationReset(targetMove.waypointIndex);
      return;
    }
    const parsed = Number.parseFloat(trimmed);
    if (Number.isFinite(parsed) && parsed > 0) {
      onWaypointAccelerationChange(targetMove.waypointIndex, parsed);
      return;
    }
    setMoveAccelerationDraft(formatNumericDraft(targetMove.accelerationValue, 0));
  };
  const commitMovePauseDraft = (
    rawValue: string,
    targetMove = visibleEditableMove,
  ) => {
    clearPendingMovePauseCommit();
    if (!targetMove || !targetMove.supportsPauseEdit) {
      return;
    }
    const trimmed = rawValue.trim();
    if (!trimmed) {
      onWaypointPauseReset(targetMove.waypointIndex);
      return;
    }
    const parsed = Number.parseFloat(trimmed);
    if (Number.isFinite(parsed) && parsed > 0) {
      onWaypointPauseChange(targetMove.waypointIndex, parsed);
      return;
    }
    setMovePauseDraft(formatNumericDraft(targetMove.pauseAfterSeconds, 1));
  };
  useEffect(() => {
    clearPendingMoveSpeedCommit();
    clearPendingMoveAccelerationCommit();
    clearPendingMovePauseCommit();
    if (!visibleEditableMove) {
      setMoveSpeedDraft("");
      setMoveAccelerationDraft("");
      setMovePauseDraft("");
      return;
    }
    setMoveSpeedDraft(formatMoveSpeedValue(visibleEditableMove.speedValue, visibleEditableMove.speedMode));
    setMoveAccelerationDraft(formatNumericDraft(visibleEditableMove.accelerationValue, 0));
    setMovePauseDraft(formatNumericDraft(visibleEditableMove.pauseAfterSeconds, 1));
  }, [
    visibleEditableMove?.moveIndex,
    visibleEditableMove?.waypointIndex,
    visibleEditableMove?.speedMode,
    visibleEditableMove?.speedValue,
    visibleEditableMove?.supportsAccelerationEdit,
    visibleEditableMove?.accelerationValue,
    visibleEditableMove?.supportsPauseEdit,
    visibleEditableMove?.pauseAfterSeconds,
  ]);
  useEffect(() => () => {
    clearPendingMoveSpeedCommit();
    clearPendingMoveAccelerationCommit();
    clearPendingMovePauseCommit();
  }, []);
  const profileSpeedValue =
    visibleEditableMove?.speedMode === "linear"
      ? parsePositiveDraft(moveSpeedDraft) ?? visibleEditableMove?.speedValue ?? null
      : null;
  const profileAccelerationDraftValue =
    visibleEditableMove?.supportsAccelerationEdit
      ? parsePositiveDraft(moveAccelerationDraft) ?? visibleEditableMove?.accelerationValue ?? null
      : null;
  const profileAccelerationValue =
    visibleEditableMove?.supportsAccelerationEdit && profileSpeedValue !== null
      ? profileAccelerationDraftValue ?? profileSpeedValue
      : null;
  const linearMotionProfile =
    visibleEditableMove?.supportsAccelerationEdit
      ? buildLinearMotionProfile(
          visibleEditableMove.moveDistanceMm,
          profileSpeedValue,
          profileAccelerationValue,
        )
      : null;
  return (
    <section className="flex h-full min-h-0 flex-col overflow-hidden rounded-2xl border border-slate-700/70 bg-slate-950/72 shadow-xl shadow-black/20 backdrop-blur">
      <div className="flex items-center justify-between border-b border-slate-700/50 px-4 py-3">
        <span className="text-xs font-semibold uppercase tracking-[0.22em] text-cyan-200/80">
          Program Tree
        </span>
        <span className="text-[10px] text-slate-400">
          {root ? root.label : "No program loaded"}
        </span>
      </div>
      <div className="flex items-center justify-between gap-3 px-4 py-3">
        <div className="text-[12px] text-slate-400">
          Select a node to sync the editor, timeline, and 3D stage.
        </div>
        <div className="flex items-center justify-end gap-1 text-[10px]">
          <button
            type="button"
            onClick={() => onChangeViewMode("chronological")}
            className={`rounded border px-2 py-1 ${
              viewMode === "chronological"
                ? "border-cyan-400/60 bg-cyan-500/20 text-cyan-100"
                : "border-slate-700/80 bg-slate-900/70 text-slate-300 hover:border-slate-500"
            }`}
          >
            Chronological
          </button>
          <button
            type="button"
            onClick={() => onChangeViewMode("grouped")}
            className={`rounded border px-2 py-1 ${
              viewMode === "grouped"
                ? "border-cyan-400/60 bg-cyan-500/20 text-cyan-100"
                : "border-slate-700/80 bg-slate-900/70 text-slate-300 hover:border-slate-500"
            }`}
          >
            Grouped
          </button>
        </div>
      </div>
      <div className="flex min-h-0 flex-1 flex-col px-3 pb-3">
        <div className="flex h-full min-h-0 flex-col rounded-xl border border-slate-800/80 bg-slate-900/35 p-2">
          <div className="min-h-0 flex-1 overflow-y-auto rounded-lg px-1 py-1">
            {root ? (
              <NodeRow
                node={root}
                depth={0}
                expanded={expanded}
                selectedNodeId={selectedNodeId}
                onToggleExpand={onToggleExpand}
                onSelectNode={onSelectNode}
              />
            ) : (
              <div className="px-2 py-2 text-xs text-slate-500">
                Plan a trajectory or weld to populate the tree.
              </div>
            )}
          </div>
          {editableControlPoint ? (
            <div className="mt-2 rounded-lg border border-slate-700/70 bg-slate-900/45 p-2">
              <div className="mb-1 flex items-center justify-between">
                <span className="text-[11px] font-semibold uppercase tracking-[0.12em] text-cyan-200/85">
                  Edit Control Point {editableControlPoint.index + 1}
                </span>
                <div className="flex items-center gap-1">
                  <button
                    type="button"
                    onClick={onAddWaypoint}
                    disabled={!canAddWaypoint}
                    className={`rounded border border-slate-600/70 px-1.5 py-0.5 text-[11px] text-slate-200 transition hover:border-slate-400 ${
                      !canAddWaypoint ? "opacity-60" : ""
                    }`}
                    aria-label="Add joint control point"
                    title="Add joint control point"
                  >
                    +
                  </button>
                  <button
                    type="button"
                    onClick={() => onRemoveWaypoint(editableControlPoint.index)}
                    disabled={!canRemoveWaypoint}
                    className={`rounded border border-slate-600/70 px-1.5 py-0.5 text-[11px] text-slate-200 transition hover:border-slate-400 ${
                      !canRemoveWaypoint ? "opacity-60" : ""
                    }`}
                    aria-label="Remove control point"
                    title="Remove control point"
                  >
                    -
                  </button>
                  <button
                    type="button"
                    onClick={onMoveWaypointUp}
                    disabled={!canMoveWaypointUp}
                    className={`rounded border border-slate-600/70 px-1.5 py-0.5 text-[11px] text-slate-200 transition hover:border-slate-400 ${
                      !canMoveWaypointUp ? "opacity-60" : ""
                    }`}
                    aria-label="Move control point up"
                    title="Move control point earlier"
                  >
                    ^
                  </button>
                  <button
                    type="button"
                    onClick={onMoveWaypointDown}
                    disabled={!canMoveWaypointDown}
                    className={`rounded border border-slate-600/70 px-1.5 py-0.5 text-[11px] text-slate-200 transition hover:border-slate-400 ${
                      !canMoveWaypointDown ? "opacity-60" : ""
                    }`}
                    aria-label="Move control point down"
                    title="Move control point later"
                  >
                    v
                  </button>
                </div>
              </div>
              <div className="grid grid-cols-3 gap-1">
                {(["x", "y", "z"] as const).map((axis) => (
                  <input
                    key={`tree-control-${editableControlPoint.index}-${axis}`}
                    className={`rounded border border-slate-600/70 bg-slate-950/70 px-1.5 py-1 text-[12px] text-slate-100 ${
                      !canEditWaypointValues ? "opacity-60" : ""
                    }`}
                    type="number"
                    step="0.001"
                    value={Number(editableControlPoint.point[axis].toFixed(4))}
                    disabled={!canEditWaypointValues}
                    onChange={(event) =>
                      onWaypointChange(
                        editableControlPoint.index,
                        axis,
                        parseOrKeep(event.target.value, editableControlPoint.point[axis]),
                      )
                    }
                  />
                ))}
              </div>
              <div className="mt-1 grid grid-cols-3 gap-1">
                {(
                  [
                    ["rollDeg", "Roll"],
                    ["pitchDeg", "Pitch"],
                    ["yawDeg", "Yaw"],
                  ] as const
                ).map(([axis, label]) => (
                  <input
                    key={`tree-control-rot-${editableControlPoint.index}-${axis}`}
                    className={`rounded border border-slate-600/70 bg-slate-950/70 px-1.5 py-1 text-[12px] text-slate-100 ${
                      !canEditWaypointValues ? "opacity-60" : ""
                    }`}
                    type="number"
                    step="0.1"
                    title={label}
                    value={editableControlPoint.point[axis] ?? 0}
                    disabled={!canEditWaypointValues}
                    onChange={(event) =>
                      onWaypointChange(
                        editableControlPoint.index,
                        axis,
                        parseOrKeep(event.target.value, editableControlPoint.point[axis] ?? 0),
                      )
                    }
                  />
                ))}
              </div>
              {showApplyWaypointEdits ? (
                <button
                  type="button"
                  onClick={onApplyWaypointEdits}
                  disabled={!canApplyWaypointEdits}
                  className={`mt-2 w-full rounded border border-slate-600/70 bg-slate-900/60 px-2 py-1.5 text-[12px] font-semibold text-slate-100 transition hover:border-slate-400 ${
                    !canApplyWaypointEdits ? "opacity-60" : ""
                  }`}
                >
                  Apply Waypoint Edits
                </button>
              ) : (
                <div className="mt-2 rounded border border-dashed border-slate-700/60 bg-slate-950/40 px-2 py-1.5 text-[11px] leading-5 text-slate-400">
                  Preview updates automatically while you edit this trajectory draft.
                </div>
              )}
            </div>
          ) : null}
          {visibleEditableMove ? (
            <div className="mt-2 rounded-lg border border-slate-700/70 bg-slate-900/45 p-2">
              <div className="mb-1 flex items-center justify-between">
                <span className="text-[11px] font-semibold uppercase tracking-[0.12em] text-amber-200/85">
                  Edit Move {visibleEditableMove.moveIndex + 1}
                </span>
                <span className="text-[10px] text-slate-400">
                  P{visibleEditableMove.startWaypointIndex + 1} -&gt; P{visibleEditableMove.endWaypointIndex + 1}
                </span>
              </div>
              <select
                className={`w-full rounded border border-slate-600/70 bg-slate-950/70 px-1.5 py-1 text-[12px] text-slate-100 ${
                  !canEditMoveType ? "opacity-60" : ""
                }`}
                value={visibleEditableMove.moveType}
                disabled={!canEditMoveType}
                onChange={(event) =>
                  onWaypointMoveTypeChange(
                    visibleEditableMove.waypointIndex,
                    event.target.value === "joint" || event.target.value === "home"
                      ? event.target.value
                      : "linear",
                  )
                }
              >
                <option value="linear">Linear move</option>
                <option value="joint">Joint move</option>
                <option value="home">Move to home</option>
              </select>
              <div className="mt-2">
                <div className="mb-1 flex items-center justify-between gap-2 text-[10px] uppercase tracking-[0.14em] text-slate-400">
                  <span>Speed</span>
                  <span>{visibleEditableMove.speedUnitLabel}</span>
                </div>
                <div className="flex items-center gap-2">
                  <input
                    className="min-w-0 flex-1 rounded border border-slate-600/70 bg-slate-950/70 px-1.5 py-1 text-[12px] text-slate-100"
                    type="number"
                    min="0.1"
                    step={visibleEditableMove.speedStep}
                    value={moveSpeedDraft}
                    placeholder={`Controller default (${visibleEditableMove.speedUnitLabel})`}
                    disabled={!canEditMoveType}
                    onChange={(event) => {
                      const nextValue = event.target.value;
                      setMoveSpeedDraft(nextValue);
                      clearPendingMoveSpeedCommit();
                      moveSpeedCommitTimeoutRef.current = setTimeout(() => {
                        commitMoveSpeedDraft(nextValue, visibleEditableMove);
                      }, 900);
                    }}
                    onBlur={() => commitMoveSpeedDraft(moveSpeedDraft, visibleEditableMove)}
                    onKeyDown={(event) => {
                      if (event.key === "Enter") {
                        commitMoveSpeedDraft(moveSpeedDraft, visibleEditableMove);
                        event.currentTarget.blur();
                      }
                    }}
                  />
                  <button
                    type="button"
                    onClick={() => {
                      clearPendingMoveSpeedCommit();
                      setMoveSpeedDraft("");
                      onWaypointSpeedReset(visibleEditableMove.waypointIndex, visibleEditableMove.speedMode);
                    }}
                    disabled={!canEditMoveType || !visibleEditableMove.hasCustomSpeed}
                    className={`rounded border border-slate-600/70 px-2 py-1 text-[11px] font-semibold text-slate-200 transition hover:border-slate-400 ${
                      !canEditMoveType || !visibleEditableMove.hasCustomSpeed ? "opacity-60" : ""
                    }`}
                  >
                    Auto
                  </button>
                </div>
                <div className="mt-1 text-[10px] leading-5 text-slate-400">
                  {visibleEditableMove.hasCustomSpeed
                    ? "Custom move speed saved on this segment."
                    : `Controller default remains active until you enter an explicit ${visibleEditableMove.speedUnitLabel} override.`}
                </div>
                {visibleEditableMove.supportsAccelerationEdit ? (
                  <div className="mt-3">
                    <div className="mb-1 flex items-center justify-between gap-2 text-[10px] uppercase tracking-[0.14em] text-slate-400">
                      <span>Acceleration</span>
                      <span>{visibleEditableMove.accelerationUnitLabel}</span>
                    </div>
                    <div className="flex items-center gap-2">
                      <input
                        className="min-w-0 flex-1 rounded border border-slate-600/70 bg-slate-950/70 px-1.5 py-1 text-[12px] text-slate-100"
                        type="number"
                        min="0.1"
                        step={visibleEditableMove.accelerationStep}
                        value={moveAccelerationDraft}
                        placeholder={`Controller default (${visibleEditableMove.accelerationUnitLabel})`}
                        disabled={!canEditMoveType}
                        onChange={(event) => {
                          const nextValue = event.target.value;
                          setMoveAccelerationDraft(nextValue);
                          clearPendingMoveAccelerationCommit();
                          moveAccelerationCommitTimeoutRef.current = setTimeout(() => {
                            commitMoveAccelerationDraft(nextValue, visibleEditableMove);
                          }, 900);
                        }}
                        onBlur={() => commitMoveAccelerationDraft(moveAccelerationDraft, visibleEditableMove)}
                        onKeyDown={(event) => {
                          if (event.key === "Enter") {
                            commitMoveAccelerationDraft(moveAccelerationDraft, visibleEditableMove);
                            event.currentTarget.blur();
                          }
                        }}
                      />
                      <button
                        type="button"
                        onClick={() => {
                          clearPendingMoveAccelerationCommit();
                          setMoveAccelerationDraft("");
                          onWaypointAccelerationReset(visibleEditableMove.waypointIndex);
                        }}
                        disabled={!canEditMoveType || !visibleEditableMove.hasCustomAcceleration}
                        className={`rounded border border-slate-600/70 px-2 py-1 text-[11px] font-semibold text-slate-200 transition hover:border-slate-400 ${
                          !canEditMoveType || !visibleEditableMove.hasCustomAcceleration ? "opacity-60" : ""
                        }`}
                      >
                        Auto
                      </button>
                    </div>
                    <div className="mt-1 text-[10px] leading-5 text-slate-400">
                      {visibleEditableMove.hasCustomAcceleration
                        ? "Custom line acceleration saved on this segment."
                        : "Controller default line acceleration targets full commanded speed in about 1 second unless you override it."}
                    </div>
                  </div>
                ) : null}
                {visibleEditableMove.supportsPauseEdit ? (
                  <div className="mt-3">
                    <div className="mb-1 flex items-center justify-between gap-2 text-[10px] uppercase tracking-[0.14em] text-slate-400">
                      <span>Pause After Move</span>
                      <span>s</span>
                    </div>
                    <div className="flex items-center gap-2">
                      <input
                        className="min-w-0 flex-1 rounded border border-slate-600/70 bg-slate-950/70 px-1.5 py-1 text-[12px] text-slate-100"
                        type="number"
                        min="0.1"
                        step={visibleEditableMove.pauseStep}
                        value={movePauseDraft}
                        placeholder="No pause"
                        disabled={!canEditMoveType}
                        onChange={(event) => {
                          const nextValue = event.target.value;
                          setMovePauseDraft(nextValue);
                          clearPendingMovePauseCommit();
                          movePauseCommitTimeoutRef.current = setTimeout(() => {
                            commitMovePauseDraft(nextValue, visibleEditableMove);
                          }, 900);
                        }}
                        onBlur={() => commitMovePauseDraft(movePauseDraft, visibleEditableMove)}
                        onKeyDown={(event) => {
                          if (event.key === "Enter") {
                            commitMovePauseDraft(movePauseDraft, visibleEditableMove);
                            event.currentTarget.blur();
                          }
                        }}
                      />
                      <button
                        type="button"
                        onClick={() => {
                          clearPendingMovePauseCommit();
                          setMovePauseDraft("");
                          onWaypointPauseReset(visibleEditableMove.waypointIndex);
                        }}
                        disabled={!canEditMoveType || !visibleEditableMove.hasCustomPause}
                        className={`rounded border border-slate-600/70 px-2 py-1 text-[11px] font-semibold text-slate-200 transition hover:border-slate-400 ${
                          !canEditMoveType || !visibleEditableMove.hasCustomPause ? "opacity-60" : ""
                        }`}
                      >
                        None
                      </button>
                    </div>
                    <div className="mt-1 text-[10px] leading-5 text-slate-400">
                      {visibleEditableMove.hasCustomPause
                        ? "Hold at this waypoint before the next authored move starts."
                        : "Leave blank for continuous motion with no inserted pause."}
                    </div>
                  </div>
                ) : null}
                {linearMotionProfile ? <MotionProfileChart profile={linearMotionProfile} /> : null}
                {visibleEditableMove.supportsAccelerationEdit && !linearMotionProfile ? (
                  <div className="mt-3 rounded border border-dashed border-slate-700/60 bg-slate-950/35 px-2 py-1.5 text-[10px] leading-5 text-slate-400">
                    Profile graph needs a resolved line speed before it can be drawn for this move.
                  </div>
                ) : null}
                {!visibleEditableMove.supportsAccelerationEdit ? (
                  <div className="mt-3 rounded border border-dashed border-slate-700/60 bg-slate-950/35 px-2 py-1.5 text-[10px] leading-5 text-slate-400">
                    Speed profile graph is shown for linear line segments. Joint, home, and pure-rotation moves
                    currently use different timing models.
                  </div>
                ) : null}
                {isTrajectoryRecalculating ? (
                  <div className="mt-2 rounded border border-cyan-500/25 bg-cyan-500/8 px-2 py-1.5 text-[10px] uppercase tracking-[0.14em] text-cyan-100/85">
                    Recalculating trajectory preview...
                  </div>
                ) : null}
              </div>
            </div>
          ) : null}
          {trajectoryDraftActions ? (
            <div className="mt-2 rounded-lg border border-slate-700/70 bg-slate-900/45 p-2">
              <div className="mb-1 flex items-center justify-between gap-2">
                <span className="text-[11px] font-semibold uppercase tracking-[0.12em] text-cyan-200/85">
                  Trajectory Draft
                </span>
                <span className="text-[10px] text-slate-400">
                  {trajectoryDraftActions.hasUnsavedChanges ? "Unsaved changes" : "Saved"}
                </span>
              </div>
              <div className="mb-2 text-[11px] leading-5 text-slate-400">
                Preview changes stay local until you save to <span className="text-slate-200">{trajectoryDraftActions.targetName}</span>.
              </div>
              <div className="grid grid-cols-3 gap-2">
                <button
                  type="button"
                  onClick={onUndoTrajectoryDraft}
                  disabled={!trajectoryDraftActions.canUndoLast}
                  className={`rounded border border-slate-600/70 bg-slate-900/60 px-2 py-1.5 text-[12px] font-semibold text-slate-100 transition hover:border-slate-400 ${
                    !trajectoryDraftActions.canUndoLast ? "opacity-60" : ""
                  }`}
                >
                  Undo Last
                </button>
                <button
                  type="button"
                  onClick={onRevertTrajectoryDraft}
                  disabled={!trajectoryDraftActions.canUndoAll}
                  className={`rounded border border-slate-600/70 bg-slate-900/60 px-2 py-1.5 text-[12px] font-semibold text-slate-100 transition hover:border-slate-400 ${
                    !trajectoryDraftActions.canUndoAll ? "opacity-60" : ""
                  }`}
                >
                  Undo All
                </button>
                <button
                  type="button"
                  onClick={onSaveTrajectoryDraft}
                  disabled={!trajectoryDraftActions.canSave}
                  className={`rounded border border-cyan-500/50 bg-cyan-500/10 px-2 py-1.5 text-[12px] font-semibold text-cyan-100 transition hover:border-cyan-300 hover:text-cyan-50 ${
                    !trajectoryDraftActions.canSave ? "opacity-60" : ""
                  }`}
                >
                  {trajectoryDraftActions.isSaving ? "Saving..." : "Save"}
                </button>
              </div>
            </div>
          ) : null}
        </div>
      </div>
    </section>
  );
}
