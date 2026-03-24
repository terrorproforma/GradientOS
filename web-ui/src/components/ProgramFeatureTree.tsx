import { ChevronDown, ChevronRight } from "lucide-react";
import type { PoseWaypoint, ProgramNode, ProgramTreeViewMode } from "../previewUtils";

type ProgramFeatureTreeProps = {
  root: ProgramNode | null;
  expandedNodeIds: string[];
  selectedNodeId: string | null;
  viewMode: ProgramTreeViewMode;
  editableControlPoint: { index: number; point: PoseWaypoint } | null;
  canEditWaypointValues: boolean;
  canAddWaypoint: boolean;
  canRemoveWaypoint: boolean;
  canMoveWaypointUp: boolean;
  canMoveWaypointDown: boolean;
  canApplyWaypointEdits: boolean;
  onToggleExpand: (id: string) => void;
  onSelectNode: (id: string) => void;
  onChangeViewMode: (value: ProgramTreeViewMode) => void;
  onWaypointChange: (
    index: number,
    axis: "x" | "y" | "z" | "rollDeg" | "pitchDeg" | "yawDeg",
    value: number,
  ) => void;
  onAddWaypoint: () => void;
  onRemoveWaypoint: (index: number) => void;
  onMoveWaypointUp: () => void;
  onMoveWaypointDown: () => void;
  onApplyWaypointEdits: () => void;
};

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
  canEditWaypointValues,
  canAddWaypoint,
  canRemoveWaypoint,
  canMoveWaypointUp,
  canMoveWaypointDown,
  canApplyWaypointEdits,
  onToggleExpand,
  onSelectNode,
  onChangeViewMode,
  onWaypointChange,
  onAddWaypoint,
  onRemoveWaypoint,
  onMoveWaypointUp,
  onMoveWaypointDown,
  onApplyWaypointEdits,
}: ProgramFeatureTreeProps) {
  const expanded = new Set(expandedNodeIds);
  const parseOrKeep = (raw: string, fallback: number) => {
    const parsed = Number.parseFloat(raw);
    return Number.isFinite(parsed) ? parsed : fallback;
  };
  return (
    <div className="pointer-events-auto absolute right-6 top-6 z-30 w-[340px] max-w-[calc(100vw-2rem)] rounded-2xl border border-slate-700/70 bg-slate-950/82 p-3 shadow-2xl shadow-black/40 backdrop-blur">
      <div className="mb-2 flex items-center justify-between">
        <span className="text-xs font-semibold uppercase tracking-[0.22em] text-cyan-200/80">
          Program Tree
        </span>
        <span className="text-[10px] text-slate-400">
          {root ? root.label : "No program loaded"}
        </span>
      </div>
      <div className="mb-2 flex items-center justify-end gap-1 text-[10px]">
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
      <div className="max-h-[52vh] overflow-y-auto rounded-lg border border-slate-800/80 bg-slate-900/35 p-1">
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
        <div className="mt-2 rounded-lg border border-slate-700/70 bg-slate-900/40 p-2">
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
                aria-label="Add control point"
                title="Add control point"
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
        </div>
      ) : null}
    </div>
  );
}
