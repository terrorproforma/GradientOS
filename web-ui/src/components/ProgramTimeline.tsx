export type ProgramTimelineTone = "cyan" | "amber" | "emerald" | "violet" | "slate";

export type ProgramTimelineItem = {
  id: string;
  label: string;
  metaLabel?: string;
  subtitle?: string;
  tone?: ProgramTimelineTone;
  active?: boolean;
  disabled?: boolean;
};

export type ProgramTimelineLane = {
  id: string;
  title: string;
  subtitle?: string;
  tone?: ProgramTimelineTone;
  items: ProgramTimelineItem[];
  emptyMessage?: string;
};

type ProgramTimelineProps = {
  lanes: ProgramTimelineLane[];
  playheadLabel: string;
  playheadDetail?: string;
  onSelectItem: (id: string) => void;
};

const TONE_STYLES: Record<
  ProgramTimelineTone,
  {
    rail: string;
    badge: string;
    item: string;
    active: string;
  }
> = {
  cyan: {
    rail: "from-cyan-400/70 via-cyan-400/20 to-transparent",
    badge: "border-cyan-400/40 bg-cyan-500/10 text-cyan-100",
    item: "border-slate-700/80 bg-slate-900/70 text-slate-200 hover:border-cyan-400/40 hover:bg-cyan-500/8",
    active: "border-cyan-300/70 bg-cyan-500/18 text-cyan-50 shadow-[0_0_0_1px_rgba(34,211,238,0.18)]",
  },
  amber: {
    rail: "from-amber-400/70 via-amber-400/20 to-transparent",
    badge: "border-amber-400/40 bg-amber-500/10 text-amber-100",
    item: "border-slate-700/80 bg-slate-900/70 text-slate-200 hover:border-amber-400/40 hover:bg-amber-500/8",
    active: "border-amber-300/70 bg-amber-500/18 text-amber-50 shadow-[0_0_0_1px_rgba(251,191,36,0.18)]",
  },
  emerald: {
    rail: "from-emerald-400/70 via-emerald-400/20 to-transparent",
    badge: "border-emerald-400/40 bg-emerald-500/10 text-emerald-100",
    item: "border-slate-700/80 bg-slate-900/70 text-slate-200 hover:border-emerald-400/40 hover:bg-emerald-500/8",
    active: "border-emerald-300/70 bg-emerald-500/18 text-emerald-50 shadow-[0_0_0_1px_rgba(52,211,153,0.18)]",
  },
  violet: {
    rail: "from-violet-400/70 via-violet-400/20 to-transparent",
    badge: "border-violet-400/40 bg-violet-500/10 text-violet-100",
    item: "border-slate-700/80 bg-slate-900/70 text-slate-200 hover:border-violet-400/40 hover:bg-violet-500/8",
    active: "border-violet-300/70 bg-violet-500/18 text-violet-50 shadow-[0_0_0_1px_rgba(167,139,250,0.18)]",
  },
  slate: {
    rail: "from-slate-300/60 via-slate-300/10 to-transparent",
    badge: "border-slate-500/40 bg-slate-800/80 text-slate-100",
    item: "border-slate-700/80 bg-slate-900/70 text-slate-200 hover:border-slate-500/60 hover:bg-slate-800/80",
    active: "border-slate-300/60 bg-slate-800/90 text-slate-50 shadow-[0_0_0_1px_rgba(148,163,184,0.16)]",
  },
};

function toneStyle(tone?: ProgramTimelineTone) {
  return TONE_STYLES[tone ?? "slate"];
}

export function ProgramTimeline({
  lanes,
  playheadLabel,
  playheadDetail,
  onSelectItem,
}: ProgramTimelineProps) {
  return (
    <section className="flex h-full min-h-0 flex-col overflow-hidden rounded-3xl border border-slate-800/80 bg-slate-950/78 shadow-2xl shadow-black/25 backdrop-blur">
      <div className="shrink-0 flex flex-wrap items-start justify-between gap-3 border-b border-slate-800/80 px-5 py-4">
        <div>
          <div className="text-[11px] font-semibold uppercase tracking-[0.28em] text-cyan-200/80">
            Shared Timeline
          </div>
          <div className="mt-1 text-sm text-slate-300">
            Use the same scrub surface to navigate control points, motion commands, and weld sections.
          </div>
        </div>
        <div className="max-w-[15rem] rounded-2xl border border-slate-700/80 bg-slate-900/75 px-3 py-1.5 text-right">
          <div className="truncate text-[12px] font-semibold text-slate-50">
            {playheadLabel}
            <span className="font-medium text-slate-400">
              {" "}
              {playheadDetail ? `· ${playheadDetail}` : "· Select a tree node, timeline block, or weld segment."}
            </span>
          </div>
        </div>
      </div>
      <div className="gradient-scrollbar min-h-0 flex-1 overflow-y-auto px-4 py-4">
        {lanes.length > 0 ? (
          <div className="min-h-0 space-y-4">
            {lanes.map((lane) => {
              const style = toneStyle(lane.tone);
              return (
                <div
                  key={lane.id}
                  className="rounded-2xl border border-slate-800/80 bg-slate-900/45 px-4 py-3"
                >
                  <div className="mb-3 flex flex-wrap items-center justify-between gap-2">
                    <div>
                      <div className="text-sm font-semibold text-slate-100">{lane.title}</div>
                      <div className="mt-0.5 text-[12px] text-slate-400">
                        {lane.subtitle ?? `${lane.items.length} timeline block(s)`}
                      </div>
                    </div>
                    <div
                      className={`rounded-full border px-2.5 py-1 text-[10px] font-semibold uppercase tracking-[0.18em] ${style.badge}`}
                    >
                      {lane.items.length} blocks
                    </div>
                  </div>
                  {lane.items.length > 0 ? (
                    <div className="relative">
                      <div
                        className={`pointer-events-none absolute left-6 right-6 top-[1.65rem] h-px bg-gradient-to-r ${style.rail}`}
                      />
                      <div className="gradient-scrollbar flex gap-2 overflow-x-auto overflow-y-hidden pb-1">
                        {lane.items.map((item) => {
                          const itemStyle = toneStyle(item.tone ?? lane.tone);
                          const activeClasses = item.active ? itemStyle.active : itemStyle.item;
                          return (
                            <button
                              key={item.id}
                              type="button"
                              disabled={item.disabled}
                              onClick={() => onSelectItem(item.id)}
                              className={`relative min-w-[7.5rem] rounded-xl border px-2.5 py-2 text-left transition ${activeClasses} ${
                                item.disabled ? "cursor-not-allowed opacity-50" : ""
                              }`}
                            >
                              <div className="mb-1.5 flex items-center justify-between gap-2">
                                <span className="text-[9px] font-semibold uppercase tracking-[0.22em] text-current/70">
                                  {item.metaLabel ?? "Block"}
                                </span>
                                {item.active ? (
                                  <span className="rounded-full border border-current/20 bg-black/10 px-2 py-0.5 text-[9px] font-semibold uppercase tracking-[0.18em] text-current">
                                    Active
                                  </span>
                                ) : null}
                              </div>
                              <div className="text-[12px] font-semibold text-current">{item.label}</div>
                              <div className="mt-1 line-clamp-2 text-[10px] leading-4 text-current/72">
                                {item.subtitle ?? "Jump to this authoring block."}
                              </div>
                            </button>
                          );
                        })}
                      </div>
                    </div>
                  ) : (
                    <div className="rounded-xl border border-dashed border-slate-700/70 bg-slate-950/40 px-3 py-4 text-sm text-slate-500">
                      {lane.emptyMessage ?? "No timeline data yet."}
                    </div>
                  )}
                </div>
              );
            })}
          </div>
        ) : (
          <div className="flex h-full min-h-0 items-center justify-center rounded-2xl border border-dashed border-slate-800/80 bg-slate-950/35 px-6 text-center text-sm leading-6 text-slate-400">
            Create a trajectory or weld preview to populate the shared motion timeline.
          </div>
        )}
      </div>
    </section>
  );
}
