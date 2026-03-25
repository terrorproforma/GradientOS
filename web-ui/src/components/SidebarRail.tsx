import type { ReactNode } from "react";

export type SidebarItem = {
  id: string;
  label: string;
  icon: ReactNode;
  shortcut?: string;
  disabled?: boolean;
};

type SidebarRailProps = {
  items: SidebarItem[];
  activeItemId: string | null;
  onSelect: (id: string) => void;
};

export function SidebarRail({ items, activeItemId, onSelect }: SidebarRailProps) {
  return (
    <div className="relative z-30 flex h-full min-h-0 flex-col items-center gap-3 overflow-visible rounded-[1.65rem] border border-slate-700/70 bg-slate-950/78 px-2.5 py-3 shadow-xl shadow-black/25 backdrop-blur">
      <div className="flex h-11 w-11 items-center justify-center rounded-2xl border border-slate-700/80 bg-slate-900/80 text-slate-300">
        <div className="flex flex-col gap-1.5">
          <span className="block h-[2px] w-4 rounded-full bg-current/90" />
          <span className="block h-[2px] w-4 rounded-full bg-current/90" />
          <span className="block h-[2px] w-4 rounded-full bg-current/90" />
        </div>
      </div>
      <div className="h-px w-8 bg-slate-700/70" />
      <div className="flex min-h-0 flex-1 flex-col items-center gap-2 overflow-visible">
        {items.map((item) => {
          const active = item.id === activeItemId;
          return (
            <button
              key={item.id}
              type="button"
              disabled={item.disabled}
              onClick={() => onSelect(item.id)}
              title={item.shortcut ? `${item.label} (${item.shortcut})` : item.label}
              className={`group relative flex h-11 w-11 items-center justify-center rounded-2xl border transition ${
                active
                  ? "border-cyan-400/70 bg-cyan-500/18 text-cyan-100 shadow-[inset_0_0_0_1px_rgba(34,211,238,0.18)]"
                  : "border-slate-700/80 bg-slate-900/70 text-slate-300 hover:border-slate-500/80 hover:bg-slate-800/90 hover:text-slate-100"
              } ${item.disabled ? "cursor-not-allowed opacity-50" : ""}`}
            >
              {item.icon}
              <span
                className="pointer-events-none absolute left-[calc(100%+0.85rem)] top-1/2 z-[80] min-w-[13rem] -translate-y-1/2 translate-x-[-6px] rounded-2xl border border-slate-700/85 bg-slate-950/98 px-3 py-2 text-left opacity-0 shadow-2xl shadow-black/45 ring-1 ring-inset ring-slate-700/40 transition group-hover:translate-x-0 group-hover:opacity-100"
              >
                <span className="block text-[13px] font-semibold text-slate-50">{item.label}</span>
                <span className="mt-0.5 block text-[11px] text-slate-400">
                  {item.shortcut ? `Shortcut ${item.shortcut}` : "Open docked pane"}
                </span>
              </span>
            </button>
          );
        })}
      </div>
      <div className="rounded-full border border-slate-700/80 bg-slate-900/75 px-2.5 py-1 text-[10px] font-semibold uppercase tracking-[0.18em] text-slate-400">
        Menu
      </div>
    </div>
  );
}
