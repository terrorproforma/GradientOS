import type { ReactNode } from "react";
import { X } from "lucide-react";

type SidebarDrawerProps = {
  children: ReactNode;
  onClose: () => void;
  headerContent?: ReactNode;
  widthClassName?: string;
  heightMode?: "content" | "full";
};

export function SidebarDrawer({
  children,
  onClose,
  headerContent,
  widthClassName = "w-full",
  heightMode = "content",
}: SidebarDrawerProps) {
  return (
    <div
      className={`flex min-h-0 flex-col overflow-hidden rounded-2xl border border-slate-700/60 bg-slate-950/72 shadow-xl shadow-black/20 backdrop-blur ${widthClassName}`}
    >
      <div className="flex items-start justify-between gap-2 border-b border-slate-700/50 px-4 py-3">
        <div className="min-w-0 flex flex-1 flex-wrap items-center gap-2">
          {headerContent ?? (
            <span className="text-xs font-semibold uppercase tracking-[0.25em] text-slate-300/90">
              Panel
            </span>
          )}
        </div>
        <button
          type="button"
          onClick={onClose}
          className="shrink-0 rounded-lg border border-slate-700/70 bg-slate-900/70 p-1 text-slate-300 transition hover:border-slate-500 hover:text-slate-100"
          aria-label="Close panel"
        >
          <X size={14} />
        </button>
      </div>
      <div
        className={`gradient-scrollbar min-h-0 flex-1 overflow-x-hidden overflow-y-auto p-3 ${
          heightMode === "full" ? "h-full" : ""
        }`}
      >
        {children}
      </div>
    </div>
  );
}
