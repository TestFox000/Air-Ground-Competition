import { Check, Loader2, X } from "lucide-react";
import { STAGE_KEYS, STAGE_META, type StageState } from "@contracts/pipeline";
import { STAGE_ICONS } from "./stage-icons";
import { cn } from "@/lib/utils";

function StageIcon({ state }: { state: StageState | undefined }) {
  const status = state?.status ?? "pending";
  if (status === "running") return <Loader2 className="h-3.5 w-3.5 animate-stage-spin text-hud" />;
  if (status === "done") return <Check className="h-3.5 w-3.5 text-mintsig" strokeWidth={3} />;
  if (status === "failed") return <X className="h-3.5 w-3.5 text-redsig" strokeWidth={3} />;
  return <span className="h-1.5 w-1.5 bg-muted-foreground/40" />;
}

export function StageTimeline({ stages, progress }: { stages: StageState[] | null | undefined; progress: number }) {
  return (
    <div>
      {/* 总进度条 */}
      <div className="mb-4">
        <div className="mb-1.5 flex items-baseline justify-between">
          <span className="label-tech">总体进度 PROGRESS</span>
          <span className="font-data hud-glow-soft text-[15px] font-semibold text-hud">{progress}%</span>
        </div>
        <div className="h-[6px] border border-hairline bg-background">
          <div
            className="h-full bg-hud transition-[width] duration-300 ease-out"
            style={{ width: `${progress}%`, boxShadow: "0 0 10px hsl(var(--hud) / 0.5)" }}
          />
        </div>
      </div>

      <ol>
        {STAGE_KEYS.map((key, i) => {
          const s = stages?.find((x) => x.key === key);
          const status = s?.status ?? "pending";
          const meta = STAGE_META[key];
          const StageGlyph = STAGE_ICONS[key];
          const last = i === STAGE_KEYS.length - 1;
          return (
            <li key={key} className="relative flex gap-3 pb-3.5 last:pb-0">
              {!last && (
                <span
                  className={cn(
                    "absolute left-[13px] top-8 h-[calc(100%-26px)] w-px",
                    status === "done" ? "bg-mintsig/50" : "bg-hairline",
                  )}
                />
              )}
              <span
                className={cn(
                  "mt-0.5 flex h-7 w-7 shrink-0 flex-col items-center justify-center border",
                  status === "running" && "border-hud/60 bg-hud/10 text-hud",
                  status === "done" && "border-mintsig/50 bg-mintsig/10 text-mintsig",
                  status === "failed" && "border-redsig/50 bg-redsig/10 text-redsig",
                  status === "pending" && "border-hairline bg-panel/50 text-muted-foreground/60",
                )}
              >
                <StageGlyph className="text-[14px]" />
              </span>
              <div className="min-w-0 flex-1">
                <div className="flex items-baseline justify-between gap-2">
                  <span
                    className={cn(
                      "flex items-center gap-1.5 text-[13px] font-medium",
                      status === "running" && "text-hud hud-glow-soft",
                      status === "done" && "text-foreground",
                      status === "failed" && "text-redsig",
                      status === "pending" && "text-muted-foreground",
                    )}
                  >
                    {meta.label}
                    <StageIcon state={s} />
                  </span>
                  <span className="font-data text-[10px] tracking-[0.14em] text-muted-foreground/70">
                    {String(i + 1).padStart(2, "0")}/{meta.eng}
                    {s?.durationMs != null && ` · ${(s.durationMs / 1000).toFixed(1)}s`}
                  </span>
                </div>
                {status === "running" && s?.logs.length ? (
                  <p className="mt-0.5 truncate font-data text-[11px] text-hud/80">
                    {s.logs[s.logs.length - 1].msg}
                  </p>
                ) : (
                  <p className="mt-0.5 truncate text-[11px] text-muted-foreground/70">{meta.desc}</p>
                )}
              </div>
            </li>
          );
        })}
      </ol>
    </div>
  );
}
