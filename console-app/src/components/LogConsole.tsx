import { useEffect, useRef } from "react";
import type { StageState } from "@contracts/pipeline";
import { STAGE_META } from "@contracts/pipeline";
import { fmtClock } from "@/lib/format";
import { cn } from "@/lib/utils";

interface FlatLog {
  key: string;
  ts: string;
  stageLabel: string;
  msg: string;
}

/** 链路日志终端：合并各阶段日志，按时间排序，自动滚到底部 */
export function LogConsole({ stages }: { stages: StageState[] | null | undefined }) {
  const bodyRef = useRef<HTMLDivElement>(null);

  const flat: FlatLog[] = [];
  for (const s of stages ?? []) {
    for (const [i, l] of s.logs.entries()) {
      flat.push({ key: `${s.key}-${i}`, ts: l.ts, stageLabel: STAGE_META[s.key].eng, msg: l.msg });
    }
  }

  useEffect(() => {
    const el = bodyRef.current;
    if (el) el.scrollTop = el.scrollHeight;
  }, [flat.length]);

  return (
    <div className="flex h-full min-h-[180px] flex-col bg-background/60">
      <div ref={bodyRef} className="thin-scroll min-h-0 flex-1 overflow-y-auto px-3 py-2 font-data text-[11.5px] leading-[1.75]">
        {flat.length === 0 ? (
          <p className="text-muted-foreground/60">
            <span className="text-hud">&gt;</span> 链路待机，等待「启动识别-规划链」指令
            <span className="animate-standby ml-1 inline-block h-3 w-[7px] translate-y-[2px] bg-hud/70" />
          </p>
        ) : (
          flat.map((l) => (
            <p key={l.key} className="break-words">
              <span className="text-muted-foreground/60">[{fmtClock(l.ts)}]</span>{" "}
              <span className="text-muted-foreground">{l.stageLabel}</span>{" "}
              <span
                className={cn(
                  "text-foreground/85",
                  l.msg.startsWith("⚠") && "text-ambersig",
                  l.msg.startsWith("✗") && "text-redsig",
                  l.msg.endsWith("✓") && "text-hud",
                )}
              >
                {l.msg}
              </span>
            </p>
          ))
        )}
      </div>
    </div>
  );
}
