import type { MissionPath, PipelineStats, TargetTrack } from "@contracts/pipeline";
import { TARGET_CLASS_META, WAYPOINT_ACTION_META } from "@contracts/pipeline";
import { fmtDuration } from "@/lib/format";
import { cn } from "@/lib/utils";

/** 关键指标格 */
export function StatsGrid({ stats }: { stats: PipelineStats }) {
  const aspect = stats.width > 0 && stats.height > 0 ? stats.width / stats.height : 16 / 9;
  const coverW = 140;
  const coverH = Math.round(coverW / aspect);
  const items: { label: string; value: string; unit?: string; tone?: "hud" | "amber" }[] = [
    { label: "稳定目标", value: String(stats.uniqueTargets), unit: "个", tone: "hud" },
    { label: "检测框总数", value: String(stats.totalDetections), unit: "框" },
    { label: "处理帧数", value: String(stats.framesProcessed), unit: `帧 @${stats.sampleIntervalS}s` },
    { label: "航线里程", value: stats.pathDistanceM.toFixed(0), unit: "m", tone: "hud" },
    { label: "预计耗时", value: fmtDuration(stats.pathEtaS), unit: "mm:ss" },
    { label: "航点数量", value: String(stats.waypointCount), unit: "个" },
    { label: "障碍回避区", value: String(stats.obstacleCount), unit: "处", tone: stats.obstacleCount > 0 ? "amber" : undefined },
    { label: "覆盖范围", value: `${coverW}×${coverH}`, unit: "m" },
  ];
  return (
    <div className="grid grid-cols-2 gap-px bg-hairline sm:grid-cols-4 lg:grid-cols-2 xl:grid-cols-4">
      {items.map((it) => (
        <div key={it.label} className="bg-card px-3 py-2.5">
          <p className="label-tech">{it.label}</p>
          <p
            className={cn(
              "font-data mt-1 text-[19px] font-semibold leading-none",
              it.tone === "hud" && "text-hud hud-glow-soft",
              it.tone === "amber" && "text-ambersig",
              !it.tone && "text-foreground",
            )}
          >
            {it.value}
            <span className="ml-1 text-[10px] font-normal text-muted-foreground">{it.unit}</span>
          </p>
        </div>
      ))}
    </div>
  );
}

/** 目标轨迹清单：点击行可将视频跳转到该目标首次出现时刻 */
export function TargetTable({ tracks, onSeek }: { tracks: TargetTrack[]; onSeek?: (t: number) => void }) {
  if (tracks.length === 0) {
    return <p className="px-1 py-6 text-center text-[12px] text-muted-foreground">未检出目标</p>;
  }
  return (
    <table className="w-full text-left">
      <thead>
        <tr className="border-b border-hairline">
          {["轨迹", "类别", "首见", "持续", "帧数", "峰值"].map((h) => (
            <th key={h} className="label-tech px-2 py-2 font-medium first:pl-0 last:pr-0">
              {h}
            </th>
          ))}
        </tr>
      </thead>
      <tbody>
        {tracks.map((t) => {
          const meta = TARGET_CLASS_META[t.cls];
          return (
            <tr
              key={t.track}
              onClick={() => onSeek?.(t.firstSeen)}
              className={cn(
                "border-b border-hairline/60 transition-colors duration-150 last:border-0",
                onSeek && "cursor-pointer hover:bg-panel/70",
              )}
              title={onSeek ? "点击跳转到该目标首次出现时刻" : undefined}
            >
              <td className="px-2 py-2 pl-0 font-data text-[12px] text-foreground">#{t.track}</td>
              <td className="px-2 py-2">
                <span
                  className="inline-flex items-center gap-1.5 border px-1.5 py-0.5 font-data text-[11px]"
                  style={{ color: meta.color, borderColor: `${meta.color}55`, background: `${meta.color}14` }}
                >
                  <span className="h-1.5 w-1.5" style={{ background: meta.color }} />
                  {meta.label}
                </span>
              </td>
              <td className="px-2 py-2 font-data text-[12px] text-muted-foreground">{fmtDuration(t.firstSeen)}</td>
              <td className="px-2 py-2 font-data text-[12px] text-muted-foreground">
                {(t.lastSeen - t.firstSeen).toFixed(1)}s
              </td>
              <td className="px-2 py-2 font-data text-[12px] text-muted-foreground">{t.frameCount}</td>
              <td className="px-2 py-2 pr-0 font-data text-[12px] text-hud">{(t.maxConf * 100).toFixed(0)}%</td>
            </tr>
          );
        })}
      </tbody>
    </table>
  );
}

/** 航点序列 */
export function WaypointList({ path }: { path: MissionPath }) {
  return (
    <ol className="space-y-0">
      {path.waypoints.map((w, i) => {
        const last = i === path.waypoints.length - 1;
        const strong = w.action === "inspect" || w.action === "loiter";
        return (
          <li key={w.seq} className="relative flex gap-3 pb-3 last:pb-0">
            {!last && <span className="absolute left-[10px] top-6 h-[calc(100%-20px)] w-px bg-hairline" />}
            <span
              className={cn(
                "mt-0.5 flex h-[21px] w-[21px] shrink-0 items-center justify-center border font-data text-[10px] font-semibold",
                strong ? "border-hud bg-hud text-background" : "border-hud/50 bg-panel text-hud",
              )}
            >
              {w.seq}
            </span>
            <div className="min-w-0 flex-1">
              <div className="flex flex-wrap items-baseline justify-between gap-x-3">
                <span className="text-[12.5px] font-medium">{WAYPOINT_ACTION_META[w.action].label}</span>
                <span className="font-data text-[10.5px] tracking-wider text-muted-foreground">
                  ({w.x.toFixed(0)}, {w.y.toFixed(0)}) · H{w.alt}m{w.speed > 0 ? ` · ${w.speed}m/s` : ""}
                </span>
              </div>
              <p className="mt-0.5 text-[11.5px] leading-relaxed text-muted-foreground">{w.note}</p>
            </div>
          </li>
        );
      })}
    </ol>
  );
}
