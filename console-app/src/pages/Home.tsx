import { Link } from "react-router";
import { ChevronRight } from "lucide-react";
import { trpc } from "@/providers/trpc";
import { TopBar } from "@/components/TopBar";
import { HudPanel } from "@/components/HudPanel";
import { UploadZone } from "@/components/UploadZone";
import { Led } from "@/components/Led";
import { IconDroneSignal, IconDroneCam, IconRadar, IconRoute } from "@/components/Icons";
import { STAGE_ICONS } from "@/components/stage-icons";
import { STAGE_KEYS, STAGE_META, type TaskStatus } from "@contracts/pipeline";
import { fmtBytes, fmtDateTime, fmtMs } from "@/lib/format";

function statusChip(status: TaskStatus | undefined, progress: number) {
  if (!status) return { text: "待机 STANDBY", cls: "border-hairline text-muted-foreground", led: "idle" as const };
  switch (status) {
    case "queued":
      return { text: "排队中 QUEUED", cls: "border-ambersig/50 text-ambersig", led: "amber" as const };
    case "running":
      return { text: `运行中 ${progress}%`, cls: "border-hud/60 text-hud", led: "hud" as const };
    case "done":
      return { text: "已完成 DONE", cls: "border-mintsig/50 text-mintsig", led: "idle" as const };
    case "failed":
      return { text: "失败 FAILED", cls: "border-redsig/50 text-redsig", led: "red" as const };
  }
}

/** 顶栏下的状态滚动条（控制台氛围） */
function StatusTicker() {
  const items = [
    "SYS.CHECK 全系统自检通过",
    "ENGINE 内置模拟引擎就绪",
    "LINK 本地链路 · 离线可用",
    "DETECT 目标识别 · 车辆/人员/骑手/障碍",
    "PLAN 航迹规划 · 聚类侦察点 · 障碍规避",
    "STORAGE 素材库在线",
    "SCRIPT-HOOK pipeline/scripts/ 接口待机",
  ];
  const line = items.map((t) => `◆ ${t}`).join("   ");
  return (
    <div className="ticker-mask relative mb-5 overflow-hidden border-y border-hairline bg-panel/30" aria-hidden>
      <div className="animate-ticker flex w-max whitespace-nowrap py-1.5 font-data text-[10.5px] tracking-[0.18em] text-muted-foreground/80">
        <span className="pr-12">{line}</span>
        <span className="pr-12">{line}</span>
      </div>
      <div className="pointer-events-none absolute inset-y-0 left-0 w-16 bg-gradient-to-r from-background to-transparent" />
      <div className="pointer-events-none absolute inset-y-0 right-0 w-16 bg-gradient-to-l from-background to-transparent" />
    </div>
  );
}

export default function Home() {
  const list = trpc.videos.list.useQuery(undefined, {
    refetchInterval: (q) =>
      q.state.data?.some((v) => v.latestTask && (v.latestTask.status === "running" || v.latestTask.status === "queued"))
        ? 1200
        : false,
  });

  return (
    <div className="min-h-dvh">
      <TopBar />
      <main className="mx-auto max-w-[1600px] px-4 py-5 sm:px-6">
        <StatusTicker />
        <div className="grid grid-cols-1 gap-5 lg:grid-cols-[minmax(320px,380px)_1fr]">
          {/* 左栏：上传 + 链路说明 */}
          <div className="flex flex-col gap-5">
            <HudPanel title="视频注入" eng="INGEST" icon={<IconDroneCam />}>
              <UploadZone />
            </HudPanel>

            <HudPanel title="识别-规划链" eng="DETECT-PLAN CHAIN" icon={<IconRoute />} bodyClassName="px-4 py-3">
              <ol className="space-y-0">
                {STAGE_KEYS.map((key, i) => {
                  const meta = STAGE_META[key];
                  const Glyph = STAGE_ICONS[key];
                  const last = i === STAGE_KEYS.length - 1;
                  return (
                    <li key={key} className="relative flex gap-3 pb-4 last:pb-0">
                      {!last && <span className="absolute left-[13px] top-8 h-[calc(100%-26px)] w-px bg-hairline" />}
                      <span className="mt-0.5 flex h-7 w-7 shrink-0 items-center justify-center border border-hud/40 bg-hud/10 text-[14px] text-hud">
                        <Glyph />
                      </span>
                      <div className="min-w-0">
                        <div className="flex items-baseline gap-2">
                          <span className="text-[13px] font-medium">{meta.label}</span>
                          <span className="label-tech">{String(i + 1).padStart(2, "0")} · {meta.eng}</span>
                        </div>
                        <p className="mt-0.5 text-[12px] leading-relaxed text-muted-foreground">{meta.desc}</p>
                      </div>
                    </li>
                  );
                })}
              </ol>
              <div className="mt-4 border-t border-hairline pt-3 text-[12px] leading-relaxed text-muted-foreground">
                链路默认由<span className="text-hud">内置模拟引擎</span>执行，全程本地离线；
                将真实脚本放入 <Link to="/docs" className="font-data text-hud underline decoration-hud/40 underline-offset-4 hover:decoration-hud">pipeline/scripts/</Link> 即可无缝接管对应阶段。
              </div>
            </HudPanel>
          </div>

          {/* 右栏：视频库 */}
          <HudPanel
            title="视频库"
            eng="FOOTAGE LIBRARY"
            icon={<IconRadar />}
            className="min-h-[420px]"
            right={
              <span className="font-data text-[11px] tracking-wider text-muted-foreground">
                {list.data ? `${list.data.length} 条记录` : "—"}
              </span>
            }
            noPad
          >
            {list.isPending ? (
              <div className="space-y-px p-px">
                {[0, 1, 2].map((i) => (
                  <div key={i} className="h-[68px] animate-pulse bg-panel/60" />
                ))}
              </div>
            ) : list.isError ? (
              <div className="flex h-full min-h-[300px] flex-col items-center justify-center gap-2 p-8 text-center">
                <p className="font-data text-[13px] text-redsig">视频库读取失败</p>
                <p className="text-[12px] text-muted-foreground">{list.error.message}</p>
              </div>
            ) : list.data.length === 0 ? (
              <div className="flex h-full min-h-[300px] flex-col items-center justify-center gap-3 p-8 text-center">
                <IconDroneSignal className="animate-standby text-[44px] text-hud/50" />
                <p className="text-[14px] text-muted-foreground">还没有视频。上传一段航拍素材，启动第一条识别-规划链。</p>
                <p className="font-data text-[11px] tracking-[0.14em] text-muted-foreground/60">NO FOOTAGE · UPLOAD TO BEGIN</p>
              </div>
            ) : (
              <ul className="divide-y divide-hairline">
                {list.data.map((v) => {
                  const chip = statusChip(v.latestTask?.status, v.latestTask?.progress ?? 0);
                  return (
                    <li key={v.id}>
                      <Link
                        to={`/videos/${v.id}`}
                        className="group flex min-h-[68px] flex-wrap items-center gap-x-4 gap-y-1 px-4 py-3 transition-colors duration-150 hover:bg-panel/70 sm:flex-nowrap"
                      >
                        <span className="hidden h-9 w-9 shrink-0 items-center justify-center border border-hairline bg-panel/60 text-[17px] text-muted-foreground transition-colors duration-150 group-hover:border-hud/40 group-hover:text-hud sm:flex">
                          <IconDroneCam />
                        </span>
                        <div className="min-w-0 flex-1 basis-full sm:basis-auto">
                          <p className="truncate text-[14px] font-medium text-foreground group-hover:text-hud">{v.name}</p>
                          <p className="mt-0.5 font-data text-[11px] tracking-wider text-muted-foreground">
                            {fmtMs(v.durationMs)} · {v.width}×{v.height} · {fmtBytes(v.sizeBytes)}
                          </p>
                        </div>
                        <span className="font-data hidden text-[11px] text-muted-foreground/70 md:inline">
                          {fmtDateTime(v.createdAt)}
                        </span>
                        <span
                          className={`inline-flex min-h-[28px] items-center gap-2 border px-2.5 font-data text-[11px] tracking-wider ${chip.cls}`}
                        >
                          <Led tone={chip.led} />
                          {chip.text}
                        </span>
                        <ChevronRight className="h-4 w-4 shrink-0 text-muted-foreground/50 transition-transform duration-150 group-hover:translate-x-0.5 group-hover:text-hud" />
                      </Link>
                    </li>
                  );
                })}
              </ul>
            )}
          </HudPanel>
        </div>
      </main>
    </div>
  );
}
