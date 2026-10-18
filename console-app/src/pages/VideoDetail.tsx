import { useMemo, useRef, useState } from "react";
import { Link, useParams } from "react-router";
import { ArrowLeft, RotateCcw } from "lucide-react";
import { toast } from "sonner";
import { IconDrone, IconDroneCam, IconRoute, IconTarget, IconRadar } from "@/components/Icons";
import { trpc } from "@/providers/trpc";
import { TopBar } from "@/components/TopBar";
import { HudPanel } from "@/components/HudPanel";
import { Led } from "@/components/Led";
import { StageTimeline } from "@/components/StageTimeline";
import { LogConsole } from "@/components/LogConsole";
import { VideoHud } from "@/components/VideoHud";
import { PathMap } from "@/components/PathMap";
import { StatsGrid, TargetTable, WaypointList } from "@/components/ResultPanels";
import { fmtBytes, fmtDateTime, fmtMs } from "@/lib/format";
import { cn } from "@/lib/utils";

export default function VideoDetail() {
  const { id } = useParams<{ id: string }>();
  const videoId = Number(id);
  const videoRef = useRef<HTMLVideoElement | null>(null);
  const [activeTaskId, setActiveTaskId] = useState<number | null>(null);
  const [rightTab, setRightTab] = useState<"targets" | "waypoints">("targets");

  const video = trpc.videos.detail.useQuery({ id: videoId }, { enabled: Number.isFinite(videoId) });
  const playback = trpc.videos.playbackUrl.useQuery(
    { id: videoId },
    { enabled: Number.isFinite(videoId), staleTime: 5 * 60 * 1000 },
  );
  const history = trpc.tasks.listByVideo.useQuery(
    { videoId },
    {
      enabled: Number.isFinite(videoId),
      refetchInterval: (q) =>
        q.state.data?.some((t) => t.status === "running" || t.status === "queued") ? 1500 : false,
    },
  );

  const currentTaskId = activeTaskId ?? history.data?.[0]?.id ?? null;
  const task = trpc.tasks.get.useQuery(
    { id: currentTaskId ?? 0 },
    {
      enabled: currentTaskId != null,
      refetchInterval: (q) => {
        const s = q.state.data?.status;
        return s === "running" || s === "queued" ? 800 : false;
      },
    },
  );

  const start = trpc.tasks.start.useMutation({
    onSuccess: (res) => {
      setActiveTaskId(res.id);
      void utils.tasks.listByVideo.invalidate({ videoId });
      void utils.tasks.get.invalidate({ id: res.id });
      if (!res.alreadyRunning) toast.success("识别-规划链已启动");
    },
    onError: (err) => toast.error(err.message),
  });
  const utils = trpc.useUtils();

  const data = task.data ?? null;
  const running = data?.status === "running" || data?.status === "queued";
  const done = data?.status === "done" && data.result;
  const failed = data?.status === "failed";
  const result = done ? data.result : null;

  const cta = useMemo(() => {
    if (running) return { label: "链路运行中", disabled: true, icon: <Loader /> };
    if (done) return { label: "重新运行", disabled: false, icon: <RotateCcw className="h-4 w-4" /> };
    if (failed) return { label: "重试运行", disabled: false, icon: <RotateCcw className="h-4 w-4" /> };
    return { label: "启动识别-规划链", disabled: false, icon: <IconDrone style={{ width: 17, height: 17 }} /> };
  }, [running, done, failed]);

  const seekTo = (t: number) => {
    const v = videoRef.current;
    if (!v) return;
    v.currentTime = t;
    void v.play().catch(() => undefined);
  };

  if (video.isPending) {
    return (
      <div className="min-h-dvh">
        <TopBar />
        <main className="mx-auto max-w-[1600px] px-4 py-5 sm:px-6">
          <div className="h-[420px] animate-pulse bg-panel/50" />
        </main>
      </div>
    );
  }
  if (video.isError) {
    return (
      <div className="min-h-dvh">
        <TopBar />
        <main className="mx-auto max-w-[1600px] px-4 py-16 text-center sm:px-6">
          <p className="font-data text-redsig">视频加载失败：{video.error.message}</p>
          <Link to="/" className="btn-ghost mt-6">返回控制台</Link>
        </main>
      </div>
    );
  }

  return (
    <div className="min-h-dvh">
      <TopBar />
      <main className="mx-auto max-w-[1600px] px-4 py-5 sm:px-6">
        {/* 页头：视频信息 + 启动按钮 */}
        <div className="mb-5 flex flex-wrap items-center gap-x-5 gap-y-3">
          <Link to="/" className="btn-ghost !min-h-[36px] !px-2.5" aria-label="返回控制台">
            <ArrowLeft className="h-4 w-4" />
          </Link>
          <div className="min-w-0">
            <h1 className="flex items-center gap-2 truncate text-[17px] font-semibold tracking-wide">
              <IconDroneCam className="shrink-0 text-hud" style={{ width: 19, height: 19 }} />
              {video.data.name}
            </h1>
            <p className="font-data text-[11px] tracking-wider text-muted-foreground">
              VID-{String(video.data.id).padStart(4, "0")} · {fmtMs(video.data.durationMs)} · {video.data.width}×
              {video.data.height} · {fmtBytes(video.data.sizeBytes)} · {fmtDateTime(video.data.createdAt)}
            </p>
          </div>
          <div className="ml-auto flex items-center gap-3">
            {data && (
              <span className="label-tech hidden items-center gap-2 sm:flex">
                <Led tone={running ? "hud" : failed ? "red" : done ? "idle" : "idle"} />
                {running ? "CHAIN ACTIVE" : done ? "CHAIN DONE" : failed ? "CHAIN FAILED" : ""}
                {data.engine && done && (
                  <span className="text-muted-foreground/70">· {data.engine === "script" ? "外挂脚本引擎" : "内置模拟引擎"}</span>
                )}
              </span>
            )}
            <button
              type="button"
              className={cn("btn-hud", !running && "btn-hud-solid")}
              disabled={cta.disabled || start.isPending}
              onClick={() => start.mutate({ videoId })}
            >
              {cta.icon}
              {start.isPending ? "正在启动…" : cta.label}
            </button>
          </div>
        </div>

        {failed && (
          <div className="mb-4 flex items-start gap-3 border border-redsig/40 bg-redsig/10 px-4 py-3" role="alert">
            <Led tone="red" className="mt-1.5" />
            <div>
              <p className="text-[13px] font-medium text-redsig">链路执行失败</p>
              <p className="mt-0.5 font-data text-[12px] text-redsig/80">{data?.error ?? "未知错误"}</p>
            </div>
          </div>
        )}

        {/* 三栏控制台 */}
        <div className="grid grid-cols-1 gap-4 lg:grid-cols-[300px_minmax(0,1fr)_360px]">
          {/* 中：视频 HUD + 航迹地图（移动端优先展示） */}
          <div className="order-1 flex min-w-0 flex-col gap-4 lg:order-2">
            <HudPanel title="光电画面" eng="EO FEED" icon={<IconDroneCam />} noPad right={
              result && (
                <span className="font-data text-[10px] tracking-[0.14em] text-muted-foreground">
                  {result.frames.length} 帧索引
                </span>
              )
            }>
              {playback.data ? (
                <VideoHud
                  src={playback.data.url}
                  frames={result?.frames ?? null}
                  engine={result?.engine ?? (data?.engine as "builtin" | "script" | null) ?? null}
                  running={!!running}
                  videoRef={videoRef}
                />
              ) : (
                <div className="flex aspect-video items-center justify-center bg-black font-data text-[12px] text-muted-foreground">
                  {playback.isPending ? "加载视频流…" : "视频流不可用"}
                </div>
              )}
            </HudPanel>

            <HudPanel
              title="航迹规划"
              eng="MISSION MAP"
              icon={<IconRoute />}
              noPad
              right={
                result && (
                  <span className="font-data text-[10px] tracking-[0.14em] text-hud">
                    {result.path.distanceM.toFixed(0)}m · ETA {fmtMs(result.path.etaS * 1000)}
                  </span>
                )
              }
            >
              {result ? (
                <PathMap path={result.path} tracks={result.tracks} />
              ) : (
                <div className="flex min-h-[260px] flex-col items-center justify-center gap-2.5 border border-dashed border-hairline text-center">
                  <IconRoute className="animate-standby text-[36px] text-hud/40" style={{ width: 36, height: 36 }} />
                  <p className="text-[12.5px] text-muted-foreground">规划完成后，此处渲染俯视航线、航点与障碍回避区</p>
                  <p className="font-data text-[10px] tracking-[0.18em] text-muted-foreground/50">AWAITING PLAN OUTPUT</p>
                </div>
              )}
            </HudPanel>
          </div>

          {/* 左：阶段 + 日志 */}
          <div className="order-2 flex min-w-0 flex-col gap-4 lg:order-1">
            <HudPanel title="链路阶段" eng="CHAIN STAGES" icon={<IconRadar />}>
              <StageTimeline stages={data?.stages} progress={data?.progress ?? 0} />
            </HudPanel>
            <HudPanel title="链路日志" eng="CONSOLE" noPad className="min-h-[220px] flex-1" bodyClassName="flex flex-col">
              <LogConsole stages={data?.stages} />
            </HudPanel>
          </div>

          {/* 右：结果 */}
          <div className="order-3 flex min-w-0 flex-col gap-4">
            <HudPanel title="任务汇总" eng="SUMMARY" icon={<IconTarget />} noPad>
              {result ? (
                <StatsGrid stats={result.stats} />
              ) : (
                <div className="flex min-h-[120px] items-center justify-center p-4 text-center text-[12.5px] text-muted-foreground">
                  {running ? "链路运行中，结果将在完成后呈现" : "尚未运行。点击右上角「启动识别-规划链」"}
                </div>
              )}
            </HudPanel>

            <HudPanel
              title={rightTab === "targets" ? "目标清单" : "航点序列"}
              eng={rightTab === "targets" ? "TARGETS" : "WAYPOINTS"}
              className="min-h-[280px]"
              right={
                <div className="flex border border-hairline">
                  <TabButton active={rightTab === "targets"} onClick={() => setRightTab("targets")} icon={<IconTarget className="text-[13px]" />} label="目标" />
                  <TabButton active={rightTab === "waypoints"} onClick={() => setRightTab("waypoints")} icon={<IconRoute className="text-[13px]" />} label="航点" />
                </div>
              }
              bodyClassName="thin-scroll max-h-[420px] overflow-y-auto"
            >
              {result ? (
                rightTab === "targets" ? (
                  <TargetTable tracks={result.tracks} onSeek={seekTo} />
                ) : (
                  <WaypointList path={result.path} />
                )
              ) : (
                <div className="flex min-h-[140px] items-center justify-center text-center text-[12.5px] text-muted-foreground">
                  {running ? "识别与规划进行中…" : "无数据"}
                </div>
              )}
            </HudPanel>
          </div>
        </div>
      </main>
    </div>
  );
}

function Loader() {
  return (
    <span className="h-4 w-4 animate-stage-spin rounded-full border-2 border-hud/30 border-t-hud" aria-hidden />
  );
}

function TabButton({ active, onClick, icon, label }: { active: boolean; onClick: () => void; icon: React.ReactNode; label: string }) {
  return (
    <button
      type="button"
      onClick={onClick}
      className={cn(
        "flex min-h-[32px] cursor-pointer items-center gap-1.5 px-2.5 font-mono text-[11px] tracking-wider transition-colors duration-150",
        active ? "bg-hud/15 text-hud" : "text-muted-foreground hover:text-foreground",
      )}
      aria-pressed={active}
    >
      {icon}
      {label}
    </button>
  );
}
