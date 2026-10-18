import { useCallback, useEffect, useRef, useState, type RefObject } from "react";
import type { EngineKind, FrameDetections } from "@contracts/pipeline";
import { TARGET_CLASS_META } from "@contracts/pipeline";
import { IconDroneSignal } from "./Icons";
import { fmtDuration } from "@/lib/format";

interface Props {
  src: string;
  frames: FrameDetections[] | null;
  engine: EngineKind | null;
  running: boolean;
  videoRef: RefObject<HTMLVideoElement | null>;
  onTimeChange?: (t: number) => void;
}

interface InterpBox {
  track: number;
  cls: keyof typeof TARGET_CLASS_META;
  conf: number;
  x: number;
  y: number;
  w: number;
  h: number;
}

/** 取 t 时刻的检测框：相邻采样帧按轨迹号线性插值，保证 HUD 平滑 */
function boxesAt(frames: FrameDetections[], t: number): InterpBox[] {
  if (frames.length === 0) return [];
  if (t <= frames[0].t) return frames[0].boxes.map((b) => ({ ...b }));
  const last = frames[frames.length - 1];
  if (t >= last.t) return last.boxes.map((b) => ({ ...b }));
  let hi = frames.findIndex((f) => f.t >= t);
  if (hi <= 0) hi = 1;
  const f0 = frames[hi - 1];
  const f1 = frames[hi];
  const span = f1.t - f0.t || 1;
  const k = Math.min(1, Math.max(0, (t - f0.t) / span));
  const byTrack = new Map(f1.boxes.map((b) => [b.track, b]));
  const out: InterpBox[] = [];
  for (const b0 of f0.boxes) {
    const b1 = byTrack.get(b0.track);
    if (b1) {
      byTrack.delete(b0.track);
      out.push({
        track: b0.track,
        cls: b0.cls,
        conf: b0.conf + (b1.conf - b0.conf) * k,
        x: b0.x + (b1.x - b0.x) * k,
        y: b0.y + (b1.y - b0.y) * k,
        w: b0.w + (b1.w - b0.w) * k,
        h: b0.h + (b1.h - b0.h) * k,
      });
    } else {
      out.push({ ...b0 });
    }
  }
  for (const b of byTrack.values()) out.push({ ...b });
  return out;
}

export function VideoHud({ src, frames, engine, running, videoRef, onTimeChange }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);
  const [now, setNow] = useState({ t: 0, d: 0, tgts: 0 });
  const framesRef = useRef(frames);
  framesRef.current = frames;

  const draw = useCallback(() => {
    const video = videoRef.current;
    const canvas = canvasRef.current;
    if (!video || !canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const dpr = window.devicePixelRatio || 1;
    const W = canvas.width / dpr;
    const H = canvas.height / dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    const fs = framesRef.current;
    if (!fs) return;
    const boxes = boxesAt(fs, video.currentTime);
    ctx.font = '600 10px "JetBrains Mono", monospace';
    for (const b of boxes) {
      const meta = TARGET_CLASS_META[b.cls];
      const bx = b.x * W;
      const by = b.y * H;
      const bw = b.w * W;
      const bh = b.h * H;
      const l = Math.max(6, Math.min(16, Math.min(bw, bh) * 0.32));
      ctx.strokeStyle = meta.color;
      ctx.lineWidth = 1.6;
      ctx.shadowColor = meta.color;
      ctx.shadowBlur = 6;
      // 八角括号
      ctx.beginPath();
      // 左上
      ctx.moveTo(bx, by + l); ctx.lineTo(bx, by); ctx.lineTo(bx + l, by);
      // 右上
      ctx.moveTo(bx + bw - l, by); ctx.lineTo(bx + bw, by); ctx.lineTo(bx + bw, by + l);
      // 右下
      ctx.moveTo(bx + bw, by + bh - l); ctx.lineTo(bx + bw, by + bh); ctx.lineTo(bx + bw - l, by + bh);
      // 左下
      ctx.moveTo(bx + l, by + bh); ctx.lineTo(bx, by + bh); ctx.lineTo(bx, by + bh - l);
      ctx.stroke();
      // 中心点
      ctx.fillStyle = meta.color;
      ctx.fillRect(bx + bw / 2 - 1, by + bh / 2 - 1, 2, 2);
      // 标签
      const label = `${meta.label}#${b.track} ${(b.conf * 100).toFixed(0)}%`;
      const tw = ctx.measureText(label).width + 10;
      const th = 15;
      const ly = by - th - 3 >= 0 ? by - th - 3 : by + 3;
      ctx.shadowBlur = 0;
      ctx.globalAlpha = 0.92;
      ctx.fillStyle = meta.color;
      ctx.fillRect(bx, ly, tw, th);
      ctx.globalAlpha = 1;
      ctx.fillStyle = "#05130c";
      ctx.fillText(label, bx + 5, ly + 11);
    }
  }, [videoRef]);

  // 尺寸跟随
  useEffect(() => {
    const wrap = wrapRef.current;
    const canvas = canvasRef.current;
    if (!wrap || !canvas) return;
    const resize = () => {
      const dpr = window.devicePixelRatio || 1;
      canvas.width = wrap.clientWidth * dpr;
      canvas.height = wrap.clientHeight * dpr;
      canvas.style.width = `${wrap.clientWidth}px`;
      canvas.style.height = `${wrap.clientHeight}px`;
      draw();
    };
    const ro = new ResizeObserver(resize);
    ro.observe(wrap);
    resize();
    return () => ro.disconnect();
  }, [draw]);

  // frames 变化（含清空）时立即重绘，避免旧检测框残留
  useEffect(() => {
    draw();
  }, [frames, draw]);

  // 播放驱动：rAF 连续绘制；暂停/seek 时按事件绘制
  useEffect(() => {
    const video = videoRef.current;
    if (!video) return;
    let raf = 0;
    const loop = () => {
      draw();
      raf = requestAnimationFrame(loop);
    };
    const onPlay = () => {
      cancelAnimationFrame(raf);
      raf = requestAnimationFrame(loop);
    };
    const onPause = () => {
      cancelAnimationFrame(raf);
      draw();
    };
    const onTime = () => {
      draw();
      setNow({
        t: video.currentTime,
        d: video.duration || 0,
        tgts: framesRef.current ? boxesAt(framesRef.current, video.currentTime).length : 0,
      });
      onTimeChange?.(video.currentTime);
    };
    video.addEventListener("play", onPlay);
    video.addEventListener("pause", onPause);
    video.addEventListener("timeupdate", onTime);
    video.addEventListener("seeked", onTime);
    video.addEventListener("loadedmetadata", onTime);
    return () => {
      cancelAnimationFrame(raf);
      video.removeEventListener("play", onPlay);
      video.removeEventListener("pause", onPause);
      video.removeEventListener("timeupdate", onTime);
      video.removeEventListener("seeked", onTime);
      video.removeEventListener("loadedmetadata", onTime);
    };
  }, [draw, videoRef, onTimeChange]);

  return (
    <div ref={wrapRef} className="scanlines relative aspect-video w-full overflow-hidden bg-black">
      <video ref={videoRef} src={src} controls playsInline preload="metadata" className="absolute inset-0 h-full w-full" />
      <canvas ref={canvasRef} className="pointer-events-none absolute inset-0 z-10" aria-hidden />

      {/* HUD 角框 */}
      <div className="pointer-events-none absolute inset-0 z-20" aria-hidden>
        <span className="absolute left-2 top-2 h-4 w-4 border-l-2 border-t-2 border-hud/70" />
        <span className="absolute right-2 top-2 h-4 w-4 border-r-2 border-t-2 border-hud/70" />
        <span className="absolute bottom-2 left-2 h-4 w-4 border-b-2 border-l-2 border-hud/70" />
        <span className="absolute bottom-2 right-2 h-4 w-4 border-b-2 border-r-2 border-hud/70" />
        {/* 中心十字 */}
        <svg className="absolute left-1/2 top-1/2 h-10 w-10 -translate-x-1/2 -translate-y-1/2 opacity-40" viewBox="0 0 40 40">
          <line x1="20" y1="6" x2="20" y2="15" stroke="hsl(var(--hud))" strokeWidth="1" />
          <line x1="20" y1="25" x2="20" y2="34" stroke="hsl(var(--hud))" strokeWidth="1" />
          <line x1="6" y1="20" x2="15" y2="20" stroke="hsl(var(--hud))" strokeWidth="1" />
          <line x1="25" y1="20" x2="34" y2="20" stroke="hsl(var(--hud))" strokeWidth="1" />
          <circle cx="20" cy="20" r="5.5" fill="none" stroke="hsl(var(--hud))" strokeWidth="1" />
        </svg>
      </div>

      {/* 遥测角标 */}
      <div className="pointer-events-none absolute left-3 top-3 z-30 flex items-center gap-2 font-data text-[11px] tracking-wider">
        {running ? (
          <>
            <span className="h-2 w-2 animate-pulse-dot-red rounded-full bg-red-500" />
            <span className="text-red-400 hud-glow-soft">REC</span>
          </>
        ) : (
          <span className="text-hud hud-glow-soft">{frames ? "REPLAY" : "STANDBY"}</span>
        )}
        <span className="text-foreground/80">{fmtDuration(now.t)} / {fmtDuration(now.d)}</span>
      </div>
      <div className="pointer-events-none absolute right-3 top-3 z-30 font-data text-[11px] tracking-wider text-foreground/80">
        TGT <span className="text-hud hud-glow-soft">{frames ? now.tgts : "—"}</span>
      </div>
      <div
        className="pointer-events-none absolute bottom-14 left-3 z-30 hidden font-data text-[10px] tracking-[0.14em] text-foreground/70 sm:block"
        style={{ textShadow: "0 1px 4px rgba(0,0,0,0.9)" }}
      >
        {engine === "script" ? "ENGINE: 外挂脚本" : engine === "builtin" ? "ENGINE: 内置模拟" : "ENGINE: —"}
      </div>
      <div
        className="pointer-events-none absolute bottom-14 right-3 z-30 hidden font-data text-[10px] tracking-[0.14em] text-foreground/70 sm:block"
        style={{ textShadow: "0 1px 4px rgba(0,0,0,0.9)" }}
      >
        OFFLINE · 本地链路
      </div>

      {/* 待机水印 */}
      {!frames && !running && (
        <div className="pointer-events-none absolute inset-0 z-20 flex flex-col items-center justify-center gap-3">
          <IconDroneSignal className="animate-standby text-hud/60 hud-glow" style={{ width: 54, height: 54 }} />
          <p className="animate-standby font-data text-[13px] tracking-[0.3em] text-hud/70 hud-glow">
            STANDBY · 等待链路启动
          </p>
        </div>
      )}
    </div>
  );
}
