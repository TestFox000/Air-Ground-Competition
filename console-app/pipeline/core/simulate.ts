import type {
  DetectionBox,
  FrameDetections,
  MissionPath,
  ObstacleZone,
  PipelineStats,
  TargetClass,
  TargetTrack,
  Waypoint,
} from "../../contracts/pipeline";
import { clamp, mulberry32, pickWeighted, range } from "./rng";

// 本文件是识别-规划链的权威算法实现（内置引擎），与 pipeline/scripts/ 外挂脚本共用契约。
// 网站后端 api/pipeline/engine.ts 从这里调用；更新规划算法时改这里（或放外挂脚本）。

export interface VideoMeta {
  durationS: number;
  width: number;
  height: number;
}

/** 各类目标的画面尺寸特征（归一化宽度范围 + 像素宽高比） */
const CLASS_SHAPE: Record<TargetClass, { wMin: number; wMax: number; ratio: number; speed: number }> = {
  vehicle: { wMin: 0.055, wMax: 0.095, ratio: 1.7, speed: 0.028 },
  person: { wMin: 0.016, wMax: 0.028, ratio: 0.45, speed: 0.008 },
  rider: { wMin: 0.026, wMax: 0.042, ratio: 0.8, speed: 0.02 },
  obstacle: { wMin: 0.07, wMax: 0.12, ratio: 1.2, speed: 0 },
};

const CLASS_WEIGHTS: [TargetClass, number][] = [
  ["vehicle", 0.38],
  ["person", 0.34],
  ["rider", 0.16],
  ["obstacle", 0.12],
];

/** 抽帧策略：约 2fps 采样，下限 12 帧、上限 96 帧 */
export function planSampling(meta: VideoMeta): { intervalS: number; frameTimes: number[] } {
  const target = clamp(Math.round(meta.durationS * 2), 12, 96);
  const intervalS = meta.durationS / target;
  const frameTimes: number[] = [];
  for (let i = 0; i < target; i++) frameTimes.push(Number((i * intervalS).toFixed(3)));
  return { intervalS, frameTimes };
}

interface SimTarget {
  track: number;
  cls: TargetClass;
  enterS: number;
  exitS: number;
  cx: number;
  cy: number;
  vx: number;
  vy: number;
  wNorm: number;
  hNorm: number;
  confBase: number;
}

/** 目标识别模拟：生成带跨帧轨迹的检测框序列（纯函数，按 seed 确定） */
export function simulateDetections(meta: VideoMeta, seed: number): FrameDetections[] {
  const rand = mulberry32(seed);
  const aspect = meta.width > 0 && meta.height > 0 ? meta.width / meta.height : 16 / 9;
  const { frameTimes } = planSampling(meta);

  const targetCount = 4 + Math.floor(rand() * 5); // 4-8 个目标
  const targets: SimTarget[] = [];
  for (let i = 0; i < targetCount; i++) {
    const cls = pickWeighted(rand, CLASS_WEIGHTS);
    const shape = CLASS_SHAPE[cls];
    const wNorm = range(rand, shape.wMin, shape.wMax);
    const enterS = rand() < 0.55 ? 0 : range(rand, 0, meta.durationS * 0.45);
    const span = range(rand, 0.45, 1) * meta.durationS;
    targets.push({
      track: 100 + i,
      cls,
      enterS,
      exitS: Math.min(meta.durationS, enterS + span),
      cx: range(rand, 0.12, 0.88),
      cy: range(rand, 0.14, 0.86),
      vx: range(rand, -shape.speed, shape.speed),
      vy: range(rand, -shape.speed, shape.speed) * 0.6,
      wNorm,
      hNorm: (wNorm * aspect) / shape.ratio,
      confBase: range(rand, 0.62, 0.92),
    });
  }
  // 保证场面里至少有一个静态障碍（供规划阶段演示避障）
  if (!targets.some((t) => t.cls === "obstacle")) {
    const t = targets[Math.floor(rand() * targets.length)];
    t.cls = "obstacle";
    t.vx = 0;
    t.vy = 0;
    const shape = CLASS_SHAPE.obstacle;
    t.wNorm = range(rand, shape.wMin, shape.wMax);
    t.hNorm = (t.wNorm * aspect) / shape.ratio;
  }

  return frameTimes.map((t) => {
    const boxes: DetectionBox[] = [];
    for (const tgt of targets) {
      if (t < tgt.enterS || t > tgt.exitS) continue;
      const dt = t - tgt.enterS;
      const cx = tgt.cx + tgt.vx * dt;
      const cy = tgt.cy + tgt.vy * dt;
      if (cx < 0.04 || cx > 0.96 || cy < 0.05 || cy > 0.95) continue;
      const conf = clamp(tgt.confBase + range(rand, -0.08, 0.08), 0.42, 0.98);
      if (conf < 0.5) continue; // 置信度闪烁：该帧漏检
      boxes.push({
        track: tgt.track,
        cls: tgt.cls,
        conf: Number(conf.toFixed(3)),
        x: Number(clamp(cx - tgt.wNorm / 2, 0, 1).toFixed(4)),
        y: Number(clamp(cy - tgt.hNorm / 2, 0, 1).toFixed(4)),
        w: Number(tgt.wNorm.toFixed(4)),
        h: Number(tgt.hNorm.toFixed(4)),
      });
    }
    return { t, boxes };
  });
}

/** 由帧序列聚合轨迹清单（脚本引擎只输出 frames 时，也用这个聚合） */
export function aggregateTracks(frames: FrameDetections[]): TargetTrack[] {
  const map = new Map<number, { cls: TargetClass; firstSeen: number; lastSeen: number; frameCount: number; maxConf: number; sx: number; sy: number }>();
  for (const f of frames) {
    for (const b of f.boxes) {
      const cur = map.get(b.track);
      const cx = b.x + b.w / 2;
      const cy = b.y + b.h / 2;
      if (!cur) {
        map.set(b.track, { cls: b.cls, firstSeen: f.t, lastSeen: f.t, frameCount: 1, maxConf: b.conf, sx: cx, sy: cy });
      } else {
        cur.lastSeen = f.t;
        cur.frameCount += 1;
        cur.maxConf = Math.max(cur.maxConf, b.conf);
        cur.sx += cx;
        cur.sy += cy;
      }
    }
  }
  return [...map.entries()]
    .map(([track, v]) => ({
      track,
      cls: v.cls,
      firstSeen: v.firstSeen,
      lastSeen: v.lastSeen,
      frameCount: v.frameCount,
      maxConf: Number(v.maxConf.toFixed(3)),
      gx: Number((v.sx / v.frameCount).toFixed(4)),
      gy: Number((v.sy / v.frameCount).toFixed(4)),
    }))
    .sort((a, b) => a.firstSeen - b.firstSeen || a.track - b.track);
}

/** 航迹规划：目标聚类 → 就近排序 → 障碍绕行 → 计算里程与耗时 */
export function planMission(meta: VideoMeta, tracks: TargetTrack[], seed: number): MissionPath {
  const rand = mulberry32(seed ^ 0x9e3779b9);
  const aspect = meta.width > 0 && meta.height > 0 ? meta.width / meta.height : 16 / 9;
  const groundWidthM = 140;
  const groundHeightM = Math.round(groundWidthM / aspect);
  const toGround = (gx: number, gy: number) => ({ x: gx * groundWidthM, y: (1 - gy) * groundHeightM });

  // 障碍区：障碍类目标 → 圆形回避区
  const obstacles: ObstacleZone[] = tracks
    .filter((t) => t.cls === "obstacle")
    .map((t) => {
      const p = toGround(t.gx, t.gy);
      return { x: Number(p.x.toFixed(1)), y: Number(p.y.toFixed(1)), r: Math.round(range(rand, 12, 20)), label: `障碍 #${t.track}` };
    });

  // 目标聚类（非障碍）：35m 贪心合并
  const points = tracks
    .filter((t) => t.cls !== "obstacle")
    .map((t) => {
      const p = toGround(t.gx, t.gy);
      const classPriority = t.cls === "person" ? 3 : t.cls === "rider" ? 2 : 1;
      return { ...t, x: p.x, y: p.y, score: classPriority * t.maxConf * Math.min(t.frameCount, 30) };
    })
    .sort((a, b) => b.score - a.score);

  interface Cluster { x: number; y: number; score: number; members: number }
  const clusters: Cluster[] = [];
  for (const p of points) {
    const hit = clusters.find((c) => Math.hypot(c.x - p.x, c.y - p.y) < 35);
    if (hit) {
      hit.x = (hit.x * hit.members + p.x) / (hit.members + 1);
      hit.y = (hit.y * hit.members + p.y) / (hit.members + 1);
      hit.score += p.score;
      hit.members += 1;
    } else {
      clusters.push({ x: p.x, y: p.y, score: p.score, members: 1 });
    }
  }
  const ranked = clusters.sort((a, b) => b.score - a.score).slice(0, 4);

  // 就近排序（从 home 出发的最近邻）
  const home = { x: 10, y: 10 };
  const ordered: Cluster[] = [];
  const rest = [...ranked];
  let cur = home;
  while (rest.length) {
    let best = 0;
    let bestD = Infinity;
    rest.forEach((c, i) => {
      const d = Math.hypot(c.x - cur.x, c.y - cur.y);
      if (d < bestD) {
        bestD = d;
        best = i;
      }
    });
    const [next] = rest.splice(best, 1);
    ordered.push(next);
    cur = next;
  }

  // 组装航点
  const waypoints: Waypoint[] = [];
  let seq = 1;
  const push = (wp: Omit<Waypoint, "seq">) => waypoints.push({ seq: seq++, ...wp });
  push({ x: home.x, y: home.y, alt: 0, speed: 0, action: "takeoff", note: "起飞，爬升至 60m 巡航高度" });

  const detour = (from: { x: number; y: number }, to: { x: number; y: number }) => {
    // 线段与障碍区相交 → 在障碍切向插入绕行点
    for (const ob of obstacles) {
      const seg = closestApproach(from, to, ob);
      if (seg < ob.r + 8) {
        const mx = (from.x + to.x) / 2;
        const my = (from.y + to.y) / 2;
        const dx = to.x - from.x;
        const dy = to.y - from.y;
        const len = Math.hypot(dx, dy) || 1;
        const px = -dy / len;
        const py = dx / len;
        const side = (ob.x - mx) * px + (ob.y - my) * py > 0 ? 1 : -1;
        const offset = ob.r + 14;
        push({
          x: Number((ob.x + px * -side * offset).toFixed(1)),
          y: Number((ob.y + py * -side * offset).toFixed(1)),
          alt: 60,
          speed: 8,
          action: "transit",
          note: `规避${ob.label}，横向偏移 ${offset}m`,
        });
      }
    }
  };

  let prev = home;
  ordered.forEach((c, i) => {
    detour(prev, c);
    push({
      x: Number(c.x.toFixed(1)),
      y: Number(c.y.toFixed(1)),
      alt: 30,
      speed: i === 0 ? 8 : 6,
      action: "inspect",
      note: `抵近目标簇 T${i + 1}（${c.members} 个目标），降至 30m 详查`,
    });
    push({
      x: Number(c.x.toFixed(1)),
      y: Number(c.y.toFixed(1)),
      alt: 30,
      speed: 0,
      action: "loiter",
      note: `盘旋监视 8s，保持光电载荷对准`,
    });
    prev = c;
  });

  detour(prev, home);
  push({ x: home.x, y: home.y, alt: 60, speed: 8, action: "return", note: "返航至起飞点上空" });
  push({ x: home.x, y: home.y, alt: 0, speed: 0, action: "land", note: "降落，任务结束" });

  // 里程与耗时
  let distanceM = 0;
  let etaS = 0;
  for (let i = 1; i < waypoints.length; i++) {
    const a = waypoints[i - 1];
    const b = waypoints[i];
    const horiz = Math.hypot(b.x - a.x, b.y - a.y);
    const vert = Math.abs(b.alt - a.alt);
    distanceM += horiz;
    if (b.action === "loiter") {
      etaS += 8;
    } else if (b.action === "takeoff") {
      etaS += 12;
    } else if (b.action === "land") {
      etaS += 10;
    } else {
      etaS += horiz / Math.max(b.speed, 1) + vert / 3;
    }
  }

  return {
    home,
    waypoints,
    obstacles,
    groundWidthM,
    groundHeightM,
    distanceM: Number(distanceM.toFixed(1)),
    etaS: Math.round(etaS),
  };
}

function closestApproach(a: { x: number; y: number }, b: { x: number; y: number }, p: { x: number; y: number }): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0) return Math.hypot(p.x - a.x, p.y - a.y);
  const t = clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq, 0, 1);
  return Math.hypot(p.x - (a.x + dx * t), p.y - (a.y + dy * t));
}

export function buildStats(
  meta: VideoMeta,
  frames: FrameDetections[],
  tracks: TargetTrack[],
  path: MissionPath,
): PipelineStats {
  const byClass: Record<TargetClass, number> = { vehicle: 0, person: 0, rider: 0, obstacle: 0 };
  for (const t of tracks) byClass[t.cls] += 1;
  return {
    durationS: Number(meta.durationS.toFixed(2)),
    width: meta.width,
    height: meta.height,
    sampleIntervalS: Number(planSampling(meta).intervalS.toFixed(3)),
    framesProcessed: frames.length,
    totalDetections: frames.reduce((s, f) => s + f.boxes.length, 0),
    uniqueTargets: tracks.length,
    byClass,
    pathDistanceM: path.distanceM,
    pathEtaS: path.etaS,
    waypointCount: path.waypoints.length,
    obstacleCount: path.obstacles.length,
  };
}
