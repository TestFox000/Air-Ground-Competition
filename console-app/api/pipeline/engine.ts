import { eq } from "drizzle-orm";
import { getDb } from "../queries/connection";
import { pipelineTasks, videos } from "../../db/schema";
import { storage } from "../lib/storage";
import type {
  EngineKind,
  FrameDetections,
  MissionPath,
  PipelineResult,
  StageKey,
  StageState,
} from "../../contracts/pipeline";
import { STAGE_KEYS, STAGE_META } from "../../contracts/pipeline";
import { aggregateTracks, buildStats, planMission, planSampling, simulateDetections, type VideoMeta } from "../../pipeline/core/simulate";
import { findScript, prepareTaskDir, runDetectScript, runPlanScript, type ScriptIo } from "./script-hook";
import fsp from "node:fs/promises";

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

/** 各阶段的进度区间（整体百分比） */
const STAGE_SPAN: Record<StageKey, [number, number]> = {
  probe: [2, 10],
  extract: [10, 22],
  detect: [22, 68],
  plan: [68, 92],
  finalize: [92, 100],
};

/**
 * 识别-规划链引擎：顺序执行 解析 → 抽帧 → 识别 → 规划 → 汇总。
 * 进度与日志实时落库，前端轮询任务行即可渲染阶段时间线。
 * detect / plan 阶段优先尝试 pipeline/scripts/ 下的外挂脚本，失败自动回退内置模拟。
 */
export async function runPipeline(taskId: number): Promise<void> {
  const db = getDb();
  const task = await db.query.pipelineTasks.findFirst({ where: eq(pipelineTasks.id, taskId) });
  if (!task) return;
  const video = await db.query.videos.findFirst({ where: eq(videos.id, task.videoId) });
  if (!video) throw new Error(`视频记录不存在: ${task.videoId}`);

  const meta: VideoMeta = {
    durationS: Math.max(video.durationMs / 1000, 0.5),
    width: video.width || 1280,
    height: video.height || 720,
  };
  const seed = video.id * 7919 + 17;

  const stages: StageState[] = STAGE_KEYS.map((key) => ({ key, status: "pending", logs: [] }));
  let engine: EngineKind = "builtin";
  let lastFlush = 0;

  const flush = async (force = false, patch: Partial<typeof pipelineTasks.$inferInsert> = {}) => {
    const now = Date.now();
    if (!force && now - lastFlush < 220) return;
    lastFlush = now;
    await db
      .update(pipelineTasks)
      .set({ stages, engine, ...patch })
      .where(eq(pipelineTasks.id, taskId));
  };

  const stage = (key: StageKey) => stages.find((s) => s.key === key)!;
  const log = async (key: StageKey, msg: string, progress?: number) => {
    const s = stage(key);
    s.logs.push({ ts: new Date().toISOString(), msg });
    if (s.logs.length > 120) s.logs.splice(0, s.logs.length - 120);
    await flush(false, progress !== undefined ? { progress, stage: key } : { stage: key });
  };

  const beginStage = async (key: StageKey) => {
    const s = stage(key);
    s.status = "running";
    s.startedAt = new Date().toISOString();
    await flush(true, { stage: key, progress: STAGE_SPAN[key][0] });
    await log(key, `> ${STAGE_META[key].eng} 阶段开始 —— ${STAGE_META[key].desc}`);
  };
  const endStage = async (key: StageKey) => {
    const s = stage(key);
    s.status = "done";
    s.finishedAt = new Date().toISOString();
    s.durationMs = Date.parse(s.finishedAt) - Date.parse(s.startedAt!);
    await flush(true, { progress: STAGE_SPAN[key][1] });
  };
  const tick = async (key: StageKey, ratio: number, msg?: string) => {
    const [a, b] = STAGE_SPAN[key];
    if (msg) await log(key, msg);
    await flush(false, { progress: Math.round(a + (b - a) * ratio) });
  };

  const io: ScriptIo = await prepareTaskDir(taskId, extOf(video.name));
  await fsp.writeFile(io.metaFile, JSON.stringify({ taskId, videoId: video.id, ...meta }, null, 2));

  try {
    await db.update(pipelineTasks).set({ status: "running" }).where(eq(pipelineTasks.id, taskId));

    // ---------- 1. 视频解析 ----------
    await beginStage("probe");
    await sleep(350);
    await log("probe", `容器轨道: ${video.contentType} · ${(video.sizeBytes / 1024 / 1024).toFixed(2)} MB`);
    await tick("probe", 0.5);
    await sleep(350);
    await log("probe", `图像参数: ${meta.width}×${meta.height} · 时长 ${meta.durationS.toFixed(2)}s`);
    const hasDetect = !!findScript("detect.py");
    const hasPlan = !!findScript("plan.py");
    await log(
      "probe",
      hasDetect || hasPlan
        ? `检测到外挂脚本: ${[hasDetect && "detect.py", hasPlan && "plan.py"].filter(Boolean).join(" + ")}，对应阶段将优先调用`
        : "未检测到 pipeline/scripts/ 外挂脚本，全链路使用内置模拟引擎",
    );
    await endStage("probe");

    // ---------- 2. 抽帧采样 ----------
    await beginStage("extract");
    const { intervalS, frameTimes } = planSampling(meta);
    await sleep(300);
    await log("extract", `采样策略: 每 ${intervalS.toFixed(2)}s 一帧，共 ${frameTimes.length} 帧`);
    for (let i = 1; i <= 4; i++) {
      await sleep(260);
      await tick("extract", i / 4, `建立帧索引 ${Math.round((frameTimes.length * i) / 4)}/${frameTimes.length}`);
    }
    await endStage("extract");

    // ---------- 3. 目标识别 ----------
    await beginStage("detect");
    let frames: FrameDetections[] | null = null;
    if (hasDetect) {
      try {
        await log("detect", "调用 pipeline/scripts/detect.py …");
        await ensureVideoLocal(io.videoFile, video.storageKey);
        frames = await runDetectScript(io);
        if (frames) {
          engine = "script";
          await log("detect", `脚本返回 ${frames.length} 帧检测结果`);
        }
      } catch (err) {
        await log("detect", `⚠ 外挂脚本执行失败，回退内置模拟引擎: ${err instanceof Error ? err.message : String(err)}`);
        frames = null;
      }
    }
    if (!frames) {
      frames = simulateDetections(meta, seed);
      // 模拟逐帧推理的推进过程
      const marks = [0.15, 0.35, 0.55, 0.75, 0.92];
      for (const m of marks) {
        await sleep(420);
        const upto = Math.max(1, Math.round(frames.length * m));
        const hits = frames.slice(0, upto).reduce((s, f) => s + f.boxes.length, 0);
        await tick("detect", m, `推理进度 ${upto}/${frames.length} 帧 · 累计检出 ${hits} 个目标框`);
      }
      await sleep(300);
    }
    const tracks = aggregateTracks(frames);
    await tick("detect", 1, `轨迹关联完成: ${tracks.length} 条稳定轨迹`);
    await endStage("detect");

    // ---------- 4. 航迹规划 ----------
    await beginStage("plan");
    let path: MissionPath | null = null;
    if (hasPlan) {
      try {
        await log("plan", "调用 pipeline/scripts/plan.py …");
        if (!hasDetect) {
          // detect 用内置、plan 用脚本：把内置检测结果写成契约文件供脚本读取
          await fsp.writeFile(io.detectionsFile, JSON.stringify({ frames, tracks }, null, 2));
        }
        path = await runPlanScript(io);
        if (path) {
          engine = "script";
          await log("plan", `脚本返回航线: ${path.waypoints.length} 个航点`);
        }
      } catch (err) {
        await log("plan", `⚠ 外挂脚本执行失败，回退内置模拟引擎: ${err instanceof Error ? err.message : String(err)}`);
        path = null;
      }
    }
    if (!path) {
      await sleep(450);
      await log("plan", `目标聚类: 合并 35m 半径内的落点…`);
      await tick("plan", 0.3);
      await sleep(450);
      path = planMission(meta, tracks, seed);
      const inspectCount = path.waypoints.filter((w) => w.action === "inspect").length;
      await tick("plan", 0.6, `生成 ${inspectCount} 个侦察点 · 标记 ${path.obstacles.length} 个障碍回避区`);
      await sleep(450);
      await tick("plan", 0.9, `航线解算完成: 里程 ${path.distanceM.toFixed(0)}m · 预计耗时 ${path.etaS}s`);
    }
    await endStage("plan");

    // ---------- 5. 汇总输出 ----------
    await beginStage("finalize");
    const stats = buildStats(meta, frames, tracks, path);
    await sleep(280);
    await log(
      "finalize",
      `目标 ${stats.uniqueTargets} 个（车辆${stats.byClass.vehicle} / 人员${stats.byClass.person} / 骑手${stats.byClass.rider} / 障碍${stats.byClass.obstacle}）`,
    );
    await sleep(280);
    await log("finalize", `写入任务结果，链路闭环 ✓`);
    const result: PipelineResult = {
      engine,
      frames,
      tracks,
      path,
      stats,
      generatedAt: new Date().toISOString(),
    };
    await endStage("finalize");
    await flush(true, { status: "done", progress: 100, result, finishedAt: new Date() });
  } catch (err) {
    const message = err instanceof Error ? err.message : String(err);
    const current = stages.find((s) => s.status === "running");
    if (current) {
      current.status = "failed";
      current.logs.push({ ts: new Date().toISOString(), msg: `✗ 阶段失败: ${message}` });
    }
    await db
      .update(pipelineTasks)
      .set({ status: "failed", error: message, stages, finishedAt: new Date() })
      .where(eq(pipelineTasks.id, taskId));
  }
}

function extOf(name: string): string {
  const m = /\.([a-z0-9]{2,5})$/i.exec(name);
  return m ? `.${m[1].toLowerCase()}` : ".mp4";
}

/** 外挂脚本需要真实视频文件时，从对象存储拉回本地工作目录 */
async function ensureVideoLocal(file: string, storageKey: string): Promise<void> {
  const bytes = await storage.readFile({ fileKey: storageKey });
  await fsp.writeFile(file, Buffer.from(bytes));
}

/** 启动任务（不阻塞请求线程） */
export function kickPipeline(taskId: number): void {
  void runPipeline(taskId).catch((err) => {
    console.error(`[pipeline] task ${taskId} crashed:`, err);
    getDb()
      .update(pipelineTasks)
      .set({ status: "failed", error: String(err), finishedAt: new Date() })
      .where(eq(pipelineTasks.id, taskId))
      .catch(() => undefined);
  });
}
