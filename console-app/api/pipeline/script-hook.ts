import { execFile } from "node:child_process";
import fs from "node:fs";
import fsp from "node:fs/promises";
import path from "node:path";
import type { FrameDetections, MissionPath } from "../../contracts/pipeline";

/**
 * 外挂脚本对接层（离线预留接口）
 *
 * 规定目录：项目根下 pipeline/scripts/
 *   - detect.py  目标识别脚本：python3 detect.py --meta meta.json --video input.mp4 --out detections.json
 *   - plan.py    航迹规划脚本：python3 plan.py --detections detections.json --meta meta.json --out path.json
 * 中间产物落在 pipeline/data/task-<id>/ 下。
 *
 * 脚本不存在或执行失败时，对应阶段自动回退到内置模拟引擎，保证链路永远可跑通。
 */

export const SCRIPTS_DIR = path.resolve(process.cwd(), "pipeline", "scripts");
export const WORK_ROOT = path.resolve(process.cwd(), "pipeline", "data");

const SCRIPT_TIMEOUT_MS = 120_000;

export function findScript(name: "detect.py" | "plan.py"): string | null {
  const p = path.join(SCRIPTS_DIR, name);
  return fs.existsSync(p) ? p : null;
}

function runPython(script: string, args: Record<string, string>): Promise<void> {
  const argv = Object.entries(args).flatMap(([k, v]) => [`--${k}`, v]);
  return new Promise((resolve, reject) => {
    execFile(
      "python3",
      [script, ...argv],
      { timeout: SCRIPT_TIMEOUT_MS, maxBuffer: 8 * 1024 * 1024 },
      (err, _stdout, stderr) => {
        if (err) reject(new Error(`${path.basename(script)} 退出异常: ${err.message}${stderr ? ` | ${stderr.slice(0, 400)}` : ""}`));
        else resolve();
      },
    );
  });
}

async function readJson<T>(file: string): Promise<T> {
  const raw = await fsp.readFile(file, "utf-8");
  return JSON.parse(raw) as T;
}

/** 粗略校验 detections.json 结构，不合法即视为脚本失败 */
function validFrames(data: unknown): data is { frames: FrameDetections[] } {
  if (!data || typeof data !== "object") return false;
  const frames = (data as { frames?: unknown }).frames;
  if (!Array.isArray(frames) || frames.length === 0) return false;
  const f0 = frames[0] as { t?: unknown; boxes?: unknown };
  if (typeof f0?.t !== "number" || !Array.isArray(f0.boxes)) return false;
  const b0 = (f0.boxes as unknown[])[0] as Record<string, unknown> | undefined;
  if (!b0) return true; // 空框帧合法
  return ["track", "cls", "conf", "x", "y", "w", "h"].every((k) => k in b0);
}

function validPath(data: unknown): data is { path: MissionPath } {
  const p = (data as { path?: MissionPath })?.path;
  return !!p && Array.isArray(p.waypoints) && p.waypoints.length >= 2 && typeof p.distanceM === "number";
}

export interface ScriptIo {
  taskDir: string;
  metaFile: string;
  videoFile: string;
  detectionsFile: string;
  pathFile: string;
}

export async function prepareTaskDir(taskId: number, videoExt: string): Promise<ScriptIo> {
  const taskDir = path.join(WORK_ROOT, `task-${taskId}`);
  await fsp.mkdir(taskDir, { recursive: true });
  return {
    taskDir,
    metaFile: path.join(taskDir, "meta.json"),
    videoFile: path.join(taskDir, `input${videoExt}`),
    detectionsFile: path.join(taskDir, "detections.json"),
    pathFile: path.join(taskDir, "path.json"),
  };
}

/** 运行 detect.py，返回帧检测序列；脚本缺失或失败返回 null（调用方回退内置引擎） */
export async function runDetectScript(io: ScriptIo): Promise<FrameDetections[] | null> {
  const script = findScript("detect.py");
  if (!script) return null;
  await runPython(script, { meta: io.metaFile, video: io.videoFile, out: io.detectionsFile });
  const data = await readJson<unknown>(io.detectionsFile);
  if (!validFrames(data)) throw new Error("detections.json 结构不符合契约（缺少 frames[].boxes 字段）");
  return data.frames;
}

/** 运行 plan.py，返回航线；脚本缺失或失败返回 null */
export async function runPlanScript(io: ScriptIo): Promise<MissionPath | null> {
  const script = findScript("plan.py");
  if (!script) return null;
  await runPython(script, { detections: io.detectionsFile, meta: io.metaFile, out: io.pathFile });
  const data = await readJson<unknown>(io.pathFile);
  if (!validPath(data)) throw new Error("path.json 结构不符合契约（缺少 path.waypoints / path.distanceM）");
  return data.path;
}
