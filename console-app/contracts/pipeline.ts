/**
 * 识别-规划链共享契约：前端、后端、外挂脚本三方共用同一套类型与常量。
 * 外挂脚本（pipeline/scripts/*.py）的输入输出 JSON 结构与本文件定义一一对应。
 */

// ---------------------------------------------------------------------------
// 流水线阶段
// ---------------------------------------------------------------------------

export const STAGE_KEYS = ["probe", "extract", "detect", "plan", "finalize"] as const;
export type StageKey = (typeof STAGE_KEYS)[number];

export const STAGE_META: Record<StageKey, { label: string; eng: string; desc: string }> = {
  probe: { label: "视频解析", eng: "PROBE", desc: "读取封装与轨道元数据，校验可解码性" },
  extract: { label: "抽帧采样", eng: "EXTRACT", desc: "按时序均匀采样关键帧，建立帧索引" },
  detect: { label: "目标识别", eng: "DETECT", desc: "逐帧检测地面目标并维持跨帧轨迹" },
  plan: { label: "航迹规划", eng: "PLAN", desc: "基于目标分布与障碍区生成巡检航线" },
  finalize: { label: "汇总输出", eng: "OUTPUT", desc: "汇总统计，写入任务结果" },
};

// ---------------------------------------------------------------------------
// 任务状态
// ---------------------------------------------------------------------------

export type TaskStatus = "queued" | "running" | "done" | "failed";
export type StageStatus = "pending" | "running" | "done" | "failed" | "skipped";
export type EngineKind = "builtin" | "script";

export interface StageLog {
  /** ISO 时间戳 */
  ts: string;
  msg: string;
}

export interface StageState {
  key: StageKey;
  status: StageStatus;
  startedAt?: string;
  finishedAt?: string;
  durationMs?: number;
  logs: StageLog[];
}

// ---------------------------------------------------------------------------
// 目标识别结果
// ---------------------------------------------------------------------------

export const TARGET_CLASSES = ["vehicle", "person", "rider", "obstacle"] as const;
export type TargetClass = (typeof TARGET_CLASSES)[number];

export const TARGET_CLASS_META: Record<TargetClass, { label: string; color: string }> = {
  vehicle: { label: "车辆", color: "#22d3ee" },
  person: { label: "人员", color: "#00f48e" },
  rider: { label: "骑手", color: "#fbbf24" },
  obstacle: { label: "障碍", color: "#f87171" },
};

/** 单帧单个检测框，坐标为相对画面宽高的归一化值 (0-1) */
export interface DetectionBox {
  /** 跨帧轨迹号 */
  track: number;
  cls: TargetClass;
  /** 置信度 0-1 */
  conf: number;
  x: number;
  y: number;
  w: number;
  h: number;
}

export interface FrameDetections {
  /** 距视频起点的秒数 */
  t: number;
  boxes: DetectionBox[];
}

export interface TargetTrack {
  track: number;
  cls: TargetClass;
  firstSeen: number;
  lastSeen: number;
  frameCount: number;
  maxConf: number;
  /** 轨迹平均落点（归一化坐标），供航迹规划与地图标注使用 */
  gx: number;
  gy: number;
}

// ---------------------------------------------------------------------------
// 航迹规划结果（本地 ENU 平面坐标，单位：米）
// ---------------------------------------------------------------------------

export type WaypointAction = "takeoff" | "transit" | "inspect" | "loiter" | "return" | "land";

export const WAYPOINT_ACTION_META: Record<WaypointAction, { label: string }> = {
  takeoff: { label: "起飞" },
  transit: { label: "转场" },
  inspect: { label: "抵近侦察" },
  loiter: { label: "盘旋监视" },
  return: { label: "返航" },
  land: { label: "降落" },
};

export interface Waypoint {
  seq: number;
  x: number;
  y: number;
  /** 相对起飞点高度，米 */
  alt: number;
  /** 飞离该点时的地速 m/s */
  speed: number;
  action: WaypointAction;
  note: string;
}

export interface ObstacleZone {
  x: number;
  y: number;
  /** 回避半径，米 */
  r: number;
  label: string;
}

export interface MissionPath {
  home: { x: number; y: number };
  waypoints: Waypoint[];
  obstacles: ObstacleZone[];
  /** 地面覆盖范围（米），用于地图比例尺 */
  groundWidthM: number;
  groundHeightM: number;
  distanceM: number;
  etaS: number;
}

// ---------------------------------------------------------------------------
// 汇总
// ---------------------------------------------------------------------------

export interface PipelineStats {
  durationS: number;
  width: number;
  height: number;
  sampleIntervalS: number;
  framesProcessed: number;
  totalDetections: number;
  uniqueTargets: number;
  byClass: Record<TargetClass, number>;
  pathDistanceM: number;
  pathEtaS: number;
  waypointCount: number;
  obstacleCount: number;
}

export interface PipelineResult {
  engine: EngineKind;
  frames: FrameDetections[];
  tracks: TargetTrack[];
  path: MissionPath;
  stats: PipelineStats;
  generatedAt: string;
}

/** 前端上传约束：服务端请求体上限 50MB，base64 膨胀约 33% */
export const MAX_VIDEO_BYTES = 30 * 1024 * 1024;
export const MAX_VIDEO_LABEL = "30MB";
