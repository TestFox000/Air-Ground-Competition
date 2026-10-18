import {
  mysqlTable,
  mysqlEnum,
  serial,
  varchar,
  text,
  int,
  bigint,
  json,
  timestamp,
} from "drizzle-orm/mysql-core";
import type { PipelineResult, StageState } from "../contracts/pipeline";

export const videos = mysqlTable("videos", {
  id: serial("id").primaryKey(),
  name: varchar("name", { length: 255 }).notNull(),
  storageKey: varchar("storage_key", { length: 512 }).notNull().unique(),
  contentType: varchar("content_type", { length: 64 }).notNull().default("video/mp4"),
  sizeBytes: bigint("size_bytes", { mode: "number", unsigned: true }).notNull(),
  durationMs: int("duration_ms").notNull().default(0),
  width: int("width").notNull().default(0),
  height: int("height").notNull().default(0),
  createdAt: timestamp("created_at").notNull().defaultNow(),
});

export const pipelineTasks = mysqlTable("pipeline_tasks", {
  id: serial("id").primaryKey(),
  videoId: bigint("video_id", { mode: "number", unsigned: true })
    .notNull()
    .references(() => videos.id),
  status: mysqlEnum("status", ["queued", "running", "done", "failed"])
    .notNull()
    .default("queued"),
  /** 当前阶段 key（contracts/pipeline.ts 的 StageKey） */
  stage: varchar("stage", { length: 32 }),
  /** 整体进度 0-100 */
  progress: int("progress").notNull().default(0),
  /** 实际执行引擎：内置模拟 / 外挂脚本 */
  engine: varchar("engine", { length: 16 }),
  stages: json("stages").$type<StageState[]>(),
  result: json("result").$type<PipelineResult>(),
  error: text("error"),
  createdAt: timestamp("created_at").notNull().defaultNow(),
  updatedAt: timestamp("updated_at").notNull().defaultNow().onUpdateNow(),
  finishedAt: timestamp("finished_at"),
});

export type Video = typeof videos.$inferSelect;
export type PipelineTask = typeof pipelineTasks.$inferSelect;
