import { z } from "zod";
import { TRPCError } from "@trpc/server";
import { desc, eq } from "drizzle-orm";
import { createRouter, publicQuery } from "../middleware";
import { getDb } from "../queries/connection";
import { pipelineTasks, videos } from "../../db/schema";
import { kickPipeline } from "../pipeline/engine";

export const tasksRouter = createRouter({
  /** 点击「开始」：为该视频创建任务并异步启动识别-规划链 */
  start: publicQuery
    .input(z.object({ videoId: z.number() }))
    .mutation(async ({ input }) => {
      const db = getDb();
      const video = await db.query.videos.findFirst({ where: eq(videos.id, input.videoId) });
      if (!video) throw new TRPCError({ code: "NOT_FOUND", message: "视频不存在" });
      const latest = await db.query.pipelineTasks.findFirst({
        where: eq(pipelineTasks.videoId, input.videoId),
        orderBy: desc(pipelineTasks.id),
      });
      if (latest && (latest.status === "running" || latest.status === "queued")) {
        // 幂等：已有进行中的任务就直接返回它，避免重复启动
        return { id: latest.id, alreadyRunning: true };
      }
      const [row] = await db.insert(pipelineTasks).values({ videoId: input.videoId }).$returningId();
      kickPipeline(row.id);
      return { id: row.id, alreadyRunning: false };
    }),

  /** 任务详情（前端轮询：阶段、进度、日志、最终结果） */
  get: publicQuery.input(z.object({ id: z.number() })).query(async ({ input }) => {
    const row = await getDb().query.pipelineTasks.findFirst({ where: eq(pipelineTasks.id, input.id) });
    if (!row) throw new TRPCError({ code: "NOT_FOUND", message: "任务不存在" });
    return row;
  }),

  /** 某视频的历史运行记录（摘要字段，不含 stages/result 大字段） */
  listByVideo: publicQuery.input(z.object({ videoId: z.number() })).query(async ({ input }) => {
    return getDb()
      .select({
        id: pipelineTasks.id,
        videoId: pipelineTasks.videoId,
        status: pipelineTasks.status,
        stage: pipelineTasks.stage,
        progress: pipelineTasks.progress,
        engine: pipelineTasks.engine,
        error: pipelineTasks.error,
        createdAt: pipelineTasks.createdAt,
        finishedAt: pipelineTasks.finishedAt,
      })
      .from(pipelineTasks)
      .where(eq(pipelineTasks.videoId, input.videoId))
      .orderBy(desc(pipelineTasks.id))
      .limit(20);
  }),
});
