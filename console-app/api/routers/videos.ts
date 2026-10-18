import { z } from "zod";
import { TRPCError } from "@trpc/server";
import { desc, eq } from "drizzle-orm";
import { createRouter, publicQuery } from "../middleware";
import { storage } from "../lib/storage";
import { getDb } from "../queries/connection";
import { pipelineTasks, videos } from "../../db/schema";
import { MAX_VIDEO_BYTES } from "../../contracts/pipeline";

export const videosRouter = createRouter({
  /** 上传视频：浏览器 → 后端 → 平台对象存储，落库视频元数据 */
  upload: publicQuery
    .input(
      z.object({
        name: z.string().min(1).max(200),
        contentBase64: z.string(),
        contentType: z.string().default("video/mp4"),
        durationMs: z.number().int().min(0).default(0),
        width: z.number().int().min(0).default(0),
        height: z.number().int().min(0).default(0),
      }),
    )
    .mutation(async ({ input }) => {
      const bytes = Uint8Array.from(Buffer.from(input.contentBase64, "base64"));
      if (bytes.byteLength === 0) throw new TRPCError({ code: "BAD_REQUEST", message: "空文件" });
      if (bytes.byteLength > MAX_VIDEO_BYTES) {
        throw new TRPCError({ code: "PAYLOAD_TOO_LARGE", message: `视频超过 ${Math.round(MAX_VIDEO_BYTES / 1024 / 1024)}MB 上限` });
      }
      const saved = await storage.uploadFile({
        fileContent: bytes,
        fileName: `videos/${input.name}`,
        contentType: input.contentType,
      });
      const [row] = await getDb()
        .insert(videos)
        .values({
          name: input.name,
          storageKey: saved.key,
          contentType: input.contentType,
          sizeBytes: bytes.byteLength,
          durationMs: input.durationMs,
          width: input.width,
          height: input.height,
        })
        .$returningId();
      return { id: row.id, key: saved.key };
    }),

  /** 视频库列表：附带每个视频最近一次任务的状态 */
  list: publicQuery.query(async () => {
    const db = getDb();
    const allVideos = await db.select().from(videos).orderBy(desc(videos.id));
    const allTasks = await db.select().from(pipelineTasks).orderBy(desc(pipelineTasks.id));
    const latestByVideo = new Map<number, (typeof allTasks)[number]>();
    const countByVideo = new Map<number, number>();
    for (const t of allTasks) {
      if (!latestByVideo.has(t.videoId)) latestByVideo.set(t.videoId, t);
      countByVideo.set(t.videoId, (countByVideo.get(t.videoId) ?? 0) + 1);
    }
    return allVideos.map((v) => ({
      ...v,
      latestTask: latestByVideo.get(v.id) ?? null,
      taskCount: countByVideo.get(v.id) ?? 0,
    }));
  }),

  detail: publicQuery.input(z.object({ id: z.number() })).query(async ({ input }) => {
    const row = await getDb().query.videos.findFirst({ where: eq(videos.id, input.id) });
    if (!row) throw new TRPCError({ code: "NOT_FOUND", message: "视频不存在" });
    return row;
  }),

  /** 播放地址：短时预签名 URL，只存 key 不落 URL */
  playbackUrl: publicQuery.input(z.object({ id: z.number() })).query(async ({ input }) => {
    const row = await getDb().query.videos.findFirst({ where: eq(videos.id, input.id) });
    if (!row) throw new TRPCError({ code: "NOT_FOUND", message: "视频不存在" });
    const { url } = await storage.getPresignedUrl({ key: row.storageKey });
    return { url };
  }),
});
