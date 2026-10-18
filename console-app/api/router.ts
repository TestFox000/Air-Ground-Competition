import { createRouter, publicQuery } from "./middleware";
import { videosRouter } from "./routers/videos";
import { tasksRouter } from "./routers/tasks";

export const appRouter = createRouter({
  ping: publicQuery.query(() => ({ ok: true, ts: Date.now() })),
  videos: videosRouter,
  tasks: tasksRouter,
});

export type AppRouter = typeof appRouter;
