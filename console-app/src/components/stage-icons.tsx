import type { ComponentType } from "react";
import type { StageKey } from "@contracts/pipeline";
import { IconCamera, IconFilm, IconTarget, IconRoute, IconPaperPlane } from "./Icons";

/** 各链路阶段的主题图标（iconfont.cn 矢量图标，随 currentColor 着色） */
export const STAGE_ICONS: Record<StageKey, ComponentType<{ className?: string }>> = {
  probe: IconCamera,
  extract: IconFilm,
  detect: IconTarget,
  plan: IconRoute,
  finalize: IconPaperPlane,
};
