import type { ReactNode } from "react";
import { cn } from "@/lib/utils";

/** 切角边框面板：外层 1px 描边 + 内层面板底，构成发丝级 HUD 容器 */
export function HudPanel({
  title,
  eng,
  icon,
  right,
  children,
  className,
  bodyClassName,
  noPad,
}: {
  title?: string;
  eng?: string;
  icon?: ReactNode;
  right?: ReactNode;
  children: ReactNode;
  className?: string;
  bodyClassName?: string;
  noPad?: boolean;
}) {
  return (
    <section className={cn("notch-outer", className)}>
      <div className="notch-inner flex h-full flex-col">
        {title && (
          <header className="flex items-center justify-between gap-2 border-b border-hairline px-4 py-2.5">
            <div className="flex min-w-0 items-center gap-2.5">
              {icon ? (
                <span className="flex h-[22px] w-[22px] shrink-0 items-center justify-center border border-hud/40 bg-hud/10 text-[13px] text-hud">
                  {icon}
                </span>
              ) : (
                <span className="h-1.5 w-1.5 shrink-0 bg-hud" />
              )}
              <h2 className="truncate text-[13px] font-semibold tracking-wide">{title}</h2>
              {eng && <span className="label-tech hidden self-baseline sm:inline">{eng}</span>}
            </div>
            {right}
          </header>
        )}
        <div className={cn("min-h-0 flex-1", noPad ? "" : "p-4", bodyClassName)}>{children}</div>
      </div>
    </section>
  );
}
