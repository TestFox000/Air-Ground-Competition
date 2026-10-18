import { useEffect, useState } from "react";
import { Link, NavLink, useLocation } from "react-router";
import { cn } from "@/lib/utils";
import { Led } from "./Led";
import { IconDrone } from "./Icons";

/** 品牌标：无人机图标 + 雷达扫描环 */
function BrandMark() {
  return (
    <span className="relative flex h-[38px] w-[38px] shrink-0 items-center justify-center">
      <svg viewBox="0 0 38 38" className="absolute inset-0 h-full w-full" aria-hidden>
        <circle cx="19" cy="19" r="17.5" fill="none" stroke="hsl(var(--hud) / 0.4)" strokeWidth="1" />
        <circle cx="19" cy="19" r="12.5" fill="none" stroke="hsl(var(--hud) / 0.18)" strokeWidth="1" />
        <g className="animate-radar" style={{ transformOrigin: "19px 19px" }}>
          <path d="M19 19 L19 1.5 A17.5 17.5 0 0 1 31.4 6.6 Z" fill="hsl(var(--hud) / 0.22)" />
          <line x1="19" y1="19" x2="19" y2="1.5" stroke="hsl(var(--hud) / 0.9)" strokeWidth="1.2" />
        </g>
        <line x1="19" y1="1.5" x2="19" y2="36.5" stroke="hsl(var(--hud) / 0.12)" strokeWidth="1" />
        <line x1="1.5" y1="19" x2="36.5" y2="19" stroke="hsl(var(--hud) / 0.12)" strokeWidth="1" />
      </svg>
      <IconDrone className="relative z-10 text-[19px] text-hud hud-glow-soft" />
    </span>
  );
}

function LiveClock() {
  const [now, setNow] = useState(() => new Date());
  useEffect(() => {
    const id = setInterval(() => setNow(new Date()), 1000);
    return () => clearInterval(id);
  }, []);
  const pad = (n: number) => String(n).padStart(2, "0");
  return (
    <span className="font-data hud-glow-soft text-[13px] tracking-wider text-hud">
      {pad(now.getHours())}:{pad(now.getMinutes())}
      <span className="hidden sm:inline">:{pad(now.getSeconds())}</span>
    </span>
  );
}

const NAV = [
  { to: "/", label: "控制台", eng: "CONSOLE" },
  { to: "/docs", label: "接入说明", eng: "INTEGRATION" },
];

export function TopBar() {
  const location = useLocation();
  return (
    <header className="sticky top-0 z-40 border-b border-hairline bg-background/85 backdrop-blur-sm">
      <div className="mx-auto flex h-14 max-w-[1600px] items-center gap-3 px-4 sm:gap-5 sm:px-6">
        <Link to="/" className="flex min-w-0 items-center gap-2.5" aria-label="返回控制台">
          <BrandMark />
          <div className="min-w-0 leading-tight">
            <div className="max-w-[130px] truncate text-[13px] font-semibold tracking-wide sm:max-w-none sm:text-[14px]">
              无人机识别-规划仿真台
            </div>
            <div className="label-tech hidden truncate md:block">UAV DETECT-PLAN SIMULATOR</div>
          </div>
        </Link>

        <nav className="ml-1 flex items-center sm:ml-6 sm:gap-1" aria-label="主导航">
          {NAV.map((item) => {
            const active = item.to === "/" ? location.pathname === "/" : location.pathname.startsWith(item.to);
            return (
              <NavLink
                key={item.to}
                to={item.to}
                className={cn(
                  "flex min-h-[44px] items-center gap-1.5 whitespace-nowrap border-b-2 px-2 font-mono text-[12px] tracking-[0.08em] transition-colors duration-150 sm:px-3 sm:tracking-[0.12em]",
                  active
                    ? "border-hud text-hud"
                    : "border-transparent text-muted-foreground hover:text-foreground",
                )}
              >
                {item.label}
                <span className="hidden lg:inline text-[10px] opacity-60">{item.eng}</span>
              </NavLink>
            );
          })}
        </nav>

        <div className="ml-auto flex items-center gap-3 sm:gap-5">
          <div className="hidden items-center gap-2 sm:flex" title="全链路本地离线运行，不依赖外网服务">
            <Led tone="hud" />
            <span className="label-tech">本地离线 OFFLINE-READY</span>
          </div>
          <span className="hidden min-[420px]:inline">
            <LiveClock />
          </span>
        </div>
      </div>
    </header>
  );
}
