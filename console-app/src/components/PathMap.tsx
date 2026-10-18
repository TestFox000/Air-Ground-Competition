import { useMemo } from "react";
import type { MissionPath, TargetTrack } from "@contracts/pipeline";
import { TARGET_CLASS_META, WAYPOINT_ACTION_META } from "@contracts/pipeline";
import { DroneGlyph } from "./DroneGlyph";

interface Props {
  path: MissionPath;
  tracks: TargetTrack[];
}

const VW = 760;
const VH = 460;
const MARGIN = 46;

/** 俯视航迹地图：地面网格 + 障碍回避区 + 目标落点 + 规划航线 */
export function PathMap({ path, tracks }: Props) {
  const geo = useMemo(() => {
    const mapW = VW - MARGIN * 2;
    const mapH = VH - MARGIN * 2 - 26;
    const scale = Math.min(mapW / path.groundWidthM, mapH / path.groundHeightM);
    const usedW = path.groundWidthM * scale;
    const usedH = path.groundHeightM * scale;
    const ox = MARGIN + (mapW - usedW) / 2;
    const oy = 14 + (mapH - usedH) / 2;
    const px = (x: number) => ox + x * scale;
    const py = (y: number) => oy + (path.groundHeightM - y) * scale; // 地面 y 轴朝北，SVG y 朝下
    return { scale, ox, oy, usedW, usedH, px, py };
  }, [path]);

  const { px, py, scale } = geo;
  const gridStep = 20; // 米
  const vLines = Math.floor(path.groundWidthM / gridStep);
  const hLines = Math.floor(path.groundHeightM / gridStep);
  const routePoints = path.waypoints.map((w) => `${px(w.x)},${py(w.y)}`).join(" ");
  const targets = tracks.filter((t) => t.cls !== "obstacle");

  return (
    <svg viewBox={`0 0 ${VW} ${VH}`} className="h-auto w-full select-none" role="img" aria-label="航迹规划俯视图">
      <defs>
        <pattern id="hatch-red" width="6" height="6" patternTransform="rotate(45)" patternUnits="userSpaceOnUse">
          <rect width="6" height="6" fill="hsl(var(--signal-red) / 0.06)" />
          <line x1="0" y1="0" x2="0" y2="6" stroke="hsl(var(--signal-red) / 0.45)" strokeWidth="1.4" />
        </pattern>
      </defs>

      {/* 底 */}
      <rect x="0" y="0" width={VW} height={VH} fill="hsl(222 50% 6%)" />

      {/* 网格 */}
      {Array.from({ length: vLines + 1 }, (_, i) => {
        const x = px(i * gridStep);
        return (
          <g key={`v${i}`}>
            <line x1={x} y1={py(path.groundHeightM)} x2={x} y2={py(0)} stroke="hsl(223 42% 15%)" strokeWidth="1" />
            {i % 2 === 0 && (
              <text x={x} y={py(0) + 14} fontSize="9" fill="hsl(217 18% 45%)" textAnchor="middle" fontFamily="'JetBrains Mono', monospace">
                {i * gridStep}
              </text>
            )}
          </g>
        );
      })}
      {Array.from({ length: hLines + 1 }, (_, i) => {
        const y = py(i * gridStep);
        return (
          <g key={`h${i}`}>
            <line x1={px(0)} y1={y} x2={px(path.groundWidthM)} y2={y} stroke="hsl(223 42% 15%)" strokeWidth="1" />
            {i % 2 === 0 && (
              <text x={px(0) - 6} y={y + 3} fontSize="9" fill="hsl(217 18% 45%)" textAnchor="end" fontFamily="'JetBrains Mono', monospace">
                {i * gridStep}
              </text>
            )}
          </g>
        );
      })}
      <rect
        x={px(0)}
        y={py(path.groundHeightM)}
        width={path.groundWidthM * scale}
        height={path.groundHeightM * scale}
        fill="none"
        stroke="hsl(223 42% 24%)"
        strokeWidth="1"
      />

      {/* 障碍回避区 */}
      {path.obstacles.map((ob, i) => (
        <g key={`ob${i}`}>
          <circle cx={px(ob.x)} cy={py(ob.y)} r={ob.r * scale} fill="url(#hatch-red)" stroke="hsl(var(--signal-red) / 0.7)" strokeWidth="1" strokeDasharray="4 3" />
          <text x={px(ob.x)} y={py(ob.y) - ob.r * scale - 4} fontSize="9" fill="hsl(var(--signal-red))" textAnchor="middle" fontFamily="'JetBrains Mono', monospace">
            {ob.label} R{ob.r}m
          </text>
        </g>
      ))}

      {/* 目标落点 */}
      {targets.map((t) => {
        const gx = t.gx * path.groundWidthM;
        const gy = (1 - t.gy) * path.groundHeightM;
        const cx = px(gx);
        const cy = py(gy);
        const color = TARGET_CLASS_META[t.cls].color;
        return (
          <g key={`t${t.track}`}>
            <path d={`M ${cx} ${cy - 5} L ${cx + 5} ${cy} L ${cx} ${cy + 5} L ${cx - 5} ${cy} Z`} fill={color} stroke="#050810" strokeWidth="1">
              <title>{`${TARGET_CLASS_META[t.cls].label} #${t.track} · 峰值置信度 ${(t.maxConf * 100).toFixed(0)}%`}</title>
            </path>
            <text x={cx + 8} y={cy + 3} fontSize="9" fill={color} fontFamily="'JetBrains Mono', monospace">
              #{t.track}
            </text>
          </g>
        );
      })}

      {/* 航线 */}
      <polyline
        points={routePoints}
        fill="none"
        stroke="hsl(var(--hud) / 0.25)"
        strokeWidth="5"
        strokeLinejoin="round"
        strokeLinecap="round"
      />
      <polyline
        points={routePoints}
        fill="none"
        stroke="hsl(var(--hud))"
        strokeWidth="1.8"
        strokeDasharray="7 5"
        strokeLinejoin="round"
        className="animate-route"
        style={{ filter: "drop-shadow(0 0 4px hsl(var(--hud) / 0.6))" }}
      />

      {/* 航点 */}
      {path.waypoints.map((w) => {
        const cx = px(w.x);
        const cy = py(w.y);
        const isHome = w.action === "takeoff" || w.action === "land";
        if (isHome) return null;
        return (
          <g key={`wp${w.seq}`}>
            {w.action === "inspect" && (
              <circle cx={cx} cy={cy} r={13} fill="none" stroke="hsl(var(--hud) / 0.45)" strokeWidth="1" strokeDasharray="3 3" />
            )}
            <circle cx={cx} cy={cy} r={8.5} fill="#0c1220" stroke="hsl(var(--hud))" strokeWidth="1.4">
              <title>{`WP${w.seq} ${WAYPOINT_ACTION_META[w.action].label} · (${w.x}, ${w.y}) · 高度 ${w.alt}m${w.speed ? ` · ${w.speed}m/s` : ""}`}</title>
            </circle>
            <text x={cx} y={cy + 3.2} fontSize="9" fontWeight="600" fill="hsl(var(--hud))" textAnchor="middle" fontFamily="'JetBrains Mono', monospace">
              {w.seq}
            </text>
          </g>
        );
      })}

      {/* 起飞点：无人机图标 */}
      <g>
        <circle cx={px(path.home.x)} cy={py(path.home.y)} r={15} fill="none" stroke="hsl(var(--hud) / 0.35)" strokeWidth="1" strokeDasharray="3 3" />
        <g
          transform={`translate(${px(path.home.x) - 11}, ${py(path.home.y) - 11}) scale(${22 / 1024})`}
          fill="hsl(var(--hud))"
          style={{ filter: "drop-shadow(0 0 5px hsl(var(--hud) / 0.7))" }}
        >
          <DroneGlyph />
        </g>
        <text x={px(path.home.x)} y={py(path.home.y) + 26} fontSize="9" fill="hsl(var(--hud))" textAnchor="middle" fontFamily="'JetBrains Mono', monospace" letterSpacing="1">
          HOME
        </text>
      </g>

      {/* 指北针 */}
      <g transform={`translate(${VW - 34}, 34)`}>
        <circle r="13" fill="none" stroke="hsl(223 42% 26%)" strokeWidth="1" />
        <path d="M 0 -8 L 4 5 L 0 2 L -4 5 Z" fill="hsl(var(--hud))" />
        <text y="-17" fontSize="9" fill="hsl(var(--hud))" textAnchor="middle" fontFamily="'JetBrains Mono', monospace">N</text>
      </g>

      {/* 比例尺 */}
      <g transform={`translate(${MARGIN}, ${VH - 18})`}>
        <line x1="0" y1="0" x2={50 * scale} y2="0" stroke="hsl(217 18% 55%)" strokeWidth="1.4" />
        <line x1="0" y1="-3" x2="0" y2="3" stroke="hsl(217 18% 55%)" strokeWidth="1.4" />
        <line x1={50 * scale} y1="-3" x2={50 * scale} y2="3" stroke="hsl(217 18% 55%)" strokeWidth="1.4" />
        <text x={(50 * scale) / 2} y="-5" fontSize="9" fill="hsl(217 18% 60%)" textAnchor="middle" fontFamily="'JetBrains Mono', monospace">
          50m
        </text>
      </g>

      {/* 图例 */}
      <g transform={`translate(${VW - MARGIN}, ${VH - 24})`} fontSize="9.5" fontFamily="'JetBrains Mono', monospace" textAnchor="end">
        <g transform="translate(-218, 4)">
          <line x1="0" y1="-3" x2="16" y2="-3" stroke="hsl(var(--hud))" strokeWidth="1.8" strokeDasharray="5 3" />
          <text x="22" y="0" fill="hsl(210 20% 80%)" textAnchor="start">航线</text>
        </g>
        <g transform="translate(-148, 4)">
          <path d="M 6 -8 L 11 -3 L 6 2 L 1 -3 Z" fill="hsl(var(--signal-cyan))" />
          <text x="17" y="0" fill="hsl(210 20% 80%)" textAnchor="start">目标</text>
        </g>
        <g transform="translate(-78, 4)">
          <circle cx="6" cy="-3" r="5" fill="url(#hatch-red)" stroke="hsl(var(--signal-red) / 0.7)" strokeWidth="1" />
          <text x="17" y="0" fill="hsl(210 20% 80%)" textAnchor="start">障碍区</text>
        </g>
        <g transform="translate(0, 4)">
          <g transform="translate(0, -12) scale(0.0117)" fill="hsl(var(--hud))">
            <DroneGlyph />
          </g>
          <text x="18" y="0" fill="hsl(210 20% 80%)" textAnchor="start">起飞点</text>
        </g>
      </g>
    </svg>
  );
}
