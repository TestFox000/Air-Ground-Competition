import { TopBar } from "@/components/TopBar";
import { HudPanel } from "@/components/HudPanel";
import { Led } from "@/components/Led";
import { IconDrone, IconTarget, IconRoute, IconRadar } from "@/components/Icons";
import { STAGE_ICONS } from "@/components/stage-icons";
import { STAGE_KEYS, STAGE_META, TARGET_CLASS_META } from "@contracts/pipeline";

function Code({ children, title }: { children: string; title?: string }) {
  return (
    <div className="border border-hairline bg-background/70">
      {title && (
        <div className="border-b border-hairline px-3 py-1.5 font-data text-[10.5px] tracking-[0.14em] text-muted-foreground">
          {title}
        </div>
      )}
      <pre className="thin-scroll overflow-x-auto px-3 py-2.5 font-data text-[11.5px] leading-relaxed text-foreground/85">
        {children}
      </pre>
    </div>
  );
}

const TREE = `pipeline/
├── core/                 ← 内置规划算法权威实现（网站后端直接调用）
│   ├── simulate.ts         目标识别模拟 + 航迹规划算法
│   └── rng.ts              确定性随机数（重跑结果可复现）
├── scripts/              ← 规定脚本目录（放进这里即被接管）
│   ├── detect.py           目标识别脚本（可选）
│   ├── plan.py             航迹规划脚本（可选）
│   ├── detect.py.example   识别脚本模板
│   └── plan.py.example     规划脚本模板
└── data/
    └── task-<id>/          每次任务的中间产物（自动创建）
        ├── meta.json         输入 · 视频元数据
        ├── input.mp4         输入 · 原始视频
        ├── detections.json   detect 输出 / plan 输入
        └── path.json         plan 输出`;

const DETECT_JSON = `{
  "frames": [
    {
      "t": 0.0,              // 距视频起点的秒数
      "boxes": [
        {
          "track": 101,      // 跨帧轨迹号，同一目标保持一致
          "cls": "vehicle",  // vehicle / person / rider / obstacle
          "conf": 0.87,      // 置信度 0-1
          "x": 0.31, "y": 0.42, "w": 0.07, "h": 0.09   // 归一化坐标
        }
      ]
    }
  ]
}`;

const PLAN_JSON = `{
  "path": {
    "home": { "x": 10, "y": 10 },          // 起飞点（本地平面坐标，米）
    "groundWidthM": 140, "groundHeightM": 79,
    "waypoints": [
      { "seq": 1, "x": 10, "y": 10, "alt": 0, "speed": 0,
        "action": "takeoff", "note": "起飞" }
      // action: takeoff / transit / inspect / loiter / return / land
    ],
    "obstacles": [ { "x": 60, "y": 40, "r": 15, "label": "障碍 #1" } ],
    "distanceM": 356.2,   // 航线里程（米）
    "etaS": 95            // 预计耗时（秒）
  }
}`;

const DETECT_CMD = `python3 pipeline/scripts/detect.py \\
  --meta   pipeline/data/task-<id>/meta.json \\
  --video  pipeline/data/task-<id>/input.mp4 \\
  --out    pipeline/data/task-<id>/detections.json`;

const PLAN_CMD = `python3 pipeline/scripts/plan.py \\
  --detections pipeline/data/task-<id>/detections.json \\
  --meta       pipeline/data/task-<id>/meta.json \\
  --out        pipeline/data/task-<id>/path.json`;

export default function Docs() {
  return (
    <div className="min-h-dvh">
      <TopBar />
      <main className="mx-auto max-w-[1100px] px-4 py-6 sm:px-6">
        <div className="mb-6">
          <h1 className="flex items-center gap-3 text-[20px] font-semibold tracking-wide">
            <span className="flex h-10 w-10 items-center justify-center border border-hud/40 bg-hud/10 text-hud">
              <IconDrone style={{ width: 22, height: 22 }} />
            </span>
            脚本接入说明
          </h1>
          <p className="mt-1 max-w-[70ch] text-[13px] leading-relaxed text-muted-foreground">
            本台的识别-规划链默认由内置模拟引擎执行，全程本地离线、不依赖任何外网服务。
            当你们把真实算法脚本放进规定目录后，后端会在对应阶段优先调用脚本；
            脚本缺失或执行失败时自动回退模拟引擎——链路在任何环境下都能跑通。
          </p>
        </div>

        <div className="grid grid-cols-1 gap-5 lg:grid-cols-2">
          <HudPanel title="规定目录" eng="LAYOUT" className="lg:col-span-2">
            <Code>{TREE}</Code>
            <div className="mt-3 grid grid-cols-1 gap-2 sm:grid-cols-2">
              {[
                ["阶段接管", "detect.py 接管「目标识别」，plan.py 接管「航迹规划」，两者独立，可只替换其一"],
                ["失败回退", "脚本超时（120s）、退出码非零或输出结构不符时，该阶段回退内置模拟引擎并记录日志"],
                ["引擎标识", "任务结果页会显示实际执行引擎：内置模拟 / 外挂脚本"],
                ["离线要求", "脚本需自带模型权重与依赖，运行期不得访问网络"],
              ].map(([t, d]) => (
                <div key={t} className="flex gap-2.5 border border-hairline bg-panel/40 px-3 py-2.5">
                  <Led tone="hud" className="mt-1.5" />
                  <div>
                    <p className="text-[12.5px] font-medium">{t}</p>
                    <p className="mt-0.5 text-[11.5px] leading-relaxed text-muted-foreground">{d}</p>
                  </div>
                </div>
              ))}
            </div>
          </HudPanel>

          <HudPanel title="目标识别契约" eng="DETECT.PY" icon={<IconTarget />}>
            <div className="space-y-3">
              <Code title="调用方式">{DETECT_CMD}</Code>
              <Code title="detections.json 输出结构">{DETECT_JSON}</Code>
              <div className="flex flex-wrap gap-2">
                {Object.entries(TARGET_CLASS_META).map(([k, v]) => (
                  <span
                    key={k}
                    className="inline-flex items-center gap-1.5 border px-2 py-1 font-data text-[11px]"
                    style={{ color: v.color, borderColor: `${v.color}55`, background: `${v.color}14` }}
                  >
                    <span className="h-1.5 w-1.5" style={{ background: v.color }} />
                    {v.label} {k}
                  </span>
                ))}
              </div>
            </div>
          </HudPanel>

          <HudPanel title="航迹规划契约" eng="PLAN.PY" icon={<IconRoute />}>
            <div className="space-y-3">
              <Code title="调用方式">{PLAN_CMD}</Code>
              <Code title="path.json 输出结构">{PLAN_JSON}</Code>
            </div>
          </HudPanel>

          <HudPanel title="链路阶段定义" eng="PIPELINE STAGES" icon={<IconRadar />} className="lg:col-span-2" bodyClassName="px-4 py-3">
            <div className="grid grid-cols-1 gap-x-8 sm:grid-cols-2 lg:grid-cols-5">
              {STAGE_KEYS.map((key, i) => {
                const Glyph = STAGE_ICONS[key];
                return (
                  <div key={key} className="border-t border-hairline py-3">
                    <p className="flex items-center gap-2 font-data text-[10.5px] tracking-[0.16em] text-hud">
                      <Glyph className="text-[13px]" />
                      {String(i + 1).padStart(2, "0")} · {STAGE_META[key].eng}
                    </p>
                    <p className="mt-1 text-[13px] font-medium">{STAGE_META[key].label}</p>
                    <p className="mt-0.5 text-[11.5px] leading-relaxed text-muted-foreground">{STAGE_META[key].desc}</p>
                  </div>
                );
              })}
            </div>
          </HudPanel>

          <HudPanel title="本地运行" eng="LOCAL RUN" className="lg:col-span-2">
            <Code title="导出代码后在你们自己的机器上">{`npm install
npm run db:push      # 需要本地 MySQL（导出代码不含平台数据库，请自备）
npm run build && npm start   #  http://localhost:3000`}</Code>
            <p className="mt-3 text-[12px] leading-relaxed text-muted-foreground">
              说明：平台版数据库与对象存储不随代码导出；本地部署需自备 MySQL 并配置 .env 中的 DATABASE_URL。
              视频文件在平台侧存于对象存储，导出后需将视频文件放入本地存储或改为本地磁盘读取。
              脚本链路本身（pipeline/）完全离线，无任何外部依赖。
            </p>
          </HudPanel>
        </div>
      </main>
    </div>
  );
}
