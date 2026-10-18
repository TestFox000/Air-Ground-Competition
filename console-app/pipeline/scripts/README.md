# 外挂脚本接口契约（离线识别-规划链）

把你们本地已有的识别 / 规划脚本放进本目录，应用后端会在对应阶段**优先调用脚本**；
脚本缺失或执行失败时，该阶段自动回退到内置模拟引擎，链路永远可跑通。

## 规定目录

```
pipeline/
├── core/               ← 内置规划算法权威实现（simulate.ts / rng.ts）
│                         网站后端直接调用；你们更新规划代码也可以改这里
├── scripts/            ← 放外挂脚本（本目录）
│   ├── detect.py       ← 目标识别（可选）
│   └── plan.py         ← 航迹规划（可选）
└── data/
    └── task-<id>/      ← 每次任务的中间产物（自动创建）
        ├── meta.json       输入：视频元数据
        ├── input.mp4       输入：原始视频文件
        ├── detections.json detect.py 的输出 / plan.py 的输入
        └── path.json       plan.py 的输出
```

## detect.py —— 目标识别

调用方式：

```bash
python3 pipeline/scripts/detect.py \
  --meta   pipeline/data/task-<id>/meta.json \
  --video  pipeline/data/task-<id>/input.mp4 \
  --out    pipeline/data/task-<id>/detections.json
```

`meta.json` 输入：

```json
{ "taskId": 1, "videoId": 1, "durationS": 12.0, "width": 1280, "height": 720 }
```

`detections.json` 输出（坐标为相对画面宽高的归一化值 0-1）：

```json
{
  "frames": [
    {
      "t": 0.0,
      "boxes": [
        { "track": 101, "cls": "vehicle", "conf": 0.87, "x": 0.31, "y": 0.42, "w": 0.07, "h": 0.09 }
      ]
    }
  ]
}
```

- `cls` 取值：`vehicle`（车辆）/ `person`（人员）/ `rider`（骑手）/ `obstacle`（障碍）
- `track` 为跨帧轨迹号，同一目标在不同帧中保持一致
- `t` 为距视频起点的秒数

## plan.py —— 航迹规划

调用方式：

```bash
python3 pipeline/scripts/plan.py \
  --detections pipeline/data/task-<id>/detections.json \
  --meta       pipeline/data/task-<id>/meta.json \
  --out        pipeline/data/task-<id>/path.json
```

`path.json` 输出（本地平面坐标，单位：米）：

```json
{
  "path": {
    "home": { "x": 10, "y": 10 },
    "groundWidthM": 140,
    "groundHeightM": 79,
    "waypoints": [
      { "seq": 1, "x": 10, "y": 10, "alt": 0, "speed": 0, "action": "takeoff", "note": "起飞" }
    ],
    "obstacles": [ { "x": 60, "y": 40, "r": 15, "label": "障碍 #1" } ],
    "distanceM": 356.2,
    "etaS": 95
  }
}
```

- `action` 取值：`takeoff` / `transit` / `inspect` / `loiter` / `return` / `land`

## 运行约定

- 单脚本超时 120s；需自行保证脚本无网络依赖（离线可用）
- 两个脚本相互独立：可以只替换其中一个，另一个阶段仍用内置引擎
- 前端「接入说明」页可在线查看本契约
