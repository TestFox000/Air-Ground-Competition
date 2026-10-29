# -*- coding: utf-8 -*-
r"""调参实验室 · 实验目录（**唯一真源**）——"改这个键，我该看见什么"。

为什么要有这份目录（而不是让每个人自己写一行命令）
------------------------------------------------------------------------
`sim_cli` 已经能把"改一个键"变成一条自包含命令（`--set pursuit.x=1.5`），
但**会敲命令 ≠ 会做对照实验**。现场反复出现三类假结论：

1. **改错了键**：`--set pursuit.lookahead_min=0.3`（少了 `_m`）→ Config 里没有这个键，
   `get()` 返回 None，跑完指标一模一样，于是被当成"这个参数没用"（t1 卡 §0.3 黑名单就是
   这么攒出来的 12 条）。
2. **改的键被别的键钳住**：前视距离 = `clamp(k_v·|v|, [min, max])`
   （`dzrobot_v2/src/dzbrain/dzbrain/core/config.py:917`），当前 yaml 下
   `1.2×0.35=0.42` 已被 `max=0.40` 钳住 ⇒ 单独把 `lookahead_min_m` 从 0.36 改到 0.18
   **一个数都不会动**（`lookahead_min` 那个实验就是把这件事实测出来并钉住的）。
3. **两臂之间顺手改了别的**：没有 A 臂（出厂）就没法说"我看见的是这个键的功劳"。

所以每个实验都**只声明 A 臂（出厂现值）与 B 臂（单变量改值）**，由 `runner.py`
机器跑、机器比、机器判"这段到底有没有变化"。目录本身只放**声明与证据锚点**，
不放数字——数字一律由实跑生成（禁占位数字）。

字段约定（新增实验照抄这一段）
------------------------------------------------------------------------
    id          实验号（ASCII，= 产物目录名）
    title       人话名字
    group       "循迹参数"（能改的键） / "观测型"（只影响判定不影响轨迹） / "注入场景"（模拟现实误差）
    key         被测配置点号键；注入类写 sim.<SimConfig 字段>
    now         robot.yaml 现值（带 路径:行 锚点；不在 yaml 里的写"yaml 未写，取 SCHEMA 默认"）
    schema      SCHEMA 默认档与合法范围（带锚点）
    consumed    代码里真正读它的位置（带锚点）——"改了为什么会有反应"的证据
    question    这个实验回答的人话问题
    from_to     结论里"旧→新"的写法（字符串，只做展示）
    watch       结论里点名比较的指标（顺序 = 结论里的顺序）
    arms        A/B 两臂：set=[点号键=值]→转 --set；flags=[...]→直接给 sim_cli
    expect      change=应看到变化 / same=应逐位相同 / obs_only=轨迹相同但观测指标变
    expect_note 机理说明（判据失败时就是给人看的诊断）

★ 判据是**机器判**（`runner.check_expectation`）：`expect=same` 的实验如果真出现差异，
说明模型里有随机源或别的键被带动了，实验室会直接判失败并退出码 3 ——
"改了没反应"和"以为没反应其实有反应"都必须当场暴露，不许靠人眼扫表。
"""

from __future__ import annotations

#: 实验目录：顺序即报告 / index 的顺序（先教"能改的键"，再教"只看不动的观测"，最后是注入场景）
EXPERIMENTS: list[dict] = [
    # ------------------------------------------------------------------ 循迹参数
    {
        "id": "cruise_speed_up",
        "title": "巡航速度（提速档）",
        "group": "循迹参数",
        "key": "pursuit.cruise_speed_mps",
        "now": "0.35   [dzrobot_v2/config/robot.yaml:629]",
        "schema": "默认 0.15，合法 0.02~0.60   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:96]",
        "consumed": "pursuit/controller.py:277（速度剖面四层 min 的第 1 层）",
        "question": "把车开快，换回来的是什么？",
        "from_to": ("0.35", "0.60"),
        "watch": ["time_s", "cross_track_rms_m", "out_of_bounds_steps", "steer_rate_p95_deg_s"],
        "arms": [
            {"id": "A", "label": "默认 0.35", "set": [], "flags": []},
            {"id": "B", "label": "改 0.60", "set": ["pursuit.cruise_speed_mps=0.60"], "flags": []},
        ],
        "expect": "change",
        "expect_note": "提速直接缩短总时长；代价出现在横向误差 / 越界拍 / 舵角速率上。",
    },
    {
        "id": "cruise_speed_down",
        "title": "巡航速度（安全档）",
        "group": "循迹参数",
        "key": "pursuit.cruise_speed_mps",
        "now": "0.35   [dzrobot_v2/config/robot.yaml:629]",
        "schema": "默认 0.15，合法 0.02~0.60   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:96]",
        "consumed": "pursuit/controller.py:277（速度剖面四层 min 的第 1 层）",
        "question": "退回安全档要付多少时间？换来多少精度？",
        "from_to": ("0.35", "0.15"),
        "watch": ["time_s", "cross_track_max_m", "out_of_bounds_steps"],
        "arms": [
            {"id": "A", "label": "默认 0.35", "set": [], "flags": []},
            {"id": "B", "label": "改 0.15", "set": ["pursuit.cruise_speed_mps=0.15"], "flags": []},
        ],
        "expect": "change",
        "expect_note": "慢下来时长必然涨；横向 MAX 通常降，但**不是线性** —— 这是本节的教学点。",
    },
    {
        "id": "lookahead_min",
        "title": "前视下限（零差异实验）",
        "group": "循迹参数",
        "key": "pursuit.lookahead_min_m",
        "now": "0.36   [dzrobot_v2/config/robot.yaml:625]",
        "schema": "默认 0.18，合法 0.05~1.00   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:90]",
        "consumed": "core/config.py:917-922（前视 = clamp(k_v·|v|, [min, max])；调用点 pursuit/controller.py:409）",
        "question": "把前视下限改小，为什么一点变化都没有？",
        "from_to": ("0.36", "0.18"),
        "watch": ["cross_track_rms_m", "cross_track_max_m", "time_s", "steer_rate_p95_deg_s"],
        "arms": [
            {"id": "A", "label": "默认 0.36", "set": [], "flags": []},
            {"id": "B", "label": "改 0.18", "set": ["pursuit.lookahead_min_m=0.18"], "flags": []},
        ],
        "expect": "same",
        "expect_note": (
            "当前 yaml 下 k_v×cruise = 1.2×0.35 = 0.42，已被 lookahead_max_m=0.40 钳住；"
            "min 只在 ld 低于它时才起作用 ⇒ 改 min 对结论零影响。"
            "这条是**故意做成的反例**：它证明「改了没反应」必须先去查钳位链，而不是下结论说参数没用。"
        ),
    },
    {
        "id": "lookahead_max",
        "title": "前视上限（真正决定前视的那个键）",
        "group": "循迹参数",
        "key": "pursuit.lookahead_max_m",
        "now": "0.40   [dzrobot_v2/config/robot.yaml:626]",
        "schema": "默认 0.45，合法 0.10~2.00   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:91]",
        "consumed": "core/config.py:917-922（clamp 的上界）；调用点 pursuit/controller.py:409-410",
        "question": "前视从 0.40 收到 0.22，车会更贴线还是更抖？",
        "from_to": ("0.40", "0.22"),
        "watch": ["cross_track_rms_m", "cross_track_max_m", "out_of_bounds_steps", "steer_rate_p95_deg_s"],
        "arms": [
            {"id": "A", "label": "默认 0.40", "set": [], "flags": []},
            {"id": "B", "label": "改 0.22", "set": ["pursuit.lookahead_max_m=0.22"], "flags": []},
        ],
        "expect": "change",
        "expect_note": "ld = clamp(0.42, [0.36, 0.22]) → 0.36（被 min 接住），前视实际 0.40→0.36。",
    },
    {
        "id": "max_steer_deg",
        "title": "舵角限幅",
        "group": "循迹参数",
        "key": "pursuit.max_steer_deg",
        "now": "30.0   [dzrobot_v2/config/robot.yaml:622]",
        "schema": "默认 30.0，合法 5.0~45.0   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:84]",
        "consumed": "pursuit/controller.py:428（每拍舵角裁剪）；tools/sim.py:499 同口径",
        "question": "舵角只给 20°，90° 弯过不去会表现成什么？",
        "from_to": ("30.0", "20.0"),
        "watch": ["cross_track_max_m", "cross_track_rms_m", "path_len_ratio", "out_of_bounds_steps"],
        "arms": [
            {"id": "A", "label": "默认 30.0", "set": [], "flags": []},
            {"id": "B", "label": "改 20.0", "set": ["pursuit.max_steer_deg=20.0"], "flags": []},
        ],
        "expect": "change",
        "expect_note": "限幅压小 ⇒ 同样的曲率需求算不出那么大的舵角 ⇒ 横向误差与顶点超调一起涨。本图实测更狠：B 臂判发散、跑不完全程 —— 【过不去弯】在仿真里就长这样，而不是【误差大一点】。",
    },
    {
        "id": "max_steer_rate",
        "title": "舵角速率限幅",
        "group": "循迹参数",
        "key": "pursuit.max_steer_rate_deg_s",
        "now": "120.0   [dzrobot_v2/config/robot.yaml:623]",
        "schema": "默认 120.0，合法 10.0~720.0   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:86]",
        "consumed": "pursuit/controller.py:440（指令侧）+ tools/sim.py:1057（执行侧斜坡）",
        "question": "把舵角速率砍到 40°/s，是变稳还是跟不上？",
        "from_to": ("120.0", "40.0"),
        "watch": ["cross_track_max_m", "steer_rate_p95_deg_s", "out_of_bounds_steps", "time_s"],
        "arms": [
            {"id": "A", "label": "默认 120.0", "set": [], "flags": []},
            {"id": "B", "label": "改 40.0", "set": ["pursuit.max_steer_rate_deg_s=40.0"], "flags": []},
        ],
        "expect": "change",
        "expect_note": "速率限幅同时作用于指令与执行侧；砍太狠 = 拐角追不上 = 峰值误差涨。",
    },
    {
        "id": "curvature_gain",
        "title": "纯追踪曲率增益",
        "group": "循迹参数",
        "key": "pursuit.curvature_gain",
        "now": "1.0   [dzrobot_v2/config/robot.yaml:624]",
        "schema": "默认 1.0，合法 0.2~2.0   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:88]",
        "consumed": "pursuit/controller.py:408（δ = gain·atan(2L·sinα/ld)）",
        "question": "这个键不是 PID 增益，改成 0.5 为什么误差会大这么多？",
        "from_to": ("1.0", "0.5"),
        "watch": ["cross_track_rms_m", "cross_track_max_m", "path_len_ratio"],
        "arms": [
            {"id": "A", "label": "默认 1.0", "set": [], "flags": []},
            {"id": "B", "label": "改 0.5", "set": ["pursuit.curvature_gain=0.5"], "flags": []},
        ],
        "expect": "change",
        "expect_note": "1.0 = 教科书几何解；改小等于整车转角只有应打的一半 ⇒ 系统性欠转（不是抖动）。本图实测 B 臂因此判发散（欠转把投影钉在顶点上）。",
    },
    {
        "id": "wheelbase",
        "title": "轴距（车体几何真值）",
        "group": "循迹参数",
        "key": "pursuit.wheelbase_m",
        "now": "0.157   [dzrobot_v2/config/robot.yaml:621]",
        "schema": "默认 0.157，合法 0.05~0.60   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:82]",
        "consumed": "pursuit/controller.py:407（曲率→舵角的几何分母）",
        "question": "轴距填错会怎样？它不是「标定误差」，而是控制器自己算错几何。",
        "from_to": ("0.157", "0.25"),
        "watch": ["cross_track_rms_m", "cross_track_max_m", "path_len_ratio", "anchors"],
        "arms": [
            {"id": "A", "label": "默认 0.157", "set": [], "flags": []},
            {"id": "B", "label": "改 0.25", "set": ["pursuit.wheelbase_m=0.25"], "flags": []},
        ],
        "expect": "change",
        "expect_note": "轴距进 atan(2L·sinα/ld) ⇒ 填大 59% 等于把每拍舵角一起放大 ⇒ 过转、切角、越界。本图实测 B 臂判发散：轴距填错不是【精度差一点】，是控制器按错的几何在开。",
    },
    {
        "id": "arrival",
        "title": "到位判定半径",
        "group": "循迹参数",
        "key": "pursuit.arrival_m",
        "now": "yaml 未写 → 取 SCHEMA 默认 0.12   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:172]",
        "schema": "默认 0.12，合法 0.03~0.35   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:172]",
        "consumed": "tools/sim.py:953（到位判定与吸附触发）",
        "question": "到点半径放大到 0.30 m，误差和越界会怎么变？",
        "from_to": ("0.12", "0.30"),
        "watch": ["cross_track_max_m", "final_error_m", "out_of_bounds_steps", "anchors"],
        "arms": [
            {"id": "A", "label": "默认 0.12", "set": [], "flags": []},
            {"id": "B", "label": "改 0.30", "set": ["pursuit.arrival_m=0.30"], "flags": []},
        ],
        "expect": "change",
        "expect_note": "半径放大 ⇒ 更早判「到了」 ⇒ 顶点超调变小但走的路更长、越界拍跟着变。",
    },
    {
        "id": "corner_speed",
        "title": "转角通过速度",
        "group": "循迹参数",
        "key": "pursuit.corner_speed_mps",
        "now": "yaml 未写 → 取 SCHEMA 默认 0.12   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:182]",
        "schema": "默认 0.12，合法 0.02~0.40   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:182]",
        "consumed": "pursuit/controller.py:292-293（转角减速窗口内取此速度）",
        "question": "拐角不再减速（0.12→0.30）能省多少时间？代价在哪？",
        "from_to": ("0.12", "0.30"),
        "watch": ["time_s", "cross_track_rms_m", "steer_rate_p95_deg_s", "out_of_bounds_steps"],
        "arms": [
            {"id": "A", "label": "默认 0.12", "set": [], "flags": []},
            {"id": "B", "label": "改 0.30", "set": ["pursuit.corner_speed_mps=0.30"], "flags": []},
        ],
        "expect": "change",
        "expect_note": "转角速度是「时间大户」：它会按 corner_slow_dist_m 的窗口逐拍覆盖 cruise。",
    },
    {
        "id": "lat_acc_max",
        "title": "横向加速度上限（曲率限速）",
        "group": "循迹参数",
        "key": "pursuit.lat_acc_max_mps2",
        "now": "0.35   [dzrobot_v2/config/robot.yaml:628]",
        "schema": "默认 0.35，合法 0.05~3.0   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:93]",
        "consumed": "pursuit/controller.py:574（v = sqrt(a_lat/|κ|)）与 core/config.py:925 曲率限速",
        "question": "把横向加速度上限压到 0.20，为什么越界拍反而变多？",
        "from_to": ("0.35", "0.20"),
        "watch": ["time_s", "out_of_bounds_steps", "cross_track_rms_m"],
        "arms": [
            {"id": "A", "label": "默认 0.35", "set": [], "flags": []},
            {"id": "B", "label": "改 0.20", "set": ["pursuit.lat_acc_max_mps2=0.20"], "flags": []},
        ],
        "expect": "change",
        "expect_note": "限速更低 ⇒ 弯里更慢更久；本图越界拍数与「慢」不是单调关系（要看轨迹）。",
    },
    # ------------------------------------------------------------------ 观测型
    {
        "id": "safety_margin",
        "title": "越界急停余量（只观测、不改控车）",
        "group": "观测型",
        "key": "pursuit.map_safety_margin_m",
        "now": "0.60   [dzrobot_v2/config/robot.yaml:637]",
        "schema": "默认 0.60，合法 0.0~1.5   [dzrobot_v2/src/dzbrain/dzbrain/core/config.py:190]",
        "consumed": "tools/sim.py:954（只用于越界判定统计；不参与控制）",
        "question": "把急停余量从 0.60 收到 0.30，车会跑得不一样吗？",
        "from_to": ("0.60", "0.30"),
        "watch": ["out_of_bounds_steps"],
        "arms": [
            {"id": "A", "label": "默认 0.60", "set": [], "flags": []},
            {"id": "B", "label": "改 0.30", "set": ["pursuit.map_safety_margin_m=0.30"], "flags": []},
        ],
        "expect": "obs_only",
        "expect_note": (
            "余量只进「越界拍」这条观测指标（sim.py 头注硬约束：只观测、不改动力学）⇒ "
            "轨迹 / 时长 / 横向误差必须**逐位不变**，只有越界拍数变。把这条做成实验，"
            "是为了让人亲眼看见「指标变了 ≠ 车行为变了」。"
        ),
    },
    # ------------------------------------------------------------------ 注入场景
    {
        "id": "mileage_no_anchor",
        "title": "里程标定偏大 + 关掉到位吸附",
        "group": "注入场景",
        "key": "sim.speed_scale_err + sim.anchor_on_arrival",
        "now": "零注入 + 锚定开（sim_cli 默认）   [dzrobot_v2/src/dzbrain/dzbrain/tools/sim_cli.py:79]",
        "schema": "--speed-scale-err 任意正负；--no-anchor 关闭吸附   [dzrobot_v2/src/dzbrain/dzbrain/tools/sim_cli.py:67]",
        "consumed": "tools/sim.py 头注：信念用 v·(1+ε) 积分、锚定把信念拽回格心（只改信念不改真值）",
        "question": "里程计多报 18% 又不做到位吸附，误差会飘到哪里去？",
        "from_to": ("0%（锚定开）", "+18%（锚定关）"),
        "watch": ["belief_error_rms_m", "belief_error_max_m", "cross_track_max_m", "final_error_m"],
        "arms": [
            {"id": "A", "label": "默认（ε=0，锚定开）", "set": [], "flags": []},
            {"id": "B", "label": "ε=+18% 且关锚定", "set": [], "flags": ["--speed-scale-err", "0.18", "--no-anchor"]},
        ],
        "expect": "change",
        "expect_note": "这只是**信念**误差（控制器看到的位姿 vs 真值），真值轨迹本身由执行量决定。本图实测 B 臂判发散：信念被 ε 一路拉偏 ⇒ 投影选错段 ⇒ 推进判据失效。",
    },
    {
        "id": "latency",
        "title": "链路延迟（指令→执行）",
        "group": "注入场景",
        "key": "sim.cmd_latency_s",
        "now": "0.06   [dzrobot_v2/src/dzbrain/dzbrain/tools/sim_cli.py:65]",
        "schema": "--latency 秒；现场链路估计 0.06，瞄准链路另算 0.26~0.27",
        "consumed": "tools/sim.py 头注：指令进延迟队列，队列空时保持上一条已生效指令（绝不回零）",
        "question": "延迟从 0.06 涨到 0.26 s，代价一定变大吗？",
        "from_to": ("0.06", "0.26"),
        "watch": ["cross_track_max_m", "time_s", "final_error_m", "cross_track_rms_m"],
        "arms": [
            {"id": "A", "label": "默认 0.06", "set": [], "flags": []},
            {"id": "B", "label": "改 0.26", "set": [], "flags": ["--latency", "0.26"]},
        ],
        "expect": "change",
        "expect_note": "延迟是一阶滞后。最容易犯的误读是【RMS 变好 = 延迟无害】：本图实测 B 臂只跑了几千拍就判发散，横向 RMS 变小纯粹是【跑得少】——报告里的【可比性】那行会拦住这个误读，别跳过它。",
    },
    {
        "id": "pos_noise",
        "title": "定位白噪声",
        "group": "注入场景",
        "key": "sim.pos_noise_m",
        "now": "0.0   [dzrobot_v2/src/dzbrain/dzbrain/tools/sim_cli.py:73]",
        "schema": "--pos-noise 米/拍；0 = 关",
        "consumed": "tools/sim.py 头注：每拍给信念位姿叠高斯噪声（毁掉零误差不变量的两个源之一）",
        "question": "定位抖 5 cm/拍，会把车甩出去吗？",
        "from_to": ("0.0", "0.05"),
        "watch": ["cross_track_rms_m", "cross_track_max_m", "steer_rate_p95_deg_s"],
        "arms": [
            {"id": "A", "label": "默认 0.0", "set": [], "flags": []},
            {"id": "B", "label": "改 0.05", "set": [], "flags": ["--pos-noise", "0.05"]},
        ],
        "expect": "change",
        "expect_note": "50 Hz 控制环 + 舵角速率限幅会把噪声低通掉大半 —— 但【大半】到底是多少必须实测。本图实测：噪声不是【抖动大一点】，B 臂判发散（噪声进了投影/前视，低通救不回来）。",
    },
]


def by_id(exp_id: str) -> dict:
    """按 id 取实验；取不到抛 KeyError（调用方负责转成人话）。"""
    for exp in EXPERIMENTS:
        if exp["id"] == exp_id:
            return exp
    raise KeyError(exp_id)


def ids() -> list[str]:
    """全部实验号（顺序与目录一致）。"""
    return [e["id"] for e in EXPERIMENTS]
