#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""调参面板的【参数规范表】。

设计原则
--------------------------------------------------------------------------
1. **唯一真源仍是 config/robot.yaml** —— 这里只描述"哪些参数值得放在操作台上、
   它们的中文名/单位/安全范围是什么"。改完仍然写回 robot.yaml，
   不走任何旁路，车上读到的和界面上看到的永远是同一份。

2. 为什么不用自动枚举全部 60+ 个参数：
   绝大多数参数（刻度换算、滤波器内部量、协议字段）属于**标定常量**，
   调错了车会撞墙。操作台只暴露"现场真的会动"的那一撮，
   其余留给配置文件直接编辑。

3. `restart` 字段决定改完后重启哪个节点：
      gimbal / vision / camera  -> 安全，不影响正在跑的任务
      path                      -> ⚠️ 会重启循迹脚本，正在跑的任务会重新开始

    ★ 为什么不直接用 `ros2 param set` 热改：
      节点只在构造函数里 get_parameter 一次，之后不会重新读取
      （上次 scan_pitch_offset_deg 的教训就是这个坑的变种）。
      所以"写 YAML + 重启该节点"才是唯一可靠且行为可预期的路径。
"""

# 节点名 -> robot.yaml 里的段名
NODES = {
    "gimbal": "/dzrknn_dataprocess",
    "vision": "/det_node",
    "path": "path_executor",
}

# 每组：{"id", "name", "node", "restart", "note", "items":[...]}
# item: {"key","label","type":num|bool|str|list,"unit","min","max","step","dec","hint"}
PARAM_GROUPS = [
    # ------------------------------------------------------------------
    #  小车
    # ------------------------------------------------------------------
    {
        "id": "car", "name": "小车行进", "node": "path", "restart": "path",
        "note": "改动会重启循迹脚本；若车正在跑任务，任务会从起点重新开始",
        "items": [
            {"key": "linear_speed", "label": "行进速度", "type": "num",
             "unit": "m/s", "min": 0.05, "max": 0.60, "step": 0.01, "dec": 2,
             "hint": "0.15 约 0.5 格/秒；降速画面更稳、瞄准更容易"},
            {"key": "auto_start", "label": "自动出发", "type": "bool",
             "hint": "关掉后车原地待命，便于调试"},
            {"key": "auto_start_delay", "label": "出发延时", "type": "num",
             "unit": "s", "min": 0.0, "max": 30.0, "step": 0.5, "dec": 1,
             "hint": "给底盘/相机/云台留初始化时间"},
            {"key": "map_safety_margin", "label": "越界急停余量", "type": "num",
             "unit": "m", "min": 0.10, "max": 2.00, "step": 0.05, "dec": 2,
             "hint": "信念位姿超出地图边界这么多即急停"},
            {"key": "drift_warn_m", "label": "漂移告警阈值", "type": "num",
             "unit": "m", "min": 0.10, "max": 1.00, "step": 0.05, "dec": 2,
             "hint": "单段漂移超过即告警（到达判定阈值 0.157m）"},
            {"key": "mission_scan_mode", "label": "扫描模式", "type": "str",
             "choices": ["continuous", "waypoint", "none"],
             "hint": "continuous=边走边扫 / waypoint=点位停站 / none=不自动扫"},
        ],
    },
    # ------------------------------------------------------------------
    #  云台 —— 俯仰（★ 之前死活调不动的就是这一组）
    # ------------------------------------------------------------------
    {
        "id": "pitch", "name": "云台 · 俯仰", "node": "gimbal", "restart": "gimbal",
        "note": "★ 这就是之前改了不生效的那组参数，根因是 declare 后漏了 get_parameter，已修复",
        "items": [
            {"key": "scan_pitch_offset_deg", "label": "俯仰基准角", "type": "num",
             "unit": "°", "min": -45.0, "max": 15.0, "step": 1.0, "dec": 1,
             "hint": "负=向下压。数值越小画面越朝下；中心刻度=2047，1°≈11.4 刻度"},
            {"key": "tracking_pitch_margin_deg", "label": "俯仰修正余量", "type": "num",
             "unit": "°", "min": 0.0, "max": 30.0, "step": 1.0, "dec": 1,
             "hint": "伺服可在基准角上下各修这么多度；太小会顶死修正（曾导致俯仰僵住）"},
        ],
    },
    # ------------------------------------------------------------------
    #  云台 —— 左右转范围
    # ------------------------------------------------------------------
    {
        "id": "yaw", "name": "云台 · 左右转范围", "node": "gimbal", "restart": "gimbal",
        "items": [
            {"key": "scan_yaw_min_deg", "label": "左转最大角", "type": "num",
             "unit": "°", "min": -180.0, "max": 0.0, "step": 5.0, "dec": 1,
             "hint": "负值。扫描从这头扫到那头"},
            {"key": "scan_yaw_max_deg", "label": "右转最大角", "type": "num",
             "unit": "°", "min": 0.0, "max": 180.0, "step": 5.0, "dec": 1,
             "hint": "正值。两个角决定扫描行程，也决定云台的软限位"},
            {"key": "scan_passes", "label": "每层往返趟数", "type": "num",
             "unit": "趟", "min": 1, "max": 6, "step": 1, "dec": 0,
             "hint": "扫满 N 趟换下一俯仰层"},
        ],
    },
    # ------------------------------------------------------------------
    #  云台 —— 速度
    # ------------------------------------------------------------------
    {
        "id": "speed", "name": "云台 · 速度", "node": "gimbal", "restart": "gimbal",
        "items": [
            {"key": "patrol_speed_ratio", "label": "扫描速度比例", "type": "num",
             "unit": "×最大", "min": 0.05, "max": 1.00, "step": 0.01, "dec": 2,
             "hint": "慢一点更容易在扫过时锁住目标"},
            {"key": "track_speed_max", "label": "跟踪速度上限", "type": "num",
             "unit": "刻度", "min": 0, "max": 3700, "step": 5, "dec": 0,
             "hint": "0 = 用下面的比例自动算"},
            {"key": "track_speed_max_ratio", "label": "跟踪上限比例", "type": "num",
             "unit": "×扫描", "min": 0.10, "max": 1.50, "step": 0.01, "dec": 2,
             "hint": "仅当上面为 0 时生效"},
            {"key": "idle_homing_speed_ratio", "label": "空闲归位速度", "type": "num",
             "unit": "×扫描", "min": 0.1, "max": 1.0, "step": 0.05, "dec": 2,
             "hint": "空闲时云台回到正前方+基准俯仰的速度"},
        ],
    },
    # ------------------------------------------------------------------
    #  瞄准跟踪
    # ------------------------------------------------------------------
    {
        "id": "track", "name": "瞄准跟踪", "node": "gimbal", "restart": "gimbal",
        "items": [
            {"key": "track_kp", "label": "比例增益 Kp", "type": "num",
             "unit": "", "min": 0.5, "max": 30.0, "step": 0.5, "dec": 1,
             "hint": "大=反应快但易过冲抖动"},
            {"key": "track_kd", "label": "微分增益 Kd", "type": "num",
             "unit": "", "min": 0.0, "max": 20.0, "step": 0.5, "dec": 1,
             "hint": "大=阻尼强更稳但略迟钝"},
            {"key": "ema_alpha", "label": "滤波强度", "type": "num",
             "unit": "", "min": 0.05, "max": 1.00, "step": 0.05, "dec": 2,
             "hint": "小=更平滑但延迟大；大=更跟手但抖"},
            {"key": "track_deadzone_px", "label": "跟踪死区", "type": "num",
             "unit": "px", "min": 0, "max": 40, "step": 1, "dec": 0,
             "hint": "偏差小于它就不动，防抖；必须小于开火框"},
            {"key": "predict_gain", "label": "移动预判增益", "type": "num",
             "unit": "", "min": 0.0, "max": 3.0, "step": 0.1, "dec": 1,
             "hint": "补偿视觉到下位机的链路延迟"},
            {"key": "gyro_ff_gain", "label": "陀螺前馈增益", "type": "num",
             "unit": "", "min": 0.0, "max": 2.0, "step": 0.05, "dec": 2,
             "hint": "0=关闭。抵消车体转弯带来的瞄准漂移"},
        ],
    },
    # ------------------------------------------------------------------
    #  开火
    # ------------------------------------------------------------------
    {
        "id": "fire", "name": "开火判定", "node": "gimbal", "restart": "gimbal",
        "items": [
            {"key": "fire_only_blue", "label": "仅打蓝靶", "type": "bool",
             "hint": "红靶仍会跟踪显示，但不消耗弹药"},
            {"key": "max_shots", "label": "全场弹药上限", "type": "num",
             "unit": "发", "min": 0, "max": 50, "step": 1, "dec": 0,
             "hint": "0 = 不限"},
            {"key": "max_shots_per_target", "label": "单目标连发上限", "type": "num",
             "unit": "发", "min": 0, "max": 20, "step": 1, "dec": 0,
             "hint": "0 = 不限。防止在一个靶上打光弹药"},
            {"key": "shot_cooldown_ms", "label": "开火冷却", "type": "num",
             "unit": "ms", "min": 50, "max": 3000, "step": 50, "dec": 0,
             "hint": "两发之间的最小间隔"},
            {"key": "fire_deadzone_px", "label": "开火框(静态)", "type": "num",
             "unit": "px", "min": 2, "max": 80, "step": 1, "dec": 0,
             "hint": "靶心进入这个框才开火；动态模式时作兜底"},
            {"key": "target_confirm_frames", "label": "目标确认帧数", "type": "num",
             "unit": "帧", "min": 1, "max": 30, "step": 1, "dec": 0,
             "hint": "连续识别到这么多帧才允许开火，防误触发"},
            {"key": "cls_switch_confirm_frames", "label": "类别改锁帧数", "type": "num",
             "unit": "帧", "min": 1, "max": 20, "step": 1, "dec": 0,
             "hint": "蓝/红判定抖动时的抗抖帧数"},
        ],
    },
    # ------------------------------------------------------------------
    #  视觉
    # ------------------------------------------------------------------
    {
        "id": "vision", "name": "视觉检测", "node": "vision", "restart": "vision",
        "note": "推理周期在启动时确定，改帧率需重启视觉节点",
        "items": [
            {"key": "draw", "label": "弹出识别窗口", "type": "bool",
             "hint": "关掉可省 CPU（30Hz 的 imshow 在板子上不便宜）"},
            {"key": "max_fps", "label": "推理帧率", "type": "num",
             "unit": "fps", "min": 5.0, "max": 30.0, "step": 1.0, "dec": 0,
             "hint": "相机上限 30；降到 20 明显省 CPU，代价是略延迟"},
            {"key": "thread_num", "label": "推理线程数", "type": "num",
             "unit": "线程", "min": 1, "max": 8, "step": 1, "dec": 0,
             "hint": "CPU 吃紧可降到 2"},
            {"key": "hot_reload", "label": "模型热加载", "type": "bool",
             "hint": "模型文件更新后 1 秒内自动重载，不用重启节点"},
        ],
    },
    # ------------------------------------------------------------------
    #  任务路线
    # ------------------------------------------------------------------
    {
        "id": "route", "name": "任务路线", "node": "path", "restart": "path",
        "note": "改动会重启循迹脚本；正在跑的任务会从起点重新开始",
        "items": [
            {"key": "mission_blocks", "label": "航点序列", "type": "list",
             "hint": "空格或逗号分隔。后缀 * = 射击点；前缀 R: = 倒车；"
                     "后缀 ,x/y = 终点朝向"},
            {"key": "mission_reset", "label": "起点重锚", "type": "str",
             "hint": "如 M10,y —— 起跑前把位姿锚定到该格与该朝向"},
        ],
    },
]


def iter_items():
    """展开成 (group, item) 便于后端遍历。"""
    for g in PARAM_GROUPS:
        for it in g["items"]:
            yield g, it


def find_item(node_section: str, key: str):
    """按 robot.yaml 的段名 + 参数键反查规范项。"""
    for g in PARAM_GROUPS:
        if NODES.get(g["node"]) != node_section:
            continue
        for it in g["items"]:
            if it["key"] == key:
                return g, it
    return None, None


def node_of_group(gid: str):
    for g in PARAM_GROUPS:
        if g["id"] == gid:
            return NODES.get(g["node"]), g.get("restart")
    return None, None
