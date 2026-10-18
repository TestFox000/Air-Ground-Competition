# -*- coding: utf-8 -*-
"""事物标签体系 —— "驯化"的第一步: 把代码里的具体事物建模成标签.

每个标签 = 一个领域事物, 附一组代码判别特征(关键词规则), 供 build_dataset.py
自动打标; 真正训练时的判别能力由模型从样本里学, 而不是靠这些规则.

用法:
    python entity_labels.py            # 打印标签体系 + 演示打标
    from entity_labels import label_text, CLASSES
"""
import re
import sys

if sys.stdout and hasattr(sys.stdout, "reconfigure"):   # Windows 控制台 UTF-8
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# 标签全集: 水桶/爆炸物/敌我打击区/桥洞/绿线 + other(负类, 非常重要)
CLASSES = [
    "bucket",        # 水桶(散落障碍物, 直径26cm/高25cm, 占一格禁入)
    "explosive",     # 爆炸物(地雷, 位于格心, 必须压过正中心全踩排雷)
    "enemy_strike",  # 敌打击区(带蓝面, 必须激光打击3发)
    "friend_strike", # 我方打击区(无蓝面, 不打, 实体禁入绕行)
    "strike_zone",   # 打击区通用/敌我判别逻辑(不区分敌我)
    "bridge",        # 桥洞(K1~K11, 高35cm, 判断可通行通道)
    "green_line",    # 绿线(禁入线, 开局已知)
    "other",         # 无关代码(负样本, 教模型学会说"都不是")
]

# 每类事物的代码判别特征(正则: 英文按词首边界, 中文按子串)
RULES = {
    "bucket": [
        r"\bBUCKET", r"\bOBSTACLE",           # BUCKET / BUCKETS_N / OBSTACLE_D_CM
        "水桶", "障碍物",
    ],
    "explosive": [
        r"\bEXPLOSIVE", r"\bexplosive",       # EXPLOSIVES_N / role=="explosive"
        r"\bmine", r"\bMINE",                 # MINE_ZONE / mine_zone
        "地雷", "排雷", "爆炸物", "压过正中心",
    ],
    "enemy_strike": [
        r"\bENEMY", r"\bstop_face", r"\bhit\b",
        "敌方", "蓝面", "激光打击", "激光",
    ],
    "friend_strike": [
        r"\bFRIEND",                          # FRIEND / new_friends
        "我方", "无蓝面",
    ],
    "strike_zone": [
        r"\bTARGET", r"\bstrike",             # TARGET / TARGETS_N / strike_cell
        "打击区", "标靶",
    ],
    "bridge": [
        r"\bBRIDGE", "桥洞",
    ],
    "green_line": [
        r"\bGREEN_LINE", r"\bLINE\b", "绿线",
    ],
}

_COMPILED = {k: [re.compile(p) for p in pats] for k, pats in RULES.items()}


def hits_of(text):
    """统计每类特征在文本中的命中次数."""
    return {k: sum(len(p.findall(text)) for p in pats)
            for k, pats in _COMPILED.items()}


def label_text(text):
    """规则打标: 返回 (标签, 命中统计).

    取命中特征最多的一类; 平票按 RULES 定义顺序优先
    (bucket > explosive > enemy_strike > ... ); 全零 -> other。
    注: 这是自动初标, 训练前建议人工抽查校正(见 README_驯化指南.md)。
    """
    h = hits_of(text)
    best = max(h, key=h.get)
    return (best, h) if h[best] > 0 else ("other", h)


if __name__ == "__main__":
    print("=" * 62)
    print("事物标签体系 (entity_train/entity_labels.py)")
    print("=" * 62)
    for i, c in enumerate(CLASSES):
        print(f"  [{i}] {c}")
    print("\n特征规则:")
    for k, pats in RULES.items():
        print(f"  {k:14s} -> {', '.join(pats)}")

    print("\n" + "-" * 62)
    print("演示打标:")
    print("-" * 62)
    demos = [
        "BUCKETS_N = 6   # 散落障碍物(直径26cm/高25cm, 占一格)",
        "def explosive_order(self, start, explosives, stop_cell):",
        "s = self.strike_cell(cur, t)   # 敌方打击区(带蓝面), 激光打击3次",
        "kind = FRIEND   # 无蓝面 -> 我方打击区, 不打",
        "self.bridge = {name2rc(n) for n in BRIDGE_CELLS}   # 桥洞 K1~K11",
        "for rc in line_cells(*GREEN_LINE): self.set(rc, LINE)   # 绿线禁入",
        "def man(a, b): return abs(a[0]-a[1]) + abs(b[0]-b[1])",
        "敌方 {rc2name(t)} 停靠蓝面正对格, 车头对准蓝面激光打击",
    ]
    for t in demos:
        lab, h = label_text(t)
        hot = [f"{k}:{v}" for k, v in h.items() if v]
        print(f"  -> {lab:14s} | {t[:48]:48s} | {', '.join(hot) or '无命中'}")
