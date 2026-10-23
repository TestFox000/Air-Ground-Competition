# -*- coding: utf-8 -*-
"""跨平台 CJK 字体定位：所有渲染中文图的脚本共用这一份候选表。

为什么必须收敛成一处（2026-10-21 机上事故）：渲染脚本各自写死
`C:/Windows/Fonts/msyh.ttc`，在 Orin 上 `ImageFont.truetype` 抛错后各自落到
`ImageFont.load_default()`——那是 PIL 的位图字体，编码 latin-1，画第一个汉字
"墙" 就 UnicodeEncodeError。视频侦察"检测/规划全跑完、最后一步渲染崩掉"，
整份实物产物拿不到。候选表 + 降级判定放在这里，新增渲染脚本一律复用。

调用方拿到 `cjk_ok=False` 时必须把中文标签换成 ASCII——绝不拿默认位图字体
去画汉字（必崩），这是本模块的契约。
"""

from __future__ import annotations

from pathlib import Path

#: 顺序 = 优先无衬线，再衬线，最后兜底。JetPack（Orin）自带 Noto Sans CJK。
FONT_CANDIDATES = (
    "C:/Windows/Fonts/msyh.ttc",                                    # Windows 微软雅黑
    "C:/Windows/Fonts/simhei.ttf",                                  # Windows 黑体
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",       # Orin / JetPack
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",                 # 常见 Linux 中文字体
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/System/Library/Fonts/PingFang.ttc",                           # macOS
)


def load_cjk_font(size: int, candidates: tuple[str, ...] = FONT_CANDIDATES):
    """返回 (font, cjk_ok)；找不到任何 CJK 字体的机器上 cjk_ok=False。"""
    from PIL import ImageFont

    for path in candidates:
        try:
            if Path(path).is_file():
                return ImageFont.truetype(path, size), True
        except OSError:
            continue
    return ImageFont.load_default(), False
