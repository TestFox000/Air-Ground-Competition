# -*- coding: utf-8 -*-
"""仓库目录索引生成器（团队共用）。
用法: python tools/wf/repo_index.py [--write]
索引 = 扫描（路径/大小/更新时间）+ 人工注解（docs/索引标注.yaml）。
新增/移动目录后：改 yaml 注解 -> 重跑 --write。
"""
from __future__ import annotations
import json, os, sys, time
from pathlib import Path

DZ = Path(__file__).resolve().parents[2]
WS = DZ.parent
ANN = DZ / "docs" / "索引标注.yaml"
OUT_MD = DZ / "docs" / "目录索引.md"
OUT_JSON = DZ / "docs" / "目录索引.json"

DEFAULT_ANN = {
    "dzrobot_v2": ("核心", "Lead", "车端决策层仓库（唯一真源）"),
    "dzrobot_v2/src": ("核心", "schema-owner", "车端源码；V1 旧树已删，归档 backup/archive_src_v1_20261018.tar.gz"),
    "dzrobot_v2/src/dzbrain": ("冻结", "schema-owner", "dzbrain 决策层；改动前登记 tools/wf/frozen.json in_flight"),
    "dzrobot_v2/src/dzrknn_dataprocess": ("冻结", "aim-tuner", "云台/视觉 C++ 包；冻结"),
    "dzrobot_v2/config": ("冻结", "Lead", "统一配置中心；cleanup 未动；与车端第七张图不同步（注意）"),
    "dzrobot_v2/config/map.yaml": ("冻结", "Lead", "本地=第六张图 D5+D6/K5+K7；车端=第七张图 D7+E7/K10+K4"),
    "dzrobot_v2/tools": ("核心", "团队", "route_check / sim_map_render / deploy_to_car / wf"),
    "dzrobot_v2/tools/deploy_to_car.py": ("冻结", "Lead", "唯一部署入口；冻结"),
    "dzrobot_v2/tools/wf/repo_index.py": ("核心", "团队", "本索引生成器"),
    "dzrobot_v2/docs": ("核心", "Lead", "决策台账/复用清单/工作流/目录索引/现状报告"),
    "dzrobot_v2/docs/目录索引.md": ("核心", "团队", "本文件（自动生成，勿手改）"),
    "dzrobot_v2/artifacts": ("证据", "团队", "仿真/实跑证据；stress 为当前基线"),
    "dzrobot_v2/backup": ("归档", "团队", "v3.9/v3.9.1/v3.9.2 bundle + src_v1 归档"),
    "dzrobot_v2/src/history/附加资源": ("活跃", "团队", "2.1GB 唯一素材副本，勿删"),
    "dzrobot_v2/_tmp_session": ("临时", "团队", "任务草稿；过期归档到 _archive"),
    "tools/plugins/dsh-dzrobot-ssh": ("活跃", "sim-dev", "车侧插件（S0 冻结）；2026-10-22 迁入"),
    "tools/plugins/dsh-dzdrone-ssh": ("活跃", "无人机", "机侧插件（S0 冻结）；2026-10-22 迁入"),
    "tools/plugins/dsh-dzairground-ssh": ("活跃", "sim-dev", "统一空地插件；2026-10-22 迁入"),
    "tools/plugins": ("工具", "团队", "DSH 插件源码区（三包按包名分目录）"),
    "_tmp_session": ("临时", "团队", "跨任务临时区；活跃目录见下表"),
    "_archive": ("归档", "团队", "已归档证据（reborn/TestFox 差异/旧配置）"),
    "dzsmartscout": ("仓库", "无人机", "无人机侧仓库"),
    "air_ground_models": ("素材", "无人机", "ONNX 模型"),
    "drone_recordings": ("证据", "无人机", "飞行记录"),
    "docs": ("文档", "Lead", "工作区级文档"),
    "tools": ("工具", "团队", "工作区级工具"),
}

def load_ann():
    ann = dict(DEFAULT_ANN)
    if ANN.exists():
        try:
            import yaml
            data = yaml.safe_load(ANN.read_text(encoding="utf-8")) or {}
            for k, v in (data.get("paths") or {}).items():
                if isinstance(v, dict):
                    ann[str(k)] = (str(v.get("status", "未登记")), str(v.get("owner", "")), str(v.get("note", "")))
                else:
                    ann[str(k)] = ("未登记", "", str(v))
        except Exception as exc:
            print("WARN 读取标注失败:", exc)
    return ann

def size_of(p):
    if p.is_file():
        return p.stat().st_size
    t = 0
    for r, ds, fs in os.walk(p):
        for f in fs:
            try:
                t += os.path.getsize(os.path.join(r, f))
            except OSError:
                pass
    return t

def fmt_size(n):
    if n >= 1 << 30: return "%.2f GB" % (n / (1 << 30))
    if n >= 1 << 20: return "%.1f MB" % (n / (1 << 20))
    if n >= 1 << 10: return "%.1f KB" % (n / (1 << 10))
    return "%d B" % n

def add(entries, rel, ann, force_status=None):
    p = WS / rel
    if not p.exists():
        return
    if rel in ann:
        st, owner, note = ann[rel]
    elif rel.startswith("dzrobot_v2/docs/"):
        st, owner, note = ("文档", "团队", "docs 文档；历史文档以文件名区分，现状看最新报告")
    elif rel.startswith("_tmp_session/") or rel.startswith("dzrobot_v2/_tmp_session/"):
        st, owner, note = ("临时", "团队", "未登记任务/临时目录；过期归档")
    else:
        st, owner, note = ("未登记", "", "请补 docs/索引标注.yaml")
    if force_status:
        st = force_status
    entries.append({"path": rel, "type": "dir" if p.is_dir() else "file",
                    "size": size_of(p),
                    "mtime": time.strftime("%Y-%m-%d %H:%M", time.localtime(p.stat().st_mtime)),
                    "status": st, "owner": owner, "note": note})

def main():
    write = "--write" in sys.argv
    ann = load_ann()
    entries = []
    for p in sorted(WS.iterdir(), key=lambda x: (not x.is_dir(), x.name.lower())):
        if p.name in (".git", ".agent-teams"): continue
        add(entries, p.name, ann)
    for p in sorted(DZ.iterdir(), key=lambda x: (not x.is_dir(), x.name.lower())):
        if p.name in (".git", ".agent-teams", "log"): continue
        add(entries, "dzrobot_v2/" + p.name, ann)
    src = DZ / "src"
    if src.is_dir():
        for p in sorted(src.iterdir(), key=lambda x: (not x.is_dir(), x.name.lower())):
            add(entries, "dzrobot_v2/src/" + p.name, ann,
                force_status=("冻结" if p.name in ("dzbrain", "dzrknn_dataprocess") else None))
    docs = DZ / "docs"
    if docs.is_dir():
        for p in sorted(docs.iterdir(), key=lambda x: x.name.lower()):
            add(entries, "dzrobot_v2/docs/" + p.name, ann)
    for base in (WS / "_tmp_session", DZ / "_tmp_session"):
        if not base.is_dir(): continue
        for p in sorted(base.iterdir(), key=lambda x: x.name.lower()):
            try:
                recent = time.time() - p.stat().st_mtime < 86400
            except OSError:
                recent = False
            if (p.is_dir() and size_of(p) > 1 << 20) or recent:
                add(entries, str(p.relative_to(WS)).replace("\\", "/"), ann)
    seen = set(); uniq = []
    for e in entries:
        if e["path"] in seen: continue
        seen.add(e["path"]); uniq.append(e)
    by = {e["path"]: e for e in uniq}
    def rows(paths):
        out = []
        for p in paths:
            e = by.get(p)
            if not e: continue
            out.append("| %s | %s | %s | %s | %s | %s |" % (e["path"], e["type"], fmt_size(e["size"]), e["status"], e["owner"], e["note"]))
        return out
    md = []
    md.append("# 仓库目录索引（团队共用 · 自动生成）\n")
    md.append("> 生成器：python tools/wf/repo_index.py --write · 注解源：docs/索引标注.yaml · 生成时间：%s\n" % time.strftime("%Y-%m-%d %H:%M"))
    md.append("> 用法：① 找文件/新建目录/新建工具前先查本索引 + docs/复用清单.md；② 新增/移动目录后改 docs/索引标注.yaml 并重跑 --write；③ 不要删别人的任务目录（_tmp_session/*），过期由 Lead 统一归档。\n")
    md.append("\n## 0. 三条真源（动手前必读）\n")
    md.append("| 真源 | 位置 | 什么时候读 |\n|---|---|---|\n")
    md.append("| 决策台账 | dzrobot_v2/docs/决策台账.md | 动手前；要重开已决事项先过翻案三问 |\n")
    md.append("| 复用清单 | dzrobot_v2/docs/复用清单.md | 新建脚本/模块/工具之前 |\n")
    md.append("| 协议细则 | dzrobot_v2/docs/工作流v2.md | 领卡/写卡/收口 |\n")
    md.append("| 目录索引 | dzrobot_v2/docs/目录索引.md | 找文件/找目录/建新目录 |\n")
    md.append("\n## 1. 工作区顶层\n")
    md.append("| 路径 | 类型 | 大小 | 状态 | Owner | 说明 |\n|---|---|---|---|---|---|\n")
    md.extend(rows([e["path"] for e in uniq if "/" not in e["path"]]))
    md.append("\n## 2. dzrobot_v2 顶层\n")
    md.append("| 路径 | 类型 | 大小 | 状态 | Owner | 说明 |\n|---|---|---|---|---|---|\n")
    md.extend(rows([e["path"] for e in uniq if e["path"].startswith("dzrobot_v2/") and e["path"].count("/") == 1]))
    md.append("\n## 3. src 模块\n")
    md.append("| 路径 | 类型 | 大小 | 状态 | Owner | 说明 |\n|---|---|---|---|---|---|\n")
    md.extend(rows([e["path"] for e in uniq if e["path"].startswith("dzrobot_v2/src/") and e["path"].count("/") == 2]))
    md.append("\n## 4. docs（技术文档）\n")
    md.append("| 路径 | 类型 | 大小 | 状态 | Owner | 说明 |\n|---|---|---|---|---|---|\n")
    md.extend(rows([e["path"] for e in uniq if e["path"].startswith("dzrobot_v2/docs/") and e["path"].count("/") == 2]))
    md.append("\n## 5. 活跃 _tmp_session / 归档\n")
    md.append("| 路径 | 类型 | 大小 | 状态 | Owner | 说明 |\n|---|---|---|---|---|---|\n")
    md.extend(rows([e["path"] for e in uniq if e["path"].startswith("_tmp_session/") or e["path"].startswith("dzrobot_v2/_tmp_session/") or e["path"].startswith("_archive/")]))
    md.append("\n## 6. 能力 → 权威实现（先查这里，别造轮子）\n")
    md.append("| 能力 | 权威实现 | 位置 | 说明/禁止 |\n|---|---|---|---|\n")
    md.append("| 路线规划（tour） | planner.py | src/dzbrain/dzbrain/route/planner.py | 唯一规划入口，禁另写规划器 |\n")
    md.append("| 路线合法性校验 | route_check.py | tools/route_check.py | 6 层，改路线必跑 |\n")
    md.append("| 离线执行仿真 | sim.py + sim_cli.py | src/dzbrain/dzbrain/tools/ | 禁另写仿真器 |\n")
    md.append("| 地图渲染 | sim_map_render.py | tools/sim_map_render.py | 唯一渲染器 |\n")
    md.append("| 参数扫描 | sim_param_sweep.py | tools/sim_param_sweep.py | 扫参走这里 |\n")
    md.append("| 部署上车 | deploy_to_car.py | tools/deploy_to_car.py | 唯一部署入口；冻结 |\n")
    md.append("| 门禁 | check.py | tools/wf/check.py | 收口必跑 |\n")
    md.append("| 目录索引 | repo_index.py | tools/wf/repo_index.py | 本索引生成器 |\n")
    md.append("| 全量回归 | 33 套 test_*.py | src/dzbrain/tests/ | 见 docs/回归守护_known_reds.md |\n")
    md.append("| 云台自整定 | autotune_pid.py | tools/aim_autotune/ | id/sweep/verify |\n")
    md.append("\n## 7. 冻结区（改前必须登记 frozen.json）\n")
    md.append("- src/dzbrain/dzbrain/** · src/dzrknn_dataprocess/** · config/** · tools/deploy_to_car.py\n")
    md.append("\n## 8. 新东西放哪里\n")
    md.append("| 产出 | 放哪里 |\n|---|---|\n")
    md.append("| 过程/草稿/实验数据 | _tmp_session/<卡号>/（≤7 天，过期归档到 _archive/<日期>/） |\n")
    md.append("| 正式结论/方案/报告 | docs/（技术）或 artifacts/（数据） |\n")
    md.append("| 会话日志 | src/history/log/log_YYYY-MM-DD_HHMM.md + 更新 INDEX.md |\n")
    md.append("| 新工具 | 先查复用清单；确需新建 → 放 tools/，登记复用清单 + 本索引 |\n")
    md.append("| 归档 | dzrobot_v2/backup/（版本/源码）或 _archive/（证据） |\n")
    md.append("\n## 9. 当前已知的不要再用\n")
    md.append("- tools/deploy_car.py（旧）→ 用 deploy_to_car.py\n")
    md.append("- tools/global_planner.py（旧全局规划器）→ 用 route/planner.py\n")
    md.append("- src_v1（V1 旧树）→ 已删除，归档 backup/archive_src_v1_20261018.tar.gz\n")
    md.append("- 旧 bundles（2026-10-09~v3.8）→ 已删除，保留 v3.9/v3.9.1/v3.9.2\n")
    md.append("\n## 10. 最近一次清理（2026-10-18）\n")
    md.append("- 工作区 6.41 GB → 3.81 GB（−2.60 GB）；提交 a43af2e（549 文件删除，仅清理）。\n")
    md.append("- V1 旧树归档：dzrobot_v2/backup/archive_src_v1_20261018.tar.gz（21.5 MB）。\n")
    md.append("- 详细清单/复核：dzrobot_v2/docs/清理记录_20261018.md；原始证据：_tmp_session/cleanup_20261018/。\n")
    text = "\n".join(md) + "\n"
    print(text[:5000])
    print("... 共 %d 条目" % len(uniq))
    if write:
        OUT_MD.write_text(text, encoding="utf-8")
        OUT_JSON.write_text(json.dumps(uniq, ensure_ascii=False, indent=1), encoding="utf-8")
        print("WROTE", OUT_MD)
        print("WROTE", OUT_JSON)
    return 0

if __name__ == "__main__":
    sys.exit(main())
