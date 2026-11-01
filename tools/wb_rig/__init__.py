# -*- coding: utf-8 -*-
"""wb-2 循迹台架子包（卡 `WB-2-RIG-20261031`）。

本包只放**编排件**，`tools/sim_world.py` 放世界模型，`tools/sim_rig.py` 放 CLI。

★ 分层铁律（硬判据①，由 `test_wb2_no_second_projection.py` 机器判）：
  本包与 `sim_world`/`sim_rig` 内**不许出现第二份投影/推进实现**。
  · 投影：唯一真源 = `dzbrain.pursuit.path.Path.project`（生产 `LocalizationTask` /
    `TrackerTask` 用的同一个类对象，不是"同值不同码"）。
  · 推进：唯一真源 = `dzbrain.tools.sim.step_bicycle`（生产仿真与下位机同构的积分器）。
  · 本包只做**调度、注入、执行器链、指标分级**四件事。

| 模块 | 职责 | 不做什么 |
|---|---|---|
| `scenarios` | 具名场景 → 夹具路径（读 `fixtures/route_sources.json`） | 不回落 `config/map.yaml`（D-054/D-062） |
| `arms` | `ideal` / `realistic` 两档注入档案 | 不读配置、不改配置 |
| `actuator` | 执行器链四项（servo / 平滑窗 / 量化 / 看门狗） | 不做运动学积分 |
| `sched` | 虚拟时钟 + 定序调度（真线程在位） | 不调 `tick()` 之外的任何任务内部 |
| `counters` | 从**生产任务的公开计数**取台账 | 不解析日志文本 |
| `metrics` | 指标表 + `transfer` 分级 + `field_modeled` 置灰 + 契约行 | 不重算物理 |
"""

from __future__ import annotations

__all__ = ["arms", "actuator", "counters", "metrics", "scenarios", "sched"]
