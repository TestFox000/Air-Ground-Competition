"""世界模型：把解析出的 dzuav 目标按类别汇总为一帧场地态势。

输入 intel_protocol.IntelFrame（世界米制），输出不可变 WorldSnapshot：
    mines     地雷所在格
    passable  可通行桥洞格（K 列 paper）
    obstacles 障碍格
    targets   敌我靶（格 + 蓝/红面）
CAR / CELL_LABEL / BUCKET_* / 起止点只用于机上定位与展示，不进入态势。

合并说明（2026-10-17 评审整改）：
  本文件是两条分叉的合并结果——
    · 分类逻辑取自「上机包」分支：D 方案 13 类模型（2026-10-16 切换），
      paper 语义由落格列判定，与机上 main.cpp className 顺序一致。
    · 障碍多帧确认 + TargetRef 三态取自「路径规划交接」分支。
  交接分支的 world_model 用的是切换前的 7 类枚举（MINE/ARROW），
  直接采用会把 cell_label 当车、把 obstacle 当蓝靶，故必须这样合并。
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from .field_frame import CellRef, world_to_cell
from .intel_protocol import IntelFrame, ObjectType


@dataclass(frozen=True)
class TargetRef:
    cell: CellRef          # 检测落点（移动靶=该帧瞬时位置）
    is_blue: bool | None   # True=敌方蓝面可见 False=友方红面可见 None=发现态（红蓝未辨）
    # 移动靶专用（全场仅一个）：track=轨道两端格，blue_face=蓝面朝向。
    # 轨道水平（两端同行）时 blue_face='S'(朝南)/'N'；垂直时 'E'/'W'。
    # 轨道两端由裁判系统地图确定（每局固定），蓝面朝向由无人机识别/现场标定。
    track: tuple[CellRef, CellRef] | None = None
    blue_face: str | None = None


@dataclass(frozen=True)
class WorldSnapshot:
    timestamp_ms: int
    mines: tuple[CellRef, ...]
    passable: tuple[CellRef, ...]
    obstacles: tuple[CellRef, ...]
    targets: tuple[TargetRef, ...]

    @property
    def has_enemy(self) -> bool:
        return any(t.is_blue for t in self.targets)


class WorldModel:
    """累积/替换最新态势。dzuav 每帧是全场结果，因此采用整帧替换。

    红靶粘性否决（禁火铁律 fail-safe）：红面（友方）靶单帧漏检/遮挡/
    落格残差过大被丢弃的那一帧，若恰好触发规划就会误击友方。因此见过
    红靶的格子在 red_veto_ttl_s 秒内持续以红靶身份留在态势里参与否决
    ——粘性只增加否决，绝不减少。2026-10-25 起武装前需 red_confirm_frames
    帧确认（默认 2）：单帧幻影红检不再触发否决，真红靶晚 N-1 帧武装。

    ⚠️ TTL 时基（2026-10-17 修订，原为 frame.timestamp_ms）：
    机载 main.cpp 用 system_clock 打时间戳，而项目已实测「整机重启后
    系统时钟回跳（无 NTP）」。用对端墙钟计量 TTL 有两个失效方向：
      · 机载钟前跳 > TTL → 所有红靶否决一次性集体过期 → 可能误击友方
      · 乱序旧帧把 now_s 拨回 → 否决永不过期
    故改用**本地 monotonic 收包时刻**。update() 的 recv_mono 参数应由
    收包线程在 recvfrom 返回时立即取 time.monotonic() 传入；不传则在
    update() 内取，仍优于对端墙钟。

    障碍多帧确认（obstacle_confirm_frames）：障碍格连续 N 帧检出才进入
    态势、连续 N 帧消失才移出（对称滞回）。N=1 精确退化为旧行为。
    动机（docs/分区规划与感知约束-20261015 问题1 ⚠️）：交界格（走廊口/
    桥洞）的障碍误检会堵死唯一通道，整区必经点被降级跳过——单帧幽灵
    必须先被多帧否决。真障碍只晚 N-1 帧（5fps 下 0.2s）进入，代价可忽略。
    2026-10-17 默认 N=1→2：实拍复盘确认单帧幻影障碍触发无效重规划的
    代价远大于 0.2s 确认延迟（车载版三帧共识）。
    """

    def __init__(self, red_veto_ttl_s: float = 5.0,
                 obstacle_confirm_frames: int = 2,
                 red_confirm_frames: int = 2) -> None:
        self._snapshot: WorldSnapshot | None = None
        self.red_veto_ttl_s = float(red_veto_ttl_s)
        self.obstacle_confirm_frames = max(1, int(obstacle_confirm_frames))
        self.red_confirm_frames = max(1, int(red_confirm_frames))
        # 红靶格 -> 最后一次见到的时刻（本地 monotonic 秒）
        self._red_seen: dict[CellRef, float] = {}
        # 候选红靶格 -> (最后一次见到时刻, TTL 内累计帧数)，确认后才进 _red_seen
        self._red_cand: dict[CellRef, tuple[float, int]] = {}
        # 障碍多帧确认状态：格名 -> (CellRef, 连续检出计, 连续漏检计)
        self._ob_state: dict[str, tuple[CellRef, int, int]] = {}

    @property
    def snapshot(self) -> WorldSnapshot | None:
        return self._snapshot

    @property
    def ready(self) -> bool:
        return self._snapshot is not None

    def veto_active(self, cell: CellRef, now_mono: float | None = None) -> bool:
        """某格的红靶否决当前是否仍然有效（供 fire_guard / 测试查询）。"""
        ts = self._red_seen.get(cell)
        if ts is None:
            return False
        now = time.monotonic() if now_mono is None else now_mono
        return (now - ts) <= self.red_veto_ttl_s

    def update(self, frame: IntelFrame,
               recv_mono: float | None = None) -> WorldSnapshot:
        now_mono = time.monotonic() if recv_mono is None else float(recv_mono)

        mines: dict[str, CellRef] = {}
        passable: dict[str, CellRef] = {}
        obstacles: dict[str, CellRef] = {}
        targets: list[TargetRef] = []

        for obj in frame.objects:
            try:
                cell = world_to_cell(obj.x, obj.y)
            except ValueError:
                # 出界/无法吸附的点不纳入态势（CAR/CELL_LABEL 也走这条忽略）
                continue

            if obj.type is ObjectType.PAPER:
                # D 方案（2026-10-16）：paper 语义由落格判定——
                # K 列桥洞区的纸片 = 通行箭头（每段桥洞有且只有一个）；
                # 其余格的纸片 = 雷（官方规则：雷即 12cm 图片粘贴）。
                if cell.name[:1] == "K":
                    passable.setdefault(cell.name, cell)
                else:
                    mines.setdefault(cell.name, cell)
            elif obj.type is ObjectType.OBSTACLE:
                obstacles.setdefault(cell.name, cell)
            elif obj.type is ObjectType.TARGET_BLUE:
                targets.append(TargetRef(cell, True))
            elif obj.type is ObjectType.TARGET_RED:
                targets.append(TargetRef(cell, False))
            # CAR / CELL_LABEL / BUCKET_* / 起止点：不进态势
            # TARGET（ft_v1 整体板）同样不进：红/蓝面才携带禁火/必打语义，
            # 整体板只是存在性证据，单独造实体会和面框的 TargetRef 重复计格。

        # 蓝靶去重（2026-10-17 修订）：同帧对同一蓝靶双检出会产生两个
        # TargetRef → 规划侧生成两组射击位候选，组合数翻倍且两个停靠格
        # 都成必经点，白绕路。mines/passable/obstacles 一直按格名去重，
        # 蓝靶此前是唯一的例外。红靶不去重：粘性否决按格记录，重复无害。
        seen_blue: set[str] = set()
        deduped: list[TargetRef] = []
        for t in targets:
            if t.is_blue:
                if t.cell.name in seen_blue:
                    continue
                seen_blue.add(t.cell.name)
            deduped.append(t)
        targets = deduped

        # 红靶粘性否决：先过 red_confirm_frames 多帧确认再武装（2026-10-25）。
        # hbb_d13 实测存在单帧幻影红检（conf 0.3~0.7 的小框、与真靶不同位、
        # 帧间不重复出现），单帧即武装 5s 禁火会恰好压住蓝靶停靠窗 → 整场
        # 拒射。真红靶在视野内连续多帧出现（5fps 下 0.2s 内确认），代价只晚
        # N-1 帧；N=1 精确退化为旧行为。TTL 内同格累计达 N 帧才登记进
        # _red_seen；未确认的候选本帧不上报、不参与否决。
        red_cells = {t.cell for t in targets if t.is_blue is False}
        for c in red_cells:
            ts, hits = self._red_cand.get(c, (now_mono, 0))
            hits = hits + 1 if now_mono - ts <= self.red_veto_ttl_s else 1
            self._red_cand[c] = (now_mono, hits)
            if hits >= self.red_confirm_frames:
                self._red_seen[c] = now_mono
        targets = [t for t in targets if t.is_blue or t.cell in self._red_seen]
        # 武装过的红靶格：TTL 内见过但本帧消失的补回态势（fail-safe：
        # 只增加否决），过期的清出历史表；候选表同步清理陈旧项。
        expired = []
        for c, ts in self._red_seen.items():
            if now_mono - ts > self.red_veto_ttl_s:
                expired.append(c)
            elif c not in red_cells:
                targets.append(TargetRef(c, False))
        for c in expired:
            del self._red_seen[c]
        for c in [k for k, (ts, _) in self._red_cand.items()
                  if now_mono - ts > self.red_veto_ttl_s]:
            del self._red_cand[c]

        # 障碍多帧确认（对称滞回）：连续 N 帧检出进入态势、连续 N 帧消失移出。
        # N=1 精确退化为旧行为（本帧检出即在态势）。
        n = self.obstacle_confirm_frames
        if n <= 1:
            confirmed_obs = list(obstacles.values())
        else:
            confirmed_obs = []
            for name, cell in obstacles.items():
                st = self._ob_state.get(name)
                seen = (st[1] + 1) if st else 1
                self._ob_state[name] = (cell, seen, 0)
                if seen >= n:
                    confirmed_obs.append(cell)
            for name in list(self._ob_state):
                if name in obstacles:
                    continue
                cell, seen, miss = self._ob_state[name]
                miss += 1
                if miss >= n:
                    del self._ob_state[name]   # 连续 N 帧未检出：彻底移出
                else:
                    self._ob_state[name] = (cell, seen, miss)
                    if seen >= n:
                        confirmed_obs.append(cell)   # 滞回期：已确认障碍保持

        self._snapshot = WorldSnapshot(
            frame.timestamp_ms,
            tuple(mines.values()),
            tuple(passable.values()),
            tuple(confirmed_obs),
            tuple(targets),
        )
        return self._snapshot
