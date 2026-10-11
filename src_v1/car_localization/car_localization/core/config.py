"""core/config.py — 参数配置（Config 封装）。

从 path_executor.py V1 拆出：
  - V1 __init__ 中硬编码字面量（L239-254）统一到 SCHEMA
  - Config.get(key) 接口，缺键回退 SCHEMA 默认值
  - 无外部文件依赖（不引入 robot.yaml 加载，V1 也没有）
  - 不改变 V1 默认值语义

Config 是一个简单的字典包装器，带 Spec 校验和默认值回退。
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


# ── Spec 描述 ───────────────────────────────────────────────

@dataclass(frozen=True)
class Spec:
    """单个配置项的描述。"""
    default: Any           # 默认值
    kind: str = 'float'    # 类型提示
    unit: str = ''         # 单位
    lo: Optional[float] = None    # 下限
    hi: Optional[float] = None    # 上限
    desc: str = ''         # 中文说明
    critical: bool = False # 关键参数（启动期必须校验）

    def check(self, value: Any) -> Optional[str]:
        """校验值是否合法；返回错误说明或 None。"""
        if self.kind == 'float':
            try:
                v = float(value)
            except (ValueError, TypeError):
                return f"期望 float，得到 {type(value).__name__}"
            if self.lo is not None and v < self.lo:
                return f"值 {v} < 下限 {self.lo}"
            if self.hi is not None and v > self.hi:
                return f"值 {v} > 上限 {self.hi}"
        elif self.kind == 'int':
            try:
                v = int(value)
            except (ValueError, TypeError):
                return f"期望 int，得到 {type(value).__name__}"
            if self.lo is not None and v < self.lo:
                return f"值 {v} < 下限 {self.lo}"
            if self.hi is not None and v > self.hi:
                return f"值 {v} > 上限 {self.hi}"
        elif self.kind == 'bool':
            if not isinstance(value, bool):
                return f"期望 bool，得到 {type(value).__name__}"
        elif self.kind == 'str':
            if not isinstance(value, str):
                return f"期望 str，得到 {type(value).__name__}"
        return None


# ── Schema ──────────────────────────────────────────────────

# V1 _CFG 字典：所有 V1 初始化时硬编码的参数值。
# 这里的默认值与 V1 __init__ 逐位一致。
SCHEMA: Dict[str, Spec] = {
    # navigation (V1 L239-254)
    'pursuit.lookahead_distance':    Spec(default=0.15, kind='float', unit='m', lo=0.01, hi=5.0, desc='前视距离', critical=True),
    'pursuit.cruise_speed_mps':      Spec(default=0.3, kind='float', unit='m/s', lo=0.01, hi=2.0, desc='巡航线速度', critical=True),
    'pursuit.max_angular_speed':     Spec(default=1.5, kind='float', unit='rad/s', lo=0.0, hi=10.0, desc='最大角速度(兼容保留)'),
    'pursuit.wheelbase':             Spec(default=0.157, kind='float', unit='m', lo=0.05, hi=0.5, desc='轴距 L', critical=True),
    'pursuit.arrival_threshold':     Spec(default=0.10 + (3.141592653589793 * 0.073 / 4.0), kind='float', unit='m', lo=0.01, hi=1.0, desc='到达判定阈值', critical=True),
    'pursuit.approach_dist':         Spec(default=0.6, kind='float', unit='m', lo=0.0, hi=10.0, desc='末端接近段长度'),
    'pursuit.max_steer_deg':         Spec(default=30.0, kind='float', unit='deg', lo=0.0, hi=90.0, desc='舵角硬限幅'),

    # shoot stop (V1 L264-266)
    'mission.shots_per_stop':        Spec(default=10, kind='int', desc='每停射击弹数', critical=True),
    'mission.shoot_timeout_s':       Spec(default=150.0, kind='float', unit='s', lo=0.0, desc='射击等待超时'),
    'mission.shoot_stop_radius':     Spec(default=0.17, kind='float', unit='m', lo=0.0, desc='射击停站判定半径'),

    # scan stop (V1 L278-279)
    'mission.scan_wait_timeout_s':   Spec(default=8.0, kind='float', unit='s', lo=0.0, desc='扫描等待超时'),
    'mission.scan_stop_radius':      Spec(default=0.20, kind='float', unit='m', lo=0.0, desc='扫描停站判定半径'),
    'mission.scan_mode':             Spec(default='continuous', kind='str', desc='任务扫描模式'),

    # pivot (V1 内联常量)
    'pivot.corner_angle_deg':        Spec(default=45.0, kind='float', unit='deg', desc='原地转弯最小角'),
    'pivot.creep_speed_mps':         Spec(default=0.08, kind='float', unit='m/s', desc='蠕动速度'),
    'pivot.settle_time_s':           Spec(default=0.5, kind='float', unit='s', desc='舵机就位等待时间'),
    'pivot.align_tolerance_deg':     Spec(default=15.0, kind='float', unit='deg', desc='航向对齐容差'),
}


class ConfigError(Exception):
    """配置相关异常。"""
    pass


@dataclass
class Issue:
    """配置校验发现的问题。"""
    key: str
    message: str
    severity: str = 'error'  # 'error' | 'warning'


class Config:
    """配置存取器：点号式键访问 + 默认值回退 + 可选校验。

    V1 中参数要么是 __init__ 硬编码字面量（_CFG 字典），要么通过
    _load_config_center 加载。本 Config 统一为：先读传参 flat dict，
    缺键回退 SCHEMA 默认值。
    """

    __slots__ = ('_data', '_source')

    def __init__(self, flat: Dict[str, Any], source: str = 'defaults', raw: Optional[Dict[str, Any]] = None):
        self._data: Dict[str, Any] = {}
        self._source: str = source
        # 先用 SCHEMA 默认值填充全部键
        for key, spec in SCHEMA.items():
            self._data[key] = spec.default
        # 覆写传参值
        if flat:
            self._data.update(flat)
        if raw:
            # raw 中的键映射后覆写
            for k, v in raw.items():
                mapped = _migrate_legacy(k)
                if mapped in SCHEMA:
                    self._data[mapped] = v
                elif k not in self._data:
                    self._data[k] = v

    @staticmethod
    def load(path: Optional[str] = None, default_data: Optional[Dict[str, Any]] = None) -> 'Config':
        """从路径加载（V1 无此功能）；如果不可用则回退默认值。"""
        if path:
            try:
                # 尝试从 YAML 加载 — 预留接口
                pass
            except Exception:
                pass
        # 回退默认值
        if default_data is not None:
            return Config(flat=default_data, source=path or 'defaults')
        return Config(flat={}, source=path or 'defaults')

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> 'Config':
        """从 dict 构造 Config。"""
        return Config(flat=data, source='dict')

    def get(self, key: str, default: Any = None) -> Any:
        """获取配置值；缺键时回退 SCHEMA 默认值或 default。"""
        if key in self._data:
            return self._data[key]
        if key in SCHEMA:
            return SCHEMA[key].default
        return default

    def require(self, key: str) -> Any:
        """获取配置值；缺键抛 ConfigError。"""
        if key in self._data:
            return self._data[key]
        if key in SCHEMA:
            return SCHEMA[key].default
        raise ConfigError(f"Required config key '{key}' not found")

    def section(self, prefix: str) -> Dict[str, Any]:
        """获取指定前缀下的所有键值子 dict（不含前缀）。"""
        result = {}
        prefix_dot = prefix + '.'
        for k, v in self._data.items():
            if k.startswith(prefix_dot):
                result[k[len(prefix_dot):]] = v
        return result

    def override(self, key: str, value: Any) -> 'Config':
        """覆写并返回 self（链式）。"""
        self._data[key] = value
        return self

    def validate(self, strict: bool = True) -> List[Issue]:
        """校验所有关键参数。返回问题列表。"""
        issues: List[Issue] = []
        for key, spec in SCHEMA.items():
            value = self.get(key)
            err = spec.check(value)
            if err:
                sev = 'error' if (spec.critical or strict) else 'warning'
                issues.append(Issue(key=key, message=err, severity=sev))
            if spec.critical:
                if value is None:
                    issues.append(Issue(key=key, message=f"关键参数未设置", severity='error'))
        return issues

    def describe(self) -> Dict[str, Any]:
        """返回全部键值（仅用于调试/日志）。"""
        return dict(self._data)

    @property
    def source(self) -> str:
        return self._source

    def __repr__(self) -> str:
        return f"Config(source={self._source}, keys={len(self._data)})"


# ── 旧键迁移 ────────────────────────────────────────────────

_LEGACY_MAP: Dict[str, str] = {
    'linear_speed': 'pursuit.cruise_speed_mps',
    'lookahead_dist': 'pursuit.lookahead_distance',
    'wheelbase': 'pursuit.wheelbase',
    'arrival_threshold': 'pursuit.arrival_threshold',
    'max_angular_speed': 'pursuit.max_angular_speed',
    'approach_dist': 'pursuit.approach_dist',
    'SHOOT_COUNT_TARGET': 'mission.shots_per_stop',
    'SHOOT_WAIT_TIMEOUT': 'mission.shoot_timeout_s',
    'SHOOT_STOP_RADIUS': 'mission.shoot_stop_radius',
    'SCAN_WAIT_TIMEOUT': 'mission.scan_wait_timeout_s',
    'SCAN_STOP_RADIUS': 'mission.scan_stop_radius',
}


def _migrate_legacy(key: str) -> str:
    """旧键 → 新键映射。"""
    return _LEGACY_MAP.get(key, key)
