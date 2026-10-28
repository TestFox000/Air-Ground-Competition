---
来源: E1 对账 D3；DL4 阻塞清单；Lead 2026-10-24 17:3x 派工
取证: 只读源码 + 离线单测（Windows 可跑）+ stub 顶替 ROS 依赖的节点冒烟
更新: 2026-10-24
status: in_progress
ns: agc-1024
agent: dl-drone
树节点: AGC-16
写域: 本卡；`dzsmartscout/src/FAST_LIO_LOCALIZATION_ROS2/scripts/tf_compat.py`（新）；同目录 `transform_fusion.py` 的 import 链；同目录 `test_tf_compat.py`（新）
摘要: transform_fusion.py 的 tf2_ros.transformations 回退路径在 ROS2 不存在，节点 import 即死，/localization 因此无发布者；用 numpy-only shim 修 import 链，离线可验（32 条单测 + 节点 stub 冒烟），不部署机端。
---

# AGC-16-TFCOMPAT-20261024 · transform_fusion 导入失败修复（numpy shim）

owner: dl-drone；reviewer: 待定（Lead 指派）。

## 1 依据与复用（五必填）

- 读过的台账：E1 对账 §D3；DL4 阻塞清单 D3；`fc-audit/启动手册.md` 里"三节点全死"的现场记录。
- 复用检索：仓内已有 numpy（`transform_fusion.py` 本身依赖）；全仓无现成 4 函数实现可复用
  （`find -name transformations.py` 为空、本机无 `tf_transformations`、无 scipy）→ 允许新增最小 shim。
- 新增轮子：`tf_compat.py`（唯一理由：ROS2 无 `tf2_ros.transformations`；只实现代码实际调用的 4 个函数，
  不把 Gohlke 的 transformations.py 整包搬进来）。
- 验收命令：本目录单测（见 §5）+ 静态证明 import 链不依赖 `tf2_ros.transformations`。
- 回滚点：`transform_fusion.py` 的 import 分支回到"只有 try/except 旧内容"＋删除 `tf_compat.py`；无其它改动。

## 2 当前问题

`transform_fusion.py` 第 11–15 行先 `import tf_transformations`，失败后 `from tf2_ros import transformations` ——
ROS2 没有这个模块 ⇒ `ImportError: cannot import name 'transformations' from 'tf2_ros'` ⇒ 节点 import 阶段即死
⇒ `/localization` 永远无发布者（D3 的一半根因；另一半是雷达 carrier=0，属物理）。
现场观感与"一切正常"几乎无差别：FAST-LIO 在跑、点云与 RViz 正常，只有定位话题不存在。

## 3 唯一假设与验收

- 假设：只补 4 个变换函数即可让节点通过 import，且位姿数值与 tf_transformations 在位时一致（±q 等价）。
- 必测：已知 T 往返、yaw=90°、单位矩阵 / 纯平移 / 纯旋转、180° 退化；若本机有 `tf_transformations` 则对拍
  （本机**没有**，故改为与独立 Rodrigues 实现逐点对拍 + 节点冒烟）。
- 边界：不部署机端、不碰 FAST_LIO 其它源码、不改 launch/config。

## 4 交付

1. `tf_compat.py`（numpy-only，4 个函数）＋ `test_tf_compat.py`（32 条）。
2. `transform_fusion.py` 的 import 链（只改这一段，19 行 → 见 §5-2）。
3. 静态 + 动态证据：全仓已无 `tf2_ros import transformations`（除文档/测试字符串引用）；
   屏蔽 `tf_transformations` 后节点真能 import 并算出同一套位姿。

## 5 执行记录（2026-10-24 18:1x · dl-drone）

### 5-1 单测（离线，Windows 本机）

| 命令 | 退出码 | 结果 |
|---|---|---|
| `python -B -X utf8 -m unittest discover -s dzsmartscout/src/FAST_LIO_LOCALIZATION_ROS2/scripts -p "test_tf_compat.py"` | **0** | Ran **32** tests · **OK**（0 skip / 0 fail） |

覆盖四组：

- **A 数值**：`translation_matrix` / `translation_from_matrix` 形状/类型/副本语义/200 组随机往返；
  `quaternion_matrix` 对 yaw/roll/pitch 各 90° 已知矩阵、非单位四元数归一化、零四元数退化单位阵；
  与**独立 Rodrigues 轴角公式**（另一套代数，不用四元数）随机 300 组逐点对拍（atol 1e-12）。
- **B 反解**：`quaternion_from_matrix` 单位阵→`[0,0,0,1]`、yaw 90°、180°（w≈0，只判轴与模长，符号歧义是数学事实）、
  随机 300 组 q→R→q′ 矩阵往返、`w>=0` 规范化；复合 T（平移+yaw）两个分量都回来；
  另有一条复刻节点算式 `T_map_to_base = T_map_to_odom @ T_odom_to_base` 与手算一致。
- **C 依赖面**：`tf_compat.py` 的 AST import 根只有 `math/numpy/__future__`；`__all__` 正好 4 个。
- **D 导入链**：见 5-3。

### 5-2 import 链改动（只改这一段）

```
import tf2_ros
try:
    import tf_transformations          # 机上有这个包就照原样用（行为不变）
except ImportError:                    # ROS2 没有 tf2_ros.transformations（AGC-16 根因）
    try:
        import tf_compat as tf_transformations
    except ImportError:                # 从别的 cwd 当脚本拉起时兜底：把脚本同级目录塞进 sys.path
        import os as _os
        import sys as _sys
        _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
        import tf_compat as tf_transformations
```

节点其余逻辑一字未动（差分只有这 19 行 → 新 21 行 + 注释）。

### 5-3 静态 + 动态证明

| 证据 | 结果 |
|---|---|
| 全仓搜索 `tf2_ros import transformations` | 生产代码 **0 处**（命中的只有本卡文档、`tf_compat.py` 的"为什么"注释、`test_tf_compat.py` 的断言、`fc-audit` 的说明、`git_save/` 与 `_tmp_session` 的历史副本） |
| AST 断言（单测内） | `transform_fusion.py` 无 `ImportFrom(tf2_ros, transformations)`；`tf2_ros` 只有一次裸 `import`；兜底串存在 |
| 动态 fallback（单测内） | 把节点那段 try/except **原文抽出 exec**，用 `sys.meta_path` 屏蔽 `tf_transformations` ⇒ 名字确实绑到 `tf_compat` |
| **节点冒烟（最强）** | 用 stub 顶掉 `rclpy / rclpy.node / tf2_ros / geometry_msgs / nav_msgs`，`exec_module` 加载 `transform_fusion.py` —— 原来这一行就是 ImportError 死点，现在通过；实例化节点后：订阅面 = `[/Odometry, /map_to_odom]`，发布面只有 `/localization` 一个；`pose_to_mat` 与直算一致；`publish_localization` 的 TF（map→camera_init）= map2odom 本身，`/localization`（map→body）= `T_map_odom @ T_odom_base`，均 atol 1e-12；`map2odom=None` 时退化为恒等（原逻辑不变） |

## 6 结论与未证

**成立（实测）**：没有 `tf_transformations` 的机器上，节点 import 通过、位姿算式与独立实现一致；
`tf_compat` 只依赖 numpy/math；import 链不再引用不存在的 `tf2_ros.transformations`。

**未证（明确不做）**：机端真机启动（需要 colcon 重装 + ROS2 环境，属部署窗口）；
与真 `tf_transformations` 的 1:1 对拍（本机无该包、无网络安装；已用独立实现替代，
差异面只剩 ±q 符号与 ≤1e-16 级浮点）。

## 7 交付面风险（只报，不改）

| # | 风险 | 影响 | 建议 |
|---|---|---|---|
| R1 | **此目录是未初始化的子模块**：父仓里 `dzsmartscout/src/FAST_LIO_LOCALIZATION_ROS2` 是 gitlink（`3a005b96…`），目录内无 `.git` ⇒ 父仓 `git add` 该路径报 "is in submodule"，本次改动**进不了父仓提交** | 修复只存在于工作区；子模块一旦 `git clean/reset/重新 init` 会被覆盖 | 二选一：①把子模块初始化并在其上游提交；②按仓内既有做法落成补丁包（`dzsmartscout/fc-audit/fixes/02-tfcompat/files/...`，模仿 `01-odom-source`），部署时随脚本一起拷 |
| R2 | **CMakeLists 的 install(PROGRAMS) 没列 `tf_compat.py`** ⇒ `colcon build` 后 `lib/FAST_LIO_LOCALIZATION_ROS2/` 里没有它，节点运行期 `import tf_compat` 仍会失败 | 部署后依旧起不来 | 部署窗口在 `CMakeLists.txt` 的 install(PROGRAMS) 里加一行 `scripts/tf_compat.py`（不在本卡写域） |
| R3 | 另有两条**同因未修**的脚本：`global_localization.py`（裸 `import tf_transformations` 无兜底）、`publish_initial_pose.py`（还用 `quaternion_from_euler`） | 它们仍会 import 即死；但按本卡边界不碰 | 需要时另立卡：给 `tf_compat` 补 `quaternion_from_euler` 或让两脚本同样回退到 shim |

## 7-1 补丁包落盘（2026-10-24 18:3x · Lead 裁决①）

因为 FAST_LIO 目录在本仓是**未初始化子模块**（父仓只有 gitlink，`git add` 报 "is in submodule"），
按 Lead 裁决改为本仓既有形态的补丁包（编号顺延：02 已被 `02-px4-path-config` 占用 → 用 **07**）：

```
dzsmartscout/fc-audit/fixes/07-tfcompat/
├── README.txt                                        # 纯文本（不建 .md，避免 R1）
├── apply.sh                                          # 应用 / --rollback；bash -n 通过
└── files/dzpx4/src/FAST_LIO_LOCALIZATION_ROS2/        # 目标机路径镜像（机端 ~/dzpx4/src/...）
    ├── CMakeLists.txt                                # install(PROGRAMS) + scripts/tf_compat.py
    └── scripts/
        ├── tf_compat.py
        └── transform_fusion.py
```

`apply.sh` 做的事：备份被覆盖的原文件到 `<DEST>/.tfcompat-backup-<时间戳>/` → 拷贝三件 →
四项校验（`import tf_compat` 成功 / 兜底串在位 / **旧兜底已清除** / CMake 已列 `tf_compat.py`）→
打印后续手动步骤（`colcon build --packages-select fast_lio_localization` 与 `ros2 topic hz /localization`）；
`--rollback` 从最近一次备份还原两个原文件并删掉新增的 `tf_compat.py`。

**实测（在临时 DEST 上用"改动前原件"跑通全流程，不碰机端）**：

| 步骤 | 结果 |
|---|---|
| `bash -n apply.sh` | exit 0 |
| apply（目标为改动前原件） | exit 0；四项校验全过（`import tf_compat OK: 4 functions` / 兜底在位 / 旧兜底 0 处 / CMake 列上） |
| 备份一致性 | `cmp` 备份的 `transform_fusion.py`、`CMakeLists.txt` 与原件**逐字节相同** |
| rollback | exit 0；两个文件**逐字节还原**、新增的 `tf_compat.py` 已删除 |
| 镜像一致性 | `files/` 三件与工作区 `dzsmartscout/src/FAST_LIO_LOCALIZATION_ROS2/` 同路径文件 sha256 全等 |

**注意（部署前置）**：`apply.sh` 只改源文件，不替你 `colcon build`、不重启节点；机上仍需现场窗口。

## 8 边界声明

不连无人机、不部署机端、不改 `config`/launch、不碰 FAST_LIO 其它源码；本次只改上表三个文件。

## 10 文档变更

新增本卡（按 Lead `_tmp_session/agc-1024/mkcards.py` 里的草稿落地并填执行记录；**Lead 若重跑该脚本请跳过 AGC-16**，
否则会覆盖本卡 §5–§7）。不改库内其它文档；长文档零新增（§7 风险若需进库，等 Lead 提炼到
`20_资料索引库/无人机/` 或 FAST_LIO 相关分册）。
