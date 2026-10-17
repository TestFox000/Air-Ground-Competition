# t72 · D-1 src-mirror 镜像移档报告

> 执行成员 plugin-lead｜卡 t72｜2026-10-15｜边界：**只 move 不 delete**；未改 `dzsmartscout/src/**`、未改 fc-audit 报告/脚本、无 git 操作、未碰 `dzrobot_v2/**`

## 0. 总账

| 项 | 值 |
|---|---|
| src-mirror 移动前 | **372 文件 / 2,729,063 B（2.603 MB）** |
| 已移动（md5 逐位一致） | **365 文件 / 2,674,624 B（2.551 MB）** |
| 原地保留（异常项） | **7 文件 / 54,439 B** |
| 归档落点 | `_tmp_session/archive/fc-audit-src-mirror/**`（**保留原相对路径**） |
| 移动中 md5 失配 | **0** |

## 1. ⚠ 卡面「36 组」与实测口径不符，以实测为准

t65 报告内部三处数字不自洽：

| 位置 | 数字 | 上下文 |
|---|---|---|
| §4.1 标题 | 「**39 组 / 78 个文件**」 | 完全相同总组数 |
| §4.1 表内「镜像对」行 | 「**36 组**」 | 该表只列**举例**，非全集 |
| §6 汇总表 | 「**37 组镜像**」 | 与上一行差 1 |

t65 §5 第 5 条已声明本轮**未评域**含 `src/**` 与 `src-mirror/**` 的 C++/msg/srv/xacro（199 `.cpp` / 475 `.msg` / 51 `.srv`…），
⇒ 「36 组」是**排除 C++/msg 后的示意口径**。

**实测（按相对路径配对 + MD5 比对，372 文件全覆盖）：**

| 判定 | 数量 |
|---|---|
| IDENTICAL（字节完全一致，可移） | **365** |
| DIFFERENT（同名但内容不同） | **4** |
| UNPAIRED（src 侧无对应路径） | **3** |
| 合计 | 372 |

按 t65 §5-5 口径复算（排除 `.cpp/.hpp/.hh/.h/.msg/.srv/.xacro/.pyc` 与 `.git/**`）子集 = **72**（`.py` 20 / `.txt` 10 / `.md` 9 / `.yaml` 9 / `.xml` 6 / `.yml` 4 / `.bash` 3 / `.gitignore` 3 / 无扩展名 2 / `.rviz` 3 / `.git` 1 / `.launch` 1 / `.json` 1）。

## 2. 异常清单（7 项，全部原地未动）

### 2.1 DIFFERENT —— 同名但字节不同（4 项）

| # | 文件（`fc-audit/src-mirror/` 下） | mirror len / md5 | src 侧 len / md5 | 判读 |
|---|---|---|---|---|
| 1 | `FAST_LIO_LOCALIZATION_ROS2/config/mid360.yaml:1` | 2,419 / `F15937F1C3D281C10C14BAC3E745FA19` | 2,377 / `6E30A33CCF437DF9A103CBA981CE3132` | ★ **mirror 是更新的版本**（多 42 B）⇒ src 侧落后，**不能单向以 src 为真源**，需人裁决 |
| 2 | `px4_odom_convert/package.xml:1` | 972 / `4F508D30E00461A3C7F26A39EF080C44` | 941 / `29CCA7C231C7A6E6027042BEE06E2591` | ★ 同上（多 31 B），疑为 t60/t59 补依赖后未回灌 src |

> 同目录另有 `px4_odom_convert/package.xml.bak`（见 2.2-3），与第 2 项构成**疑似「改前备份留在 mirror、正式改动只在 mirror」**的模式。

### 2.2 UNPAIRED —— src 侧无对应路径（3 项）

| # | 文件 | len | 判读 |
|---|---|---|---|
| 3 | `px4_control/px4_control_README_DELIVERY.md` | 未记录 | **交付说明文档，src 侧没有** ⇒ 移档会丢交付留痕 |
| 4 | `px4_path/px4_path_README_DELIVERY.md` | 未记录 | 同上 |
| 5 | `px4_odom_convert/package.xml.bak` | 未记录 | **唯一副本**（src 侧无）⇒ `.bak` 后缀即 t65 §5 未覆盖的遗留 |

### 2.3 DIFFERENT（`.git` 索引，同长不同内容，2 项）

| # | 文件 | mirror len / md5 | src len / md5 | 判读 |
|---|---|---|---|---|
| 6 | `px4_msgs/.git/index` | 17,435 / `EA39FEC6EFC47F25D8FC92F5D3458B75` | 17,435 / `DB8A717BC0C4BD139747A7008DDA2801` | 同长不同内容 = **工作区状态差异**，非源码差异 |
| 7 | `px4_ros_com/.git/index` | 2,904 / `624B876D50FCDD0879149301B78431A2` | 2,904 / `8310776BFDFB2EEB6AF3754C136E280F` | 同上 |

> ⚠ **这两项是本卡发现的真风险**：同目录其余 `.git/**` 文件（config/HEAD/refs/objects/packed-refs/logs）md5 **全部一致**并已移档，
> 唯独 `index` 不一致 ⇒ **两个嵌套仓库的暂存区状态不同**，移动后 `_tmp_session/archive/fc-audit-src-mirror/px4_msgs/.git/` 与
> `px4_ros_com/.git/` **处于「refs 全同、index 不同」的半状态**。不构成损坏（`.git/index` 可由 `git reset` 重建），
> 但**若要复原成一个可用仓库，须知道 index 哪边才是对的**。已单列，未擅自处理。

## 3. 移动后复核

| 检查项 | 结果 |
|---|---|
| src-mirror 剩余 | **7 文件**（= 异常清单全量，无遗漏、无多余） |
| 归档落点文件数 | **365**，与移动数一致 |
| 移动中 md5 校验 | 每文件移动前算 MD5、移后重算，**365/365 一致，0 失配** |
| `dzsmartscout/src/**` | files=**1333** / bytes=**394,729,066** ⇒ 与移动前基线**逐位相同，一个字节没动** |
| `fc-audit/*.md`（4 份报告） | 全在：启动手册 8,206 / 场地规则解读与雷达应用 9,873 / 无人机技术文档 58,045 / 飞控代码整理 41,527，md5 已记录 |
| `fc-audit/verify_frames.py`（审计脚本） | 在，7,908 B，md5 `66F9D11D5CCA6C42D1CC62BFFEBED9D0` |
| `fc-audit/notes` / `fixes` / `arena_pdf_img` | 3 / 24 / 38 文件，全部未动 |
| `dzrobot_v2` | HEAD 仍 `4b610d1`，本卡无任何 git 命令 |
| 删除操作 | **0 次**（全部 `Move-Item`） |

## 4. 待裁决（本卡不处理）

1. **`mid360.yaml` 与 `package.xml` 是「mirror 比 src 新」** —— src 侧疑似落后于 mirror。若按「src 是被审计本体」直接采信 src，会丢掉 mirror 里较新的配置/依赖。需确认哪边是真源。
2. **`.git/index` 两项** —— 归档区里的 `px4_msgs` / `px4_ros_com` 现为半状态（refs 同、index 异）。复原为可用仓库前需裁决 index 取舍。
3. **两份 `*_README_DELIVERY.md` + 一个 `package.xml.bak`** 是 src 侧没有的独有件，已原地保留；若确认无用可另开卡删除（本卡禁止 delete）。