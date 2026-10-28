---
来源: 用户 2026-10-25 循迹替换交接说明
取证: 实/算/未
更新: 2026-10-25
取代: 无
status: in_progress
ns: traj-1025
agent: /root/speed
owner: /root/speed
reviewer: /root/baseline
树节点: TR-3
写域: dzrobot_v2/src/dzbrain/dzbrain/pursuit/velocity.py, dzrobot_v2/src/dzbrain/tests/test_pursuit.py
---

# TRAJ-SPEED-20261025 · 有刹停约束与 jerk 界的速度剖面

## 1 目标与台账
有刹停约束与 jerk 界的速度剖面。已读 D-008/D-012/D-023/D-027/D-028/D-034/D-041：冻结登记、保留 planner、旧红分类、阿克曼前提、卡树隔离、双远端分叉不得擅自强推。用户本轮授权替换循迹，但不授权部署。离线候选通过前不切现役默认。

## 2 复用检索
已查 doc_search「连续轨迹 阿克曼 循迹」、复用清单、能力索引、目录索引；源码搜索 trajectory/clothoid/dubins/quintic/jerk 无现成实现。复用 Path/Route/geometry/physics/route_check/现有 sim 与现有 tests 树。仿真不能证明深度相机实测 RMS。

## 3 新增轮子及理由
velocity.py：现有 speed_profile 是单拍局部 min，无全程制动可达性与 jerk 状态。

## 4 写域
dzrobot_v2/src/dzbrain/dzbrain/pursuit/velocity.py, dzrobot_v2/src/dzbrain/tests/test_pursuit.py。只写本卡列举文件；不覆盖既有在途文件，不碰 planner/云台/感知/config 默认值，不联网操作车。

## 5 验收命令
根目录：python docs/_工具/gen_index.py --ns traj-1025；python docs/_工具/check_doc_policy.py；python docs/_工具/check_anchors.py；python docs/_工具/check_numbers.py。
dzrobot_v2：python tools/wf/check.py；python tools/wf/check_links.py；python -m pytest src/dzbrain/tests -q；现有 test_pursuit.py / test_corner_cut.py 逐脚本验收。无 pytest 不能算通过，另记原生 harness 实测。

## 6 回滚点
根仓 HEAD aa9b37341187f311c707d30288ff433c7851eea8；本轮原件在 `_tmp_session/traj-1025/before/`。只逆转本卡差异，禁止 reset/clean 或覆盖其他团队改动；两个嵌套仓分开核对。

## 7 执行记录
2026-10-25 已读取规则并登记；基线与实现进行中。检查和独立复核未完成，不提交、不归档。

## 8 结论
未验收。外部深度相机真值、全部布局可行性与 180 秒实车预算未证实。

## 9 经验总结
现象：根仓 status 不能覆盖 dzrobot_v2 嵌套仓；后者有既有改动及暂存项。处置：分别读取 git status/diff，保存改前副本，只归属本卡差异。

## 10 文档变更
新增：本卡与 traj-1025 任务树。修改：ns 登记、复用/能力登记、冻结登记与团队状态中的本轮段落。归档：无。

## 7.1 速度核本轮实测（2026-10-25）

- 复用检索定位纠正：没有独立斜坡测试文件，实际在 `test_pursuit.py` 的 test_speed_ramp 函数内。按 root 串行移交，新增同文件 §7c/§7d，保留全部旧断言。
- 新入口 `pursuit/velocity.py`：最大二阶可达包络两遍扫描；速度网格按加速距离自动细分；单调段合并后用连续加速度 smoothstep；解析最大 jerk 校验及时间伸长。另比较全程最小安全 cap 候选，避免微小限速变化让全程时间伸长。两个候选均可行，**不是严格 jerk 时间最优求解器**。
- `python -B -X utf8 dzrobot_v2/src/dzbrain/tests/test_pursuit.py`：速度核补入时 143 PASS / 0 FAIL（其中本卡 21 条）；后续控制卡继续在同树增加契约，计数随其结果走。
- 独立探针：300 组非均匀弧长/曲率/任务包络/停点输入全部满足区间包络与解析 a/j；50000 拍随机在线目标经过修复后不崩溃。此探针只辅助排错，不替代现有测试树。
- 4 m 直线 4 mm 网格：12.89143 s、巡航 0.35 m/s、最大加速度 0.35889 m/s²、最大 jerk 0.98133 m/s³；两端点输入自动细分后时长与密网格差 < 1%。`second_order_duration_s` 仅为当前离散包络的二阶参考，不能宣传成全路径 jerk 最优下界。
- MotionState 保留上一拍加速度；支持 reset、普通目标及解析 acceleration feedforward。安全硬限速最终优先，不可达时 constraint_override 为真，输出真实超限 a/j；急停不冒充平滑执行。
- 发现的数值坑：极小加速度持续多拍可能得到 0.010000000000000026，不能把 1 ULP 当非法状态；用尺度容差及状态舍入保护，原树新增 7d.7。静止速度 <1e-14 归零，防浮点正余量导致停站永不握手。

## 8.1 本卡当前判定
离线候选速度核可复跑；非全局 jerk 时间最优。未部署、未提交、未归档；全链路与门禁由 root 统一复核。源变更仅 velocity.py 与原 test_pursuit.py 的 §7c/7d。

## 7.2 既有停点之间独立时间参数化

- 现象：旧候选在第七图参考速度上限最小0.07422m/s，但25%分位至最大均为0.35m/s；全程最小cap或全程最坏jerk dilation把局部瓶颈传播到全部直道。249.50s行程不是物理上限所要求，不能拿龟速换低RMS宣告提升。
- 通用修正：仅按输入已经存在的零速、零加速度停点分区；每区独立比较变化cap/均匀cap候选并独立dilation，再连续拼接。区间交界v=a=0，两侧jerk仍各自有界；不增加停点、不改加速度/jerk/巡航参数。保留全程候选作时间不劣的兜底。
- 同一桥结构版几何采样输入：249.500971s→128.242985s，二阶参考保持59.607499s；解析max a=0.360938m/s²、max jerk=0.999999999997m/s³、包络超限0、停点速度0。过程数据 `_tmp_session/traj-1025/velocity-locality.json`。
- 原树新增7c.15/7c.16验证低速段不再拖慢停点后的普通直道，以及独立剖面停点拼接连续；全test_pursuit现160/160。
- 局限仍在：7.722→12.053m的单条4.33m区间仍需55.72s，区间内部瓶颈仍可能传播；并非严格jerk时间最优，不对180s任务预算作通过承诺。

## 7.3 变化速度包络的逐区间证书

- 下一单变量修正：单调速度过渡合并不再要求整段最高速度小于整段最低cap；改为在每一个原始cap小区间上，按解析单调v(s)的最坏端点检查。速度单调保证该端点覆盖区间全部时刻，后续全区间时间伸长只降低速度，证书保留。加速度/jerk仍解析检查。
- 同一旧几何与物理限值：128.242985s→92.777954s，长4.33m区间55.718697s→25.550155s；二阶参考59.607499s不变，max a=.360938、max jerk=.999999999997、包络超限0、停点速度0；计算耗时约0.313s。证据 `_tmp_session/traj-1025/velocity-envelope-certificate.json`。这份旧几何比较只隔离速度算法，不代表最终车心雷覆盖验收。
- 原树7c.17/7c.18：4m局部平滑cap谷，同参28.296s→15.280s；1mm逐原cell密采样核对速度包络，12000点独立时间差分核对a/j；没有新增停点。全test_pursuit现163/163。
- 速度核最终候选SHA256：`a8dadc15be418844f1394e3abaa518b6df4169b9114be1ef156730b5cc89171c`。仍为保守可行构造，不能宣称全局jerk时间最优；实际任务时间还需反向速度比例、停站等待和射击。

## 7.4 在线几何cap可达过滤

第一轮整图证据说明离线profile满足a/j不保证“时间速度+进度反馈”的命令仍满足：未来cap跌落前未提前制动，末层hardcap产生不连续。新增SpeedEnvelope仅作同一速度状态的可达性过滤，使用sampled_s/caps、当前v/a、jerk和延迟距离预算，给MotionState下一拍加速度上界；不替代现有剖面、不新增停点、不取消最终hardcap。不可达初始状态仍显式报失败。短场景前后及源哈希见 `_tmp_session/traj-1025/velocity-reachability-step.json`，原树7d.8/9通过；全树此时167/167。第二轮统一两图未出前不改总验收结论。
