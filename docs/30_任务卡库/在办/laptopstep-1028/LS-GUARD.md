---
卡号: LS-GUARD
ns: laptopstep-1028
agent: step-contract
owner: step-contract
reviewer: stop-review
树节点: LS-2
写域: _tmp_session/laptop_step_control/protocol.py；guard.py；tests/test_protocol.py；tests/test_guard.py
kind: implementation
status: in_progress
创建: 2026-10-28
更新: 2026-10-28
来源: 本轮用户请求；dzrobot_v2/docs/决策台账.md
取证: 实
摘要: 协议与独立租约状态机；本地实验，不将离线结果冒充实车通过。
---

# LS-GUARD · 协议与独立租约状态机

## 读过的台账条目
D-039布局构造器唯一真源；D-051运行面硬接管优先；D-055 ttl=0不是失联停车；D-057是历史另一趟，不作本轮授权。

## 复用检索结果
docs入口及空地链路检索；复用清单→能力索引→目录索引，复用dzbrain.scout.layout.build_layout、io.contract.TOPICS、StepLoop及Tracker；不重建规划器或控制器。现有supervisor不是透明双向桥，旧发射器会自动推进，不满足人工逐段及锁存停车。

## 新增轮子及理由
本轮仅实验目录中的最小JSONL协议与纯状态机，已按独占写域实现；独立驻车端服务负责本地单调租约。协议归一化与格名/邻格判据复用生产实现，不复制布局构造器、坐标公式、规划器或控制器；无新依赖、无生产配置键。写入前已在本卡及共享任务登记。

## 写域
[_tmp_session/laptop_step_control/protocol.py](../../../../_tmp_session/laptop_step_control/protocol.py)、[guard.py](../../../../_tmp_session/laptop_step_control/guard.py)、[test_protocol.py](../../../../_tmp_session/laptop_step_control/tests/test_protocol.py)、[test_guard.py](../../../../_tmp_session/laptop_step_control/tests/test_guard.py)及本卡；共享任务revision 4已登记。禁止改冻结生产源码、默认配置及他人文件；禁止commit/push。除Lead外禁止设备连接、部署及运动发布。

## 验收命令
工作目录=v1；预期退出0。作者已执行下列纯单测：协议11项、监护22项均OK/exit0；独立reviewer须另复跑并留原始输出，当前未验收。

~~~bash
python -B -X utf8 -m unittest discover -s _tmp_session/laptop_step_control/tests -p test_protocol.py -v
python -B -X utf8 -m unittest discover -s _tmp_session/laptop_step_control/tests -p test_guard.py -v
~~~

## 验收清单（领卡初始全false）
- [ ] 单段正确性及真实回执匹配
- [ ] 重复、旧回执、超时、断链与重连不续跑
- [ ] 独立reviewer复跑并记录退出码
- [ ] 冻结区增量0及门禁结果明确
- [ ] 实车未证项与残余风险明确

## 回滚点
原生产零改动；停止本轮新进程，保留证据，仅移除本卡新增实验文件。共享登记按本轮增量外科逆转，不reset、不覆盖他人。设备写入另需部署清单/哈希和回滚记录。

## 执行记录
2026-10-28 Lead建卡。基础check.py GREEN/exit0；根仓和车仓均有大量既有改动，全部保留。用户已确认本轮N10朝M10、清场及有人保障；软件验收未完成前不发车。

### 作者候选v1（局部自测，不是验收）

- [协议归一化与真实done匹配](../../../../_tmp_session/laptop_step_control/protocol.py)：严格输入类型，忽略ts的同版签名；不把run_id、done.replay或普通FINISH充作本趟运动完成。
- [锁存与挑战租约](../../../../_tmp_session/laptop_step_control/guard.py)：启动/故障锁1；只消费一次当前challenge回显，超时边界先锁后处理，重连不续跑。
- [人工同步/permit](../../../../_tmp_session/laptop_step_control/guard.py)：bootstrap必须人工FRESH+B0+N10及新鲜真实GID；不伪造缺失版本/位姿。首段N10至M10，首permit发0一次；正常后段不再切接管。
- [done只转WAIT_HUMAN](../../../../_tmp_session/laptop_step_control/guard.py)：匹配段/版本/目标/完成序号后仍需现场停稳确认和下一permit；done不触发锁1。
- SHA256：协议 `33e0dba6a8b1563b6d78203097f4c2bd5da0e51b0bb8c6e7c202f9bce8e6fdb5`；监护 `e582b1f463d3673f36f022078f604a3417b1c5837b8e8063e3f06096365ca62b`；协议测试 `8c1caeae8ebbac73d944693de11b104720831b5858e2859f9f3ee30040f237b6`；监护测试 `7900cf8a50c8ceb1a20828dc9cc867d50a6a42dd64298de38218c6d7f68e60f4`。
- 本任务仅新增四个实验源码/测试并更新本卡，无生产文件修改，无SSH/设备连接，无ROS/运动发布；既有脏工作树保持。工作流门禁由Lead统一，当前不得声称门禁收口或实车通过。
- 未证：真实DDS GID、首release与layout跨话题到达、实际制动/滑行、实验2秒租约与状态TTL的现场适用性。监护进程死亡/ROS或底盘失效仍有残余风险；生产终点快速清done闩锁，末段漏收不能用FINISH伪造完成。禁火须Lead核实实际生产联锁，不由协议拒shoot字段代替。

### 独立窄验收（stop-review，限本地pure边界）

独立评审员已以冻结哈希复跑作者协议11项/监护22项和独立纯类15项反例，均exit0；不是作者自评分。[原始日志第4至18行](../../../../_tmp_session/laptop_step_control/review/1793193377772574900_independent.log)给出纯类反例逐项结果，[结构化清单](../../../../_tmp_session/laptop_step_control/review/1793193377772574900_suites.json)记录原命令、exit0、snapshot_stable=true以及hardware_go=false。清单中接线进程7项是另一边界，不与本卡pure结果混记。读取证据可见冻结区diff在该轮前后同hash；整库门禁/真实GID/制动/实车尚未验收，由Lead统一收口。本卡实现交付完成，但不擅自关闭整体验收清单或宣称可发车。

## 文档变更
本卡及本ns树；正式收口由Lead统一索引。

## 经验总结
现象：无人机不可达不必阻断笔记本模拟输入。处置：保留真实控制闭环，只在实验链路适配；done不等于停稳、心跳TTL不等于停车。
