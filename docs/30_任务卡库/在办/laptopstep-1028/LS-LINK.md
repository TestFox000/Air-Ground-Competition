---
卡号: LS-LINK
ns: laptopstep-1028
agent: laptop-link
owner: laptop-link
reviewer: stop-review
树节点: LS-1
写域: _tmp_session/laptop_step_control/laptop.py；_tmp_session/laptop_step_control/transport.py；_tmp_session/laptop_step_control/car_bridge.py；_tmp_session/laptop_step_control/tests/test_transport.py；本卡
kind: implementation
status: in_progress
创建: 2026-10-28
更新: 2026-10-28
来源: 本轮用户请求；dzrobot_v2/docs/决策台账.md
取证: 实
摘要: 笔记本及车端薄适配；本地实验，不将离线结果冒充实车通过。
---

# LS-LINK · 笔记本及车端薄适配

## 读过的台账条目
D-039布局构造器唯一真源；D-051运行面硬接管优先；D-055 ttl=0不是失联停车；D-057是历史另一趟，不作本轮授权。

## 复用检索结果
docs入口及空地链路检索；复用清单→能力索引→目录索引，复用dzbrain.scout.layout.build_layout、io.contract.TOPICS、StepLoop及Tracker；不重建规划器或控制器。现有supervisor不是透明双向桥，旧发射器会自动推进，不满足人工逐段及锁存停车。

## 新增轮子及理由
本轮实验JSONL适配已实现（四文件见写域）；协议与租约状态机复用LS-GUARD候选，不重写。车端独立serve负责租约与ROS，attach不拥有ROS、不起serve。新增理由仍为现成supervisor不能透明下发布局及完整回传；无新依赖、无生产配置键，冻结区未写。

## 写域
_tmp_session/laptop_step_control/laptop.py；_tmp_session/laptop_step_control/transport.py；_tmp_session/laptop_step_control/car_bridge.py；_tmp_session/laptop_step_control/tests/test_transport.py（本卡独占候选已实现）。另只维护本卡；未写本ns树或共享登记。禁止改冻结生产源码、默认配置及他人文件；禁止commit/push。除Lead外禁止设备连接、部署及运动发布。

## 验收命令
工作目录=v1；预期退出0。作者定向自测已执行，独立reviewer正在复跑；作者输出不是独立验收，以下清单暂不勾。

~~~bash
python -B -X utf8 -m unittest discover -s _tmp_session/laptop_step_control/tests -p test_transport.py -v
~~~

## 验收清单（领卡初始全false）
- [ ] 单段正确性及真实回执匹配
- [x] 重复、旧回执、超时、断链与重连不续跑（仅独立离线机制/黑盒，不代真实ROS或停车）
- [x] 独立reviewer复跑并记录退出码
- [ ] 冻结区增量0及门禁结果明确
- [x] 实车未证项与残余风险明确（独立评审保留hardware_go=false）

## 回滚点
原生产零改动；停止本轮新进程，保留证据，仅移除本卡新增实验文件。共享登记按本轮增量外科逆转，不reset、不覆盖他人。设备写入另需部署清单/哈希和回滚记录。

## 执行记录
2026-10-28 Lead建卡。基础check.py GREEN/exit0；根仓和车仓均有大量既有改动，全部保留。用户已确认本轮N10朝M10、清场及有人保障；软件验收未完成前不发车。

### 作者交付与边界（2026-10-28，独立验收待回）

- [视频观测接入口](../../../../_tmp_session/laptop_step_control/laptop.py)只接受显式 map/known/source.video，调用唯一 build_layout；不读默认历史图，不填全场 known。视频识别由LS-VIDEO独立生产，当前视频现场一致性是用户确认，不代替识别证据。
- [人工逐段驱动](../../../../_tmp_session/laptop_step_control/laptop.py)等待真实 Guard ready、显式sync和STOPPED/EXCLUSIVE许可；首段固定N10到M10，之后每段仍需新许可。真实done后不预送，终点done后结束许可。命令记录本地输入时刻，早于当前ready/done的排队许可不能生效。
- [独立服务](../../../../_tmp_session/laptop_step_control/car_bridge.py)与[一次性附着](../../../../_tmp_session/laptop_step_control/car_bridge.py)分离。Guard唯一出ROS effects；消息完整透传，GID来自真实MessageInfo。启动/每permit前/运行期查override独占，查询未知拒release；周期锁1、首permit仅一次0由Guard决定，不以多发布者覆盖真实人工接管。
- [有界writer](../../../../_tmp_session/laptop_step_control/transport.py)隔离stdout/socket/stderr反压，EOF/黑洞不靠write成功续命；emit失败当拍trip且不执行后续release/layout。无PTY、无shell payload、无凭据硬编码、无自动重连重播。
- 作者命令：上节定向unittest；首版22项OK，新增“预输入许可”和“emit失败阻断后续效果”后最终24项OK，exit0，耗时0.165s。仅pure/fake IO，无SSH、无真ROS、无设备连接。独立reviewer=stop-review，正在进行额外故障注入，未将作者绿算验收绿。

当前冻结SHA256（后续改动须重新评审）：

~~~text
laptop: 605a54f1b7511e303efc33df3bcdcec53cf5d87e4399beb67f4a0a5bb3a6c75f
transport: ea9a7075762aae39b5d6398c4c63da8308bc0319f7a2426c60a6a99be34fe761
car_bridge: 4e10a5fafbcac28b724b7adf0bd302d9fd215641caca1e9d71b172db645ebff3
test_transport: b99210a87c2e578af488af02aa864aae5384051988db3c6864d226ed62fdd534
~~~

未证：冷启动是否先有真实car_state/GID、现役ROS helper/API及部署身份、真实DDS交付/接管停车/滑行距离、视频自由格与桥洞语义。冷启动无遥测不能伪造ready放行，Lead单独核实；离线fake mode不代证。监护自身/ROS/底盘故障为残余风险，软件租约不是硬急停。全链路和实车验收均未完成。

本卡未写冻结区或启动设备；本轮共享索引/门禁由Lead统一，未自报收口GREEN，未commit/push。回滚仍只移除本卡四个实验文件，不reset他人。归档落点由Lead整合入空地链路主题或本轮决策，当前不归档。

### 独立窄范围接受（不代表卡面实车整体验收）

stop-review已对上列冻结候选完成独立复跑，结论SCOPED_ACCEPT，仅允许Lead考虑新实验文件窄推和保持锁停的ROS静态联调；不授权permit、起brain、发布layout或发override0。[稳定性与退出码原件](../../../../_tmp_session/laptop_step_control/review/1793193377772574900_suites.json)记录local_pass=true、snapshot_stable=true、hardware_go=false，源码前后哈希相同，冻结diff前后哈希相同。

亲读原件及日志：全候选作者套件由独立方重跑97项OK（其中本卡24项），额外独立纯guard15项与serve隔离进程黑盒7项合计22项OK，两次进程returncode均0、无timeout。独立黑盒覆盖EOF、无EOF黑洞、下行黑洞机制、stdout堵塞、半行JSON、异常JSON和override竞争；[原始独立日志](../../../../_tmp_session/laptop_step_control/review/1793193377772574900_independent.log)与[全候选日志](../../../../_tmp_session/laptop_step_control/review/1793193377772574900_author.log)可复查。

视频源现役输出仍为NO_GO，故“视频→航点→真车”目标未完成，绝不以固定图填空。真实GID/版本、ROS helper、冷启动无运动bootstrap、物理停稳/刹停与来源hash对应仍交Lead核验；本卡相关真实验收保留false。共享任务complete仅表示本卡实现及独立离线交付完成，文件卡不归档，门禁和整体实车收口留Lead。

## 文档变更
本卡及本ns树；正式收口由Lead统一索引。

## 经验总结
现象：无人机不可达不必阻断笔记本模拟输入。处置：保留真实控制闭环，只在实验链路适配；done不等于停稳、心跳TTL不等于停车。
