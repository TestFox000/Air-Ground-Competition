# 车端上机包 · 2026-10-17

> 车端香橙派 RK3588 一次性上机全部资产。
> 全部部署器遵循铁律：备份→推送→编译→重启→验证，失败可回滚（§五）。
> 包内为资产快照（`01/02/03` 分节保持了脚本相对依赖，包内可直接运行）。

## 一、包内清单

| 分节 | 内容 | 用途 |
|---|---|---|
| `01_车端大脑ag_integrate/` | src/ag_integrate（态势大脑：intel_protocol 13类解析 / world_model paper 落格判语义 / mission_router 分区规划 / fire_guard 禁火 / car_adapter 等）+ src/ag_core（黑匣子依赖）+ car_deploy.py + remote_run.py | 车端大脑部署（~/ag_ws colcon） |
| `02_标靶pose模型/` | target_pose_v2_best.onnx（11MB）+ deploy_pose_rknn.py | 靶标 pose 模型 ONNX→RKNN 一键部署（P=0.985 / R=0.999 / mAP50=0.995，红靶零误检） |
| `03_dznav循迹/` | src/dznav 整包 + dznav_deploy.py | 自研循迹接管（SIL：S 弯 RMS 3.2cm，雷格过心全 8 格 ≤6cm） |
| `04_标靶锁定增强/` | C++ 伺服 5 修复补丁（dzrknn_dataprocess）+ target_motion_predictor.py（移动靶运动预测）+ target_speed_calib.py + 标靶锁定增强补丁说明.md | 治"锁不住/锁了就丢/移动靶打不中" |

## 二、前置

```powershell
set AG_CAR_PASS=<车端密码>    # 缺省自动尝试 duzhong / 12345678
```

- 车端 IP 以现场为准（历史：pose 链路 192.168.0.199，dznav 示例 192.168.0.198）。
- 车端 ROS 域真源 `ROS_DOMAIN_ID=118`（car_deploy 已内置处理）。
- 本机依赖：paramiko；pose RKNN 转换还需 rknn-toolkit2（缺失时 deploy_pose_rknn.py 给安装指引，或 `--rknn` 用现成转换件跳过）。

## 三、上机顺序（1→5）

**1 · 车端大脑**（基础平台，先于其他项）：
```powershell
cd <本包>\01_车端大脑ag_integrate
python tools\car_deploy.py --host <车IP> --user duzhong
```
（或从主仓库原位置运行：`air_ground\tools\car_deploy.py`。）
流程：打包 src/ag_integrate + src/ag_core → 上传 `~/ag_ws/src/` → `colcon build` → 集成节点冒烟验证。
判据：build 通过 + integrate_node 冒烟正常。

**2 · 标靶 pose 模型（一键全自动）**：
```powershell
cd <本包>\02_标靶pose模型
python deploy_pose_rknn.py --check-only          # 先本地自检（不连车）
python deploy_pose_rknn.py --host <车IP>         # 转换→上传→备份→替换→重启→验证
```
判据：det_node 在 ROS 图 + 检测话题出帧 + 重启日志无模型加载失败。板端不在线可 `--wait 3600` 挂机轮询。

**3 · dznav 自研循迹（待命铁律）**：
```powershell
cd <本包>\03_dznav循迹
python air_ground\tools\dznav_deploy.py --host <车IP> --user duzhong
```
部署器自动以 **`DZ_AUTO_START=0` 待命**拉起（不自动发车，比赛时才显式置 1）。
判据：/odom、/localization 发布者数==1、信念流 50Hz、无双源告警；legacy 进程命令已留档。

**4 · 标靶锁定增强**：按 `04_标靶锁定增强\标靶锁定增强补丁说明.md` 执行：
- 一期 C++ 伺服 5 修复：§四 4 步（备份 → 推送 cpp/h → colcon build → 重启节点），remote_run.py 在 `01_车端大脑ag_integrate\tools\`；
- 二期运动预测器：§5.4 robot.yaml 配置（两键开火 + `predict_gain: 0.0` 防双重补偿）+ 启动 `target_motion_predictor.py`。
判据（§三）：锁定后不点头、边缘丢靶微找重捕、`扫描确认目标，允许射击` 连贯出现。

**5 · 标靶速度标定（现场）**：
```powershell
python <本包>\04_标靶锁定增强\target_speed_calib.py --self-test
```
自检通过后对实车靶现场标定，产出 px_per_tick / 周期 / 停顿参数回填 robot.yaml。

## 四、执行建议

- 步骤 2/3/4 相互独立，可按现场情况调整顺序；建议 1 → 2 → 3 → 4 → 5 完整走一遍后实车验证。
- dznav 与锁定增强都涉及车端节点重启，逐项部署逐项验证，不要并行改动。

## 五、回滚

| 项 | 方法 |
|---|---|
| 车端大脑 | `~/ag_ws` 内为 colcon 覆盖安装，重跑 car_deploy 推旧版源码即可；黑匣子依赖 ag_core 同步回退 |
| pose 模型 | `deploy_pose_rknn.py --rollback`（.bak_pose_v2 自动恢复+重启） |
| dznav 循迹 | `dznav_deploy.py --host ... --rollback`（停 dznav → 按留档命令重启 legacy 两进程） |
| 锁定增强 | 车端 `.bak_20261017` 复制回去重新编译；参数级回滚：`momentum_comp_on / micro_search_on / predictor enable` 全置 false |

## 六、注意事项

1. 车端操作必须走标准 `remote_run.py` 规范（`01_车端大脑ag_integrate\tools\`），不得直接改车上原文件。
2. `dznav_deploy.py` / `car_deploy.py` 按脚本所在目录定位源码（`parents[1]/src`），包内分节结构已保持该相对依赖；从主仓库原位置运行同样有效。
3. src/ag_integrate 内 anchored_registration / ground_projection / evidential_map 为无人机端映射库（随包携带，车端无害不引用）；主仓库中 dznav、car_deploy 相关代码尚未提交 git，本包快照即为版本记录。
