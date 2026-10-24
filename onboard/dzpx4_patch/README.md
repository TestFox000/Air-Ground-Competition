# dzpx4_patch · px4_odom_convert 增强补丁（v2.2，2026-10-24）

部署唯一入口：`python tools/patch_deploy.py --host <Orin IP> --user duzhong`（备份→推送→colcon 重编→重启节点→验证横幅）。

本版（v2.2）在 v2.1 基础上的加固（全部来自静态审计 + 10-16/10-17 实飞日志证据）：
1. **NaN/非有限熔断**：SLAM 崩溃帧（NaN 位姿）旧版绕过全部三级防护直通 EKF（NaN 与任何比较恒 false ⇒ 落"正常放行"分支）；现按 L1 熔断丢帧，恢复需 86 帧稳定。
2. **armed 门控新鲜度**：/fmu/out/vehicle_status 断流时旧版 fail-open（armed 下跟丢大位移被当"搬运"放行）；现 `disarmedConfirmed()` 要求状态新鲜（默认 2s），过期按 armed 处理，health 行新增 `status_age_s` 字段。
3. **对齐帧单次重锚**：对齐完成帧旧版"先 bump 后 hold"错序双重锚（EKF 拿新 reset_counter 锚到旧位姿 ~0.2s）；现对齐帧 gate 锚点同步重置，一次到位。
4. jump_hold_m / jump_yaw_rad / jump_confirm_n 参数化（旧版硬编码却宣传可调）；timestamp_sample=0 回退 now()。
5. package.xml 补 nav_msgs/tf2_ros 依赖（CMakeLists 早已 find_package）。

机上验证（部署后看日志）：
- 启动横幅含 `Resilience layer v2` 与 `jump_guard(hold 0.50m ... confirm 8 frames)`；
- `[px4_odom_convert_health]` 行出现 `status_age_s=` 字段；
- 3s 周期健康行持续输出即 watchdog 数据源正常。

回滚：`patch_deploy.py --rollback`（按备份索引 + 指纹校验）。
