# B 部署对账：仓库 main 的 dzpx4_patch vs 无人机 ~/dzpx4 现役

> 审查复核（2026-10-23）衍生发现：**「已修」≠「已部署」**——定位链咽喉件
> px4_odom_convert 的多处修复只存在于 Gitee main 的 `onboard/dzpx4_patch/`，
> 无人机 `~/dzpx4` 仍在跑旧版。本清单逐项列差（✅=已实测核验，⚠️=待链路恢复补核）。

## 差异清单

| # | 项 | Gitee main（onboard/dzpx4_patch/） | 无人机 ~/dzpx4 现役 | 判定 |
|---|---|---|---|---|
| 1 | **F1 时钟域 steadyNowS()** | ✅ 有（L225 定义，L309/353/440/720 四处使用，单调钟） | ✅ **无**（grep 零命中，EKF 风暴窗仍用 `this->now().seconds()` 墙钟） | **修复未上机**：混时钟域问题在机上仍活着 |
| 2 | **armed 判据** | ✅ L282 `== 2 \|\| == 4` | ✅ L254 `== 2` | 两版不一致；`==4` 语义待拍板（审查#7）。**部署 main 前必须先拍板**——否则把待确认判据带上机 |
| 3 | **nan_drop / z_src 前兆健康行** | ✅ 发射端 L470-474 产出（注释自称「比跳变更早的告警」） | ⚠️ 待核（预期无，与 steadyNowS 同批演进） | 前兆信号链（发射→watchdog 消费）整条未上机 |
| 4 | **localization_watchdog.py** | ✅ 有（402 行，日志尾随式，v2.1） | ✅ 全盘 find **不存在**（~/drone_tools 也无） | 定位质量 watchdog 整个未部署——EKF reset 风暴那类事故（14h 无人察觉）在机上仍无第二道防线 |

## 结论

无人机定位链运行的是**落后仓库至少一个修复批次**的版本。审查「已修 14 条」里的
F1 属于「仓库已修、机上未部署」——按同口径，机上真实的已修数要打折扣。

## 建议部署序（链路恢复后执行）

1. **先拍板审查#7**（`==4` 语义）：若判错，先在 main 修掉再走下一步；
2. 用既有 `tools/patch_deploy.py`（自动 备份→推送→强制重编→重启→验证→体检，
   带 `--rollback`）部署 main 的 dzpx4_patch 到无人机；
3. watchdog 随包部署（patch_deploy 已含）并确认 `~/drone_logs/` 健康行出现
   `nan_drop=`/`z_src=` 字段；
4. 部署后跑 `tools/check_stack_health.py` 体检收尾。

## 关联

- 审查复核全报告：`审查复核-20261023.md`（Gitee 个人档 20261023-bug-audit-verification）
- 三份修复补丁：`patches/D1-*.patch`、`D4-*.patch`、`C2-*.patch`（待审合入）
