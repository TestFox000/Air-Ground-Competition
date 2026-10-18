# 2026-10-18 dzuav + live 同款 HUD 最优方案（决策记录）

> 本文档整理自 2026-10-18 会话长文（原始出处见 `docs/archive/20261018-会话原始长文-dzuav-live-HUD方案.txt`），
> 记录了"live 实时读数错误"问题的最佳修复方案设计与落地状态。

## 1. 背景与问题

- 第三方 **live 程序**（带 HUD 的实时检测工具）存在**读数错误**：预处理与 v4d 契约不符导致形近字符混淆（典型现场真值如 H9 被读成 I6、G6、J7 等），影响现场判读。
- 我方 dzuav 识别链路**已验证 106/106 全对**（三处契约已修复），但相机模式无 HUD、无画框，现场调试体验退化。

## 2. 方案对比

| 对比项 | A. 修 live 源码 | B. 裸换 dzuav | **C. dzuav + live 同款 HUD（选定）** |
|---|---|---|---|
| 识别正确性 | 未知代码，改动风险 | 已验证 106/106 | 已验证 106/106 |
| 实时可视化 | 有（原有） | 无（体验退化） | **有（同款 HUD + 画框）** |
| 源码掌控 | 无（黑盒） | 有 | 有 |
| 可维护/回滚 | 难 | 易 | 易（一键部署 + 回滚） |

**结论**：不修别人的黑盒程序（风险不可控），也不裸换我方链路（丢实时画面），而是给我方 dzuav 补上与原 live 程序**逐字符同格式**的实时 HUD，一步到位替代。

## 3. 技术设计（onboard/dzuav/src/main.cpp）

- **HUD 格式与原 live 程序逐字符一致**：`dets= cells= localized= fps=` + 画框 + 格号标注，用户零学习成本。
- **显示开关**：相机模式检测到 `DISPLAY`（VNC/桌面）自动开画面；无显示环境自动退回纯文本输出（远程/无头可用）。`DZUAV_VIEW=1/0` 可强制开/关。
- **性能**：D13 推理 90–150ms/帧（6–11fps），加 clone + 绘制 + imshow 约 10–20ms，fps 无实质影响。
- **代码改动点**：
  - `processFrame` 加 `FrameStats* stats` 与 `float fps_hud` 输出参数（默认参数，离线分支零改动），返回 `cv::Mat`（live 模式返回 visual）。
  - `runCamera`：DISPLAY 检测、fps EMA 滚动统计（α=0.1）、`imshow("dzuav live")` + `waitKey(1)`。
  - 窗口名用 `"dzuav live"`，HUD 行首加 `[dzuav]` 标识，与来源区分。
- **验证策略**：本地无 C++/TensorRT 环境 → 代码复查 + 上机部署工具三重验证（编译成功 / 进程存活 / HUD 行出现）+ 备份回滚兜底。

## 4. 一键部署工具 fix_live_hud.py

| 子命令 | 动作 |
|---|---|
| `--probe` | 抓 live 程序进程与完整 cmdline（供回滚）、检查板上 dzuav 源码完整性（5 cpp + CMakeLists）、DISPLAY/X11 socket 可用性、相机设备与分辨率 |
| `--deploy` | 备份板上 main.cpp → 推新 main.cpp + md5 校验 → 编译 → 二进制同步 → 停第三方 live 程序（启动命令存档）→ tmux 以 `DISPLAY=:0` 启动 dzuav camera → 验证 |
| `--verify` | capture-pane 统计 HUD 行（`dets=` 等）与 localized=1 占比 |
| `--rollback` | 恢复 main.cpp → 重编 → 同步 → 按备份 cmdline 原样重启原 live 程序 |

## 5. 验收判据

VNC 出现 `"dzuav live"` 窗口，对准 H7/H8/H9 区域——读数应显示 **H7/H8/H9**（不再 I6/G6/J7），且 localized 翻 1。不满意随时 `--rollback`。

## 6. 落地状态（截至 2026-10-18）

- ✅ main.cpp HUD 改造完成（HUD 格式与原程序逐字符一致，本地对拍 PASS）
- ✅ fix_live_hud.py 一键部署工具就绪
- ✅ 上机包 v5 已重打包（含 HUD 版 main.cpp + 纠错地图 v2）
- ⏳ 上机部署验证（需按第 5 节判据实测）

**恢复动作提醒**：主仓库本地的 `MapLocalizer.cpp` 曾被删除；板上副本完整，部署只需推送 main.cpp，不依赖本地该文件。
