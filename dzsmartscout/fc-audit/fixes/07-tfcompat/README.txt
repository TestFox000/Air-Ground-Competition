AGC-16 补丁 07-tfcompat —— transform_fusion 的 tf2_ros.transformations 兜底修复
==============================================================================

卡号   : AGC-16（AGENTS ns = agc-1024）
日期   : 2026-10-24
作者   : dl-drone（离线件；未连无人机、未部署）
状态   : 源文件已在工作区改好并通过离线单测；机上部署待现场窗口

------------------------------------------------------------------------------
一、为什么需要这个补丁
------------------------------------------------------------------------------
机端 /localization 从来没有发布者。链条（行号是改动前的）：

  transform_fusion.py:13   import tf_transformations            -> 机上没有这个包
  transform_fusion.py:15   from tf2_ros import transformations  -> ROS2 没有这个模块
  => 节点 import 阶段就抛 ImportError，进程直接死
  => 它是 /localization 的唯一发布者
  => 车的 px4_odom_convert 订阅一个不存在的话题，PX4 永远拿不到外部视觉

现场观感与“一切正常”几乎无差别：FAST-LIO 在跑、点云与 RViz 正常，只有定位话题不存在。

------------------------------------------------------------------------------
二、补丁内容（3 个文件）
------------------------------------------------------------------------------
目标机路径（示例；apply.sh 的 DEST 默认 ~/dzpx4）：

  <DEST>/src/FAST_LIO_LOCALIZATION_ROS2/scripts/tf_compat.py         （新增）
  <DEST>/src/FAST_LIO_LOCALIZATION_ROS2/scripts/transform_fusion.py  （只改 import 兜底）
  <DEST>/src/FAST_LIO_LOCALIZATION_ROS2/CMakeLists.txt               （install 多一行）

本地镜像（apply.sh 直接从这里拷）：
  files/dzpx4/src/FAST_LIO_LOCALIZATION_ROS2/...

1) tf_compat.py（新增，只依赖 numpy + math）
   只实现 transform_fusion 实际调用的 4 个函数：
     translation_matrix / quaternion_matrix / translation_from_matrix / quaternion_from_matrix
   四元数一律 [x, y, z, w]（w 在最后，与 geometry_msgs/Quaternion 字段序一致）；
   quaternion_from_matrix 统一规范化为 w >= 0（q 与 -q 表示同一旋转）。

2) transform_fusion.py（只改 import 链；其余逻辑一字未动）

     try:
         import tf_transformations          # 机上有这个包就照原样用
     except ImportError:
         try:
             import tf_compat as tf_transformations
         except ImportError:                # 从别的 cwd 当脚本拉起时兜底
             sys.path.insert(0, <脚本同级目录>)
             import tf_compat as tf_transformations

3) CMakeLists.txt
   install(PROGRAMS ...) 里加一行 scripts/tf_compat.py。
   这一行不能省：install(PROGRAMS) 是逐个文件装进 lib/<pkg>/ 的，不列就不会装
   => 运行期 import tf_compat 仍然失败（AGC-16 评审的 R2）。

------------------------------------------------------------------------------
三、怎么用
------------------------------------------------------------------------------
  bash apply.sh                     # 默认 -d ~/dzpx4
  bash apply.sh -d /path/to/dzpx4   # 指定目标根（其下要有 src/FAST_LIO_LOCALIZATION_ROS2）

脚本会：备份被覆盖的原文件到 <DEST>/.tfcompat-backup-<时间戳>/ -> 拷贝 -> 四项校验
（import tf_compat 成功 / 兜底串在位 / 旧兜底已清除 / CMake 已列 tf_compat）。

然后手动（脚本不代替你执行）：

  cd <DEST> && colcon build --packages-select fast_lio_localization && source install/setup.bash

------------------------------------------------------------------------------
四、验证（部署后）
------------------------------------------------------------------------------
  source /opt/ros/humble/setup.bash && source <DEST>/install/setup.bash

  1) 节点起得来（原来这里是 ImportError 死）：应打印
     “Transform Fusion Node Initialized”
  2) ros2 topic list | grep localization     # 应有 /localization
  3) ros2 topic hz /localization             # 期望稳定频率（约与 /Odometry 同频）
  4) ros2 topic echo /localization --once    # 位姿非全 0

离线自验（不需要机子，任何平台）：
  python -B -X utf8 -m unittest discover -s dzsmartscout/src/FAST_LIO_LOCALIZATION_ROS2/scripts -p test_tf_compat.py
  期望：Ran 32 tests / OK（0 skip）

------------------------------------------------------------------------------
五、回滚
------------------------------------------------------------------------------
  bash apply.sh --rollback -d <DEST>
  # 还原 transform_fusion.py + CMakeLists.txt，并删掉新增的 tf_compat.py
  然后重新 colcon build --packages-select fast_lio_localization。

------------------------------------------------------------------------------
六、边界与已知未修（本补丁不处理）
------------------------------------------------------------------------------
* 同目录另外两个脚本有同因问题，本补丁不碰（Lead 已另立卡）：
    global_localization.py   裸 import tf_transformations，无兜底
    publish_initial_pose.py  还用 tf_transformations.quaternion_from_euler
* 不改 launch / 不改 config / 不改 FAST_LIO 其它源码 / 不改行为（数值与 tf_transformations 在位时一致）。
* 不改雷达（D1 carrier=0）与 PX4 插板（D3）——那两条是物理动作。

------------------------------------------------------------------------------
七、为什么做成补丁包而不是直接提交
------------------------------------------------------------------------------
dzsmartscout/src/FAST_LIO_LOCALIZATION_ROS2 在本仓里是未初始化的子模块
（父仓只有 gitlink 3a005b96…，目录内没有 .git）=> 父仓 git add 该路径会报
“is in submodule”，源文件改动进不了父仓提交、也只在工作区。
所以按本仓既有形态（fc-audit/fixes/01..06）落成可审、可复现的补丁包；
子模块日后若初始化，再把这三个文件正式提交到子模块上游。

（编号说明：Lead 原话给的目录名是 02-tfcompat，但 02 已被 02-px4-path-config 占用，
  故顺延为 07-tfcompat；内容与裁决一致。）

关联证据（本仓内，只读）：
  _tmp_session/agc-1024/e1/REVIEW_DL1.md                                    D3 根因与三节点全死的现场记录
  dzsmartscout/src/FAST_LIO_LOCALIZATION_ROS2/scripts/test_tf_compat.py    32 条离线单测
  docs/30_任务卡库/在办/agc-1024/AGC-16-TFCOMPAT-20261024.md               本卡执行记录
