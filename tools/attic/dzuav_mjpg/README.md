# dzuav_mjpg —— 归档(2026-10-25 未接线审计定案)

机上 13 类部署线的原始状态收编(8cbfbb31),含板载 CellGrid/RoutePlanner 规划线。
归档原因(三线分叉已收敛到 `onboard/dzuav` 本体):

1. **规划职责已迁移**:CellGrid/RoutePlanner 板载规划被 Python sidecar
   (`tools/orin_plan_sidecar.py`)+ mission_router 取代,无任何调用方传
   matrix_json/route_json/path_file;
2. **参数契约落后**:本目录 main.cpp 的 port 恒取 argv[4](旧序),而
   `onboard/dzuav` 已含 8df968f8 参数序实测修复(image 形态=[image,png,hbb,cell,port?,outdir?]);
3. **不在部署编排线**:orin_deploy.py 只推/编 `onboard/dzuav`,本目录靠机上
   残留二进制存活(orin_run_dzuav_d13.sh 引用 `~/dzuav_deploy/.../dzuav_mjpg/build/dzuav`),
   10-25 部署实测现役为 `onboard/dzuav`。

相机后端差异存证:本目录为 V4L2 直开,本体 `onboard/dzuav` 为 GStreamer pipeline
(2026-10-25 机上实测通过)。复活前先 diff 本体再动手。
