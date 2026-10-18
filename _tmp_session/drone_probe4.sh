#!/bin/bash
echo ---SHA_MODELS
sha256sum ~/dzuav/models/cell_classifier_v4d_20261015.onnx ~/dzuav_deploy/air_ground/models/release/hbb_detector_d13_20261016.onnx ~/dzuav_deploy/air_ground/models/release/cell_classifier_v4c_ft_20261010.onnx 2>/dev/null
echo ---SHA_ENGINES
sha256sum ~/dzuav/engines/*.engine ~/dzuav/engines_d13/*.engine 2>/dev/null
echo ---START_SCRIPTS
grep -rnE "dzuav|engine" ~/start_drone.sh ~/start_fastlio.sh ~/start_odom_v21.sh 2>/dev/null | head -20
echo ---AUTOSTART
ls ~/dzlink_tools 2>/dev/null | head; grep -rnE "dzuav|engine" ~/dzlink_tools/*.sh 2>/dev/null | head -20
echo ---PROC
pgrep -af "dzuav|dzlink" | head -10
echo ---README; sed -n '1,40p' ~/dzuav/src/cell_classifier/README.md 2>/dev/null
echo ---VIEW; ls -la ~/dzuav_deploy/air_ground/*.jpg
