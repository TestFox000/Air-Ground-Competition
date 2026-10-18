#!/bin/bash
cd ~/dzuav/models || exit 1
if [ -f 'hbb_detector_d13_20261016(1).onnx' ]; then mv -f 'hbb_detector_d13_20261016(1).onnx' hbb_detector_d13_20261016.onnx; echo renamed; fi
sha256sum cell_classifier_v4d_20261015.onnx hbb_detector_d13_20261016.onnx
ls -la
echo ---BIN
ls -la ~/ag_test/dzuav/build/dzuav 2>/dev/null || echo NO_BIN_AC_TEST
ls ~/ag_test/dzuav 2>/dev/null
