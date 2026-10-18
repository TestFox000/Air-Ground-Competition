#!/bin/bash
echo ---ENUM; grep -rn "enum class ObjectClass" -A 20 ~/dzuav/include ~/dzuav/src ~/ag_test/dzuav 2>/dev/null | head -30
echo ---NAMES; grep -rn "cell_label\|target_blue\|obstacle\|mine\|car" ~/dzuav/include ~/dzuav/src 2>/dev/null | head -30
echo ---META; ls ~/dzuav_deploy/air_ground/models/release/ 2>/dev/null; find ~/dzuav_deploy -name "*.json" -path "*model*" 2>/dev/null | head
echo ---CONST; find ~/dzuav_deploy -name "constants.py" -o -name "classes*.txt" -o -name "*.names" 2>/dev/null | head
echo ---ONNXMETA; python3 - <<'EOF'
import onnx
for p in ["/home/duzhong/dzuav/models/hbb_detector_d13_20261016.onnx"]:
    m = onnx.load(p, load_external_data=False)
    print("---", p)
    print("inputs:", [(i.name, [d.dim_value for d in i.type.tensor_type.shape.dim]) for i in m.graph.input])
    print("outputs:", [(o.name, [d.dim_value for d in o.type.tensor_type.shape.dim]) for o in m.graph.output])
    print("meta:", {p.key: p.value[:80] for p in m.metadata_props})
EOF
