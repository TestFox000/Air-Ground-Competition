#!/bin/bash
echo ---YAML; find /home/duzhong -maxdepth 5 \( -name "data.yaml" -o -name "*.yaml" -path "*dataset*" -o -name "*.names" \) 2>/dev/null | head -10
echo ---GREP_PY; grep -rln "cell_label" ~/dzuav_deploy ~/ag_test ~/dzuav/src 2>/dev/null | head -10
echo ---DATASET; find /home/duzhong -maxdepth 4 -type d -iname "*dataset*" 2>/dev/null | head -10
echo ---EXPORT; find /home/duzhong -maxdepth 5 -name "export*.py" 2>/dev/null | head -10
