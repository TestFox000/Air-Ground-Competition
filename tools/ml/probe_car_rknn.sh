#!/usr/bin/env bash
# 只读探测：车端 RKNN 运行时 + 官方 8400 后处理代码位置
set +e
echo "=== rknnlite ==="
python3 -c "from rknnlite.api import RKNNLite; print('rknnlite OK')"
echo "=== 解码 8400 的源码 ==="
grep -rln "8400" "$HOME/dzrobot/src" 2>/dev/null | head
echo "=== 模型目录 ==="
ls "$HOME/dzrobot/src/rknn_pt/model/" 2>/dev/null
echo "=== rknn 相关 python 包 ==="
python3 -c "import rknnlite, os; print(os.path.dirname(rknnlite.__file__))" 2>/dev/null
pip3 list 2>/dev/null | grep -i rknn
