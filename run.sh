#!/usr/bin/env bash
# Linux / macOS 一键入口；用法与 run.py 一致：./run.sh demo
set -euo pipefail
cd "$(dirname "$0")"
exec python3 run.py "$@"
