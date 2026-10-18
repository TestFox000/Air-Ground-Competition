#!/usr/bin/env bash
# =============================================================================
# check_install.sh —— 确认 build_ws.sh 到底装出了什么
# =============================================================================
#     bash "/mnt/c/Users/Ran/Desktop/无人车/src-car/check_install.sh"
# =============================================================================

set +e
BOLD=$'\033[1m'; RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; CYN=$'\033[36m'; RST=$'\033[0m'
ok()   { printf "  ${GRN}✓${RST} %s\n" "$1"; }
bad()  { printf "  ${RED}✗${RST} %s\n" "$1"; }
warn() { printf "  ${YEL}!${RST} %s\n" "$1"; }
info() { printf "  ${CYN}·${RST} %s\n" "$1"; }
sec()  { printf "\n${BOLD}=== %s ===${RST}\n" "$1"; }

WS="$HOME/ws"

printf "${BOLD}检查工作空间装出了什么${RST}\n"

# ---------------------------------------------------------------- 1. install 目录
sec "1. $WS/install 内容"
if [ -d "$WS/install" ]; then
  ls -la "$WS/install" | sed 's/^/  /'
else
  bad "$WS/install 不存在 —— 编译没成功，或还没跑过 build_ws.sh"
fi

sec "2. 各包是否装出 local_setup.bash"
for p in serial dzmission; do
  if [ -f "$WS/install/$p/share/$p/local_setup.bash" ]; then
    ok "$p 已正确安装"
  elif [ -d "$WS/install/$p" ]; then
    warn "$p 目录存在但缺 share/$p/local_setup.bash"
    find "$WS/install/$p" -maxdepth 3 -type d 2>/dev/null | head -8 | sed 's/^/      /'
  else
    bad "$p 未安装"
  fi
done

# ---------------------------------------------------------------- 3. 编译日志
sec "3. 最近一次编译日志"
if [ -d "$WS/log/latest_build" ]; then
  ls "$WS/log/latest_build" | sed 's/^/  /'
  printf "\n  各包的编译结果：\n"
  for d in "$WS/log/latest_build"/*/; do
    [ -d "$d" ] || continue
    n=$(basename "$d")
    if [ -f "$d/stderr.log" ] && [ -s "$d/stderr.log" ]; then
      bad "$n 有 stderr 输出（见下）"
    else
      ok "$n 无 stderr"
    fi
  done
else
  bad "$WS/log/latest_build 不存在 —— 从没跑过 colcon build"
fi

# ---------------------------------------------------------------- 4. 报错摘要
sec "4. ★ 编译错误摘要（关键）"
FOUND=0
if [ -d "$WS/log/latest_build" ]; then
  while IFS= read -r f; do
    if [ -s "$f" ]; then
      printf "\n  --- %s ---\n" "${f#$WS/log/latest_build/}"
      grep -iE 'error|错误|undefined reference|No such file|fatal' "$f" \
        | head -25 | sed 's/^/      /'
      FOUND=1
    fi
  done < <(find "$WS/log/latest_build" -name 'stderr.log' 2>/dev/null)
fi

if [ "$FOUND" = "0" ]; then
  if [ -d "$WS/log/latest_build" ]; then
    ok "没有找到编译错误 —— 那可能是某种安装/路径问题"
  fi
fi

# ---------------------------------------------------------------- 5. 源文件是否就位
sec "5. $WS/src 里的包"
if [ -d "$WS/src" ]; then
  for d in "$WS/src"/*/; do
    [ -d "$d" ] || continue
    n=$(basename "$d")
    px="no"; cm="no"
    [ -f "$d/package.xml" ] && px="yes"
    [ -f "$d/CMakeLists.txt" ] && cm="yes"
    printf "  %-16s package.xml=%-4s CMakeLists=%s\n" "$n" "$px" "$cm"
  done
else
  bad "$WS/src 不存在"
fi

# ---------------------------------------------------------------- 6. colcon 能否发现
sec "6. colcon 能否发现包"
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -d "$WS/src" ]; then
  cd "$WS" || exit 1
  colcon list 2>&1 | sed 's/^/  /'
fi

sec "汇总"
printf "  请把 ${BOLD}第 4 节（编译错误摘要）${RST} 的完整输出贴回给助手。\n"
printf "  如果第 4 节是空的，也请说明 —— 那意味着编译其实过了，问题在别处。\n\n"
exit 0
