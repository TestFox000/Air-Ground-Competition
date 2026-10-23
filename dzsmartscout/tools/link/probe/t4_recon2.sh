#!/bin/bash
# t4 第二轮取证：域号真相源如何被真正使用 + 车侧部署契约核对（只读）
echo "=== A. dzros.env / dzlink.env 谁在 source ==="
for d in "$HOME/dzrobot" "$HOME/dzlink_tools"; do
  [ -d "$d" ] || continue
  echo "-- $d --"
  grep -rn "dzros.env\|dzlink.env" "$d" --include="*.sh" --include="*.py" --include="*.service" \
    2>/dev/null | grep -v "/install/\|/build/" | head -12 | sed 's/^/  /'
done

echo
echo "=== B. start_all.sh 头部 40 行（域号怎么进来）==="
if [ -r "$HOME/dzrobot/start_all.sh" ]; then
  sed -n '1,40p' "$HOME/dzrobot/start_all.sh" | sed 's/^/  /'
else
  echo "  (无 start_all.sh)"
fi

echo
echo "=== C. 车侧 .bashrc 里 ROS 相关行（含上下文）==="
if [ -r "$HOME/.bashrc" ]; then
  grep -nE "ROS_|DZ_|dzros" "$HOME/.bashrc" | sed 's/^/  /'
  echo "  -- 交互式守卫（决定 bash -lc 看不看得到上面那些）--"
  sed -n '1,20p' "$HOME/.bashrc" | grep -nE "case \\\$-|\\*i\\*|return|esac" | sed 's/^/  /'
fi

echo
echo "=== D. 车侧部署里 dzlink 契约（话题/QoS 硬编码处）==="
SRC="$HOME/dzrobot/src"
[ -d "$SRC" ] || SRC="$HOME/dzrobot"
grep -rn "drone_to_car\|car_to_drone\|/nav/waypoint" "$SRC" 2>/dev/null \
  | grep -v "/install/\|/build/\|\.pyc" | head -20 | sed 's/^/  /'
echo "-- QoS 定义处（RELIABLE/BEST_EFFORT 与上述话题的关系）--"
grep -rn -B2 -A4 "drone_to_car" "$SRC" 2>/dev/null \
  | grep -iE "qos|reliab|best_effort|history|depth|keep_last" | head -20 | sed 's/^/  /'

echo
echo "=== E. 车侧 git 状态（零改动基线的硬证据）==="
for r in "$HOME/dzrobot" "$HOME/dzrobot_v2"; do
  [ -d "$r/.git" ] && { echo "-- $r --"; git -C "$r" status --porcelain 2>&1 | head -20 | sed 's/^/  /'; echo "  (空 = 工作区干净)"; } \
                    || echo "-- $r: 不是 git 仓库 --"
done

echo
echo "=== F. 车上已存在哪些 /tmp 探针（历史遗留）==="
ls -l /tmp/*.sh /tmp/*.py /tmp/dzlink* 2>/dev/null | sed 's/^/  /' || echo "  (无)"
echo "-- 车上最近的 ~/.ros 目录 --"
ls -lt "$HOME/.ros" 2>/dev/null | head -6 | sed 's/^/  /'

echo "### t4_recon2 done on $(hostname)"
