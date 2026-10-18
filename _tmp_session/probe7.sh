set +e
echo "== count of agents =="; pgrep -x MicroXRCEAgent | wc -l
echo "== io loop (pid + rchar + wchar every 2s) =="
for i in 1 2 3 4 5 6; do
  P=$(pgrep -x MicroXRCEAgent | head -1)
  if [ -n "$P" ]; then
    R=$(awk '/^rchar/{print $2}' /proc/$P/io 2>/dev/null)
    W=$(awk '/^wchar/{print $2}' /proc/$P/io 2>/dev/null)
    E=$(ps -o etimes= -p $P 2>/dev/null | tr -d ' ')
    echo "t$i pid=$P etime=${E}s rchar=$R wchar=$W"
  else echo "t$i no agent"; fi
  sleep 2
done
echo "== start_drone.sh head 40 =="; sed -n '1,40p' /home/duzhong/start_drone.sh
echo "== start_drone.sh env sourcing =="; grep -n -E 'source|export|DOMAIN' /home/duzhong/start_drone.sh
echo "== ~/.bashrc ROS lines =="; grep -n 'ROS_DOMAIN_ID\|ROS_' /home/duzhong/.bashrc | head
