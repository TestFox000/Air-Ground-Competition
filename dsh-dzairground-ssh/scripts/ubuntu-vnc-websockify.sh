#!/usr/bin/env bash
# ============================================================================
# Ubuntu 端：TigerVNC + websockify 完整安装脚本
#
# 目标：让任何机器上的 DSH「VNC 连接器」面板能连上这台 Ubuntu 的桌面：
#   连接器(noVNC) ──ws──▶ websockify:6080 ──tcp──▶ TigerVNC :1 (5901)
#
# 两种暴露方式（见第 6 步，二选一）：
#   A. 局域网明文  ：防火墙放行 6080，连接器填 ws://<server>:6080/websockify
#   B. SSH 隧道（推荐）：websockify 只绑 127.0.0.1、防火墙不放行，
#      笔记本上先开隧道再连 ws://127.0.0.1:6080/websockify —— 全程加密，
#      桌面密码不出网，也不依赖 websockify 自己（几乎不维护）的 TLS 支持。
#
# 用法：在 Ubuntu 机器上 bash 执行。vncpasswd 一步是交互式的（设置 VNC 密码，
#      与 Linux 登录密码是两回事；最多 8 位有效）。
# ============================================================================
set -euo pipefail

USER_NAME="$(whoami)"
HOME_DIR="$HOME"

echo "==> 1/6 安装 TigerVNC + websockify（novnc 可选，仅用于自带网页客户端）"
sudo apt update
sudo apt install -y tigervnc-standalone-server tigervnc-common websockify

echo "==> 2/6 设置 VNC 密码（交互式）"
mkdir -p "$HOME_DIR/.vnc"
if [ -f "$HOME_DIR/.vnc/passwd" ]; then
  echo "    已存在 ~/.vnc/passwd；如需更换请手动运行 vncpasswd。"
else
  vncpasswd
fi

echo "==> 3/6 安装 vncserver@:1 systemd 单元"
# -localhost no：TigerVNC 新版默认只绑回环，websockify 在同机转发也需要它可被
#   本机 TCP 连接（回环即可），但为了 A 方案兼容与少一类故障，这里显式放开到
#   本机所有接口；若走 B 方案（SSH 隧道）可改成 -localhost yes。
sudo tee /etc/systemd/system/vncserver@:1.service >/dev/null <<EOF
[Unit]
Description=TigerVNC server for display :1
After=syslog.target network.target

[Service]
Type=forking
User=$USER_NAME
Group=$USER_NAME
WorkingDirectory=$HOME_DIR
ExecStartPre=/bin/sh -c '/usr/bin/vncserver -kill :1 > /dev/null 2>&1 || :'
ExecStart=/usr/bin/vncserver :1 -geometry 1920x1080 -depth 24 -localhost no -SecurityTypes VncAuth -PasswordFile $HOME_DIR/.vnc/passwd
ExecStop=/usr/bin/vncserver -kill :1
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "==> 4/6 启动 VNC 服务"
sudo systemctl daemon-reload
sudo systemctl enable --now vncserver@:1

echo "==> 5/6 安装 websockify systemd 单元（只绑 127.0.0.1 —— 两个方案都安全）"
# --web 指向 apt 的 novnc 包，只是让"别人用浏览器直接开 http://server:6080/vnc.html"
# 时也有页面；DSH 连接器自带 noVNC，可以去掉 --web 参数。
sudo tee /etc/systemd/system/websockify-5901.service >/dev/null <<EOF
[Unit]
Description=WebSocket-to-TCP proxy for TigerVNC :1
After=network.target vncserver@:1.service

[Service]
ExecStart=/usr/bin/websockify 6080 127.0.0.1:5901
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now websockify-5901

echo "==> 6/6 暴露方式（二选一）"
echo "    A. 局域网明文："
echo "         sudo ufw allow 6080/tcp"
echo "        连接器填：ws://$(hostname -I | awk '{print $1}'):6080/websockify"
echo "    B. SSH 隧道（推荐，防火墙不放行）："
echo "        笔记本执行：ssh -N -L 6080:127.0.0.1:6080 $USER_NAME@<server>"
echo "        连接器填：ws://127.0.0.1:6080/websockify"
echo
echo "验证："
echo "    systemctl status vncserver@:1 websockify-5901"
echo "    ss -lnt | grep -E '5901|6080'"
echo "    # 从另一台机器（B 方案需先开隧道）"
echo "    python3 - <<'PY'  # 有 websockify 自带模块，即可验证 ws 端点活着"
echo "import socket; s=socket.create_connection(('127.0.0.1',6080),3); s.close(); print('6080 reachable')"
echo "PY"
