set +e
echo "== host =="; hostname; date; uptime
echo "== tty devices =="; ls -l /dev/ttyACM* /dev/ttyUSB* 2>&1
echo "== by-id =="; ls -l /dev/serial/by-id/ 2>&1
echo "== lsusb =="; lsusb 2>&1
echo "== udev ttyACM0 =="; udevadm info -q property -n /dev/ttyACM0 2>&1 | grep -E 'ID_MODEL=|ID_VENDOR=|ID_SERIAL=|DEVLINKS=|ID_USB_INTERFACE_NUM='
echo "== udev ttyACM1 =="; udevadm info -q property -n /dev/ttyACM1 2>&1 | grep -E 'ID_MODEL=|ID_VENDOR=|ID_SERIAL=|ID_USB_INTERFACE_NUM='
echo "== procs =="; pgrep -af 'MicroXRCE|px4|mavros|mavlink|QGround|gcs' 2>&1
echo "== who owns tty =="; (sudo -n fuser -v /dev/ttyACM0 /dev/ttyACM1 2>&1) || echo "no sudo/fuser"
echo "== dmesg usb tail =="; (sudo -n dmesg 2>/dev/null || dmesg 2>/dev/null) | grep -iE 'cdc_acm|ttyACM|usb [0-9]-|px4' | tail -30
echo "== dmesg last =="; (sudo -n dmesg 2>/dev/null || dmesg 2>/dev/null) | tail -15
echo "== serial cfg =="; stty -F /dev/ttyACM0 -a 2>&1 | head -5
echo "== ROS_DOMAIN =="; grep -h ROS_DOMAIN_ID ~/.bashrc ~/dzlink_tools/dzlink.env ~/dzpx4/*.env 2>/dev/null
echo "== ros topics =="; source /opt/ros/humble/setup.bash 2>/dev/null; source ~/dzpx4/install/setup.bash 2>/dev/null; export ROS_DOMAIN_ID=118; timeout 10 ros2 topic list 2>&1 | head -50
echo "== ros nodes =="; timeout 10 ros2 node list 2>&1 | head -30
echo "== tmp logs =="; ls -lt /tmp/*.log 2>/dev/null | head -10
