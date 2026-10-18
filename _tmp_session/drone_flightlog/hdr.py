import sys
from pyulog import ULog
p = sys.argv[1] if len(sys.argv) > 1 else '/home/duzhong/dzpx4/log_1_2026-10-15-17-10-50.ulg'
try:
    u = ULog(p, parse_header_only=True)
    print('start', u.start_timestamp, 'last', u.last_timestamp, 'dur_s', (u.last_timestamp - u.start_timestamp)/1e6)
    print('info', u.msg_info)
except Exception as e:
    print('HEADER FAIL:', repr(e))
