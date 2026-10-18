import sys
from pyulog import ULog
p = sys.argv[1]
u = ULog(p)
print("file:", p)
print("start_us:", u.start_timestamp, "last_us:", u.last_timestamp, "duration_s:", (u.last_timestamp - u.start_timestamp)/1e6)
print("msg_info:", u.msg_info)
print("datasets:", len(u.data_list))
for d in u.data_list:
    n = 0
    for k,v in d.data.items():
        n = len(v); break
    print("DS", d.name, "rows", n, "cols", len(d.data_fields))
