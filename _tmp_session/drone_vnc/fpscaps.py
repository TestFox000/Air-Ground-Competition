import os, fcntl, struct
from ctypes import c_uint32, Structure

class FrmIval(Structure):
    _fields_ = [("index", c_uint32), ("pixel_format", c_uint32), ("width", c_uint32), ("height", c_uint32),
                ("type", c_uint32), ("num", c_uint32), ("den", c_uint32),
                ("max_num", c_uint32), ("max_den", c_uint32), ("step_num", c_uint32), ("step_den", c_uint32),
                ("reserved", c_uint32*2)]

VIDIOC_ENUM_FRAMEINTERVALS = 0xC034564B
MJPG = 0x47504A4D

def fourcc(v):
    return ''.join(chr(b) if 32 <= b < 127 else '.' for b in struct.pack('<I', v))

fd = os.open('/dev/video0', os.O_RDWR | os.O_NONBLOCK)
try:
    for (w, h) in [(1920,1200),(1920,1080),(1280,960),(1280,720),(896,1200)]:
        out = []
        i = 0
        while True:
            fi = FrmIval(); fi.index = i; fi.pixel_format = MJPG; fi.width = w; fi.height = h
            try:
                fcntl.ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, fi)
            except OSError:
                break
            if fi.type == 1:
                if fi.num: out.append('%.1f' % (fi.den / fi.num))
                else: out.append('inf')
            else:
                out.append('step %.1f-%.1f' % (fi.den/fi.max_num if fi.max_num else 0, fi.den/fi.num if fi.num else 0))
            i += 1
        print(w, 'x', h, '->', ','.join(out) if out else 'none')
finally:
    os.close(fd)
