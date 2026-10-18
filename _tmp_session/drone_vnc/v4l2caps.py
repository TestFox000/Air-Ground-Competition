import os, fcntl, struct, ctypes
from ctypes import c_uint32, c_char, Structure

class FmtDesc(Structure):
    _fields_ = [("index", c_uint32), ("type", c_uint32), ("flags", c_uint32),
                ("description", c_char*32), ("pixelformat", c_uint32),
                ("mbus_code", c_uint32), ("reserved", c_uint32*3)]

class FrmSize(Structure):
    _fields_ = [("index", c_uint32), ("pixel_format", c_uint32), ("type", c_uint32),
                ("width", c_uint32), ("height", c_uint32),
                ("max_w", c_uint32), ("max_h", c_uint32),
                ("step_w", c_uint32), ("step_h", c_uint32),
                ("reserved", c_uint32*2)]

VIDIOC_ENUM_FMT = 0xC0405602
VIDIOC_ENUM_FRAMESIZES = 0xC02C564A

def fourcc(v):
    s = struct.pack('<I', v)
    return ''.join(chr(b) if 32 <= b < 127 else '.' for b in s)

fd = os.open('/dev/video0', os.O_RDWR | os.O_NONBLOCK)
try:
    i = 0
    while True:
        d = FmtDesc(); d.index = i; d.type = 1
        try:
            fcntl.ioctl(fd, VIDIOC_ENUM_FMT, d)
        except OSError as e:
            print('formats:', i); break
        print('FMT', fourcc(d.pixelformat), d.description.decode(errors='replace').strip())
        j = 0
        while True:
            fs = FrmSize(); fs.index = j; fs.pixel_format = d.pixelformat
            try:
                fcntl.ioctl(fd, VIDIOC_ENUM_FRAMESIZES, fs)
            except OSError:
                break
            if fs.type == 1:  # discrete
                print('   SIZE', fs.width, 'x', fs.height)
            else:
                print('   STEP', fs.min_width, '-', fs.max_width, 'x', fs.min_height, '-', fs.max_height,
                      'step', fs.step_w, fs.step_h)
            j += 1
        i += 1
finally:
    os.close(fd)
