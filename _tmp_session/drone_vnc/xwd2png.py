import struct, sys
from PIL import Image
p = sys.argv[1]; out = sys.argv[2]
b = open(p,'rb').read()
(hdr,) = struct.unpack('>I', b[0:4])
(w,h) = struct.unpack('>II', b[16:24])
bpp, bpl = struct.unpack('>II', b[44:52])
byte_order, = struct.unpack('>I', b[28:32])
rmask, gmask, bmask = struct.unpack('>III', b[56:68])
stride = bpl // w
print('hdr',hdr,'w',w,'h',h,'bpp',bpp,'bpl',bpl,'stride',stride,'order',byte_order,'masks',hex(rmask),hex(gmask),hex(bmask))
raw = b[hdr:hdr + bpl*h]
if stride == 4 and rmask == 0xff0000 and byte_order == 0:
    mode = 'BGRX'
elif stride == 4 and rmask == 0xff:
    mode = 'XRGB'
elif stride == 3:
    mode = 'BGR'
else:
    raise SystemExit('unsupported layout')
im = Image.frombytes('RGB', (w,h), raw, 'raw', mode, bpl, 1)
im.save(out)
print('saved', out, im.size)
