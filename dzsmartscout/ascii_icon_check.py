# -*- coding: utf-8 -*-
"""Rasterize the airplane SVG paths onto a 48x48 grid and print ASCII art,
so the shape can be inspected without an image-capable model."""
import math

GRID = 48
SCALE = GRID / 24.0
STROKE = 1.7  # svg stroke width in viewBox units
R = STROKE / 2.0 * SCALE + 0.4  # pixel radius around polyline

def cubic(p0, p1, p2, p3, n=48):
    pts = []
    for i in range(n + 1):
        t = i / n
        mt = 1 - t
        x = mt**3*p0[0] + 3*mt*mt*t*p1[0] + 3*mt*t*t*p2[0] + t**3*p3[0]
        y = mt**3*p0[1] + 3*mt*mt*t*p1[1] + 3*mt*t*t*p2[1] + t**3*p3[1]
        pts.append((x, y))
    return pts

def line(p0, p1, n=24):
    return [(p0[0] + (p1[0]-p0[0])*i/n, p0[1] + (p1[1]-p0[1])*i/n) for i in range(n+1)]

# ---- geometry ----
polylines = []
# fuselage (two cubic segments right side, mirrored left)
polylines.append(cubic((12,2.2),(13.3,3.8),(13.9,5.9),(13.9,8.3)))
polylines.append(cubic((13.9,8.3),(13.9,11.7),(13.6,15.2),(12.8,18)))
polylines.append(cubic((12.8,18),(12.5,19.5),(12.4,20.2),(12,21.8)))
polylines.append(cubic((12,21.8),(11.5,20.2),(11.6,19.5),(11.2,18)))
polylines.append(cubic((11.2,18),(10.4,15.2),(10.1,11.7),(10.1,8.3)))
polylines.append(cubic((10.1,8.3),(10.1,5.9),(10.7,3.8),(12,2.2)))
# wings
polylines.append(line((9.8,11),(2.9,13.9)) + line((2.9,13.9),(3.4,15.5)) + line((3.4,15.5),(10.5,14.2)) + line((10.5,14.2),(9.8,11)))
polylines.append(line((14.2,11),(21.1,13.9)) + line((21.1,13.9),(20.6,15.5)) + line((20.6,15.5),(13.5,14.2)) + line((13.5,14.2),(14.2,11)))
# tailplane bar
polylines.append(line((7.1,19.1),(16.9,19.1)))

grid = [[' '] * GRID for _ in range(GRID)]

def put(px, py):
    if 0 <= px < GRID and 0 <= py < GRID:
        grid[py][px] = '#'

# strokes
for pl in polylines:
    for (x, y) in pl:
        cx, cy = x*SCALE, y*SCALE
        r = math.ceil(R)
        for dy in range(-r, r+1):
            for dx in range(-r, r+1):
                if dx*dx + dy*dy <= R*R:
                    put(int(round(cx))+dx, int(round(cy))+dy)

# cockpit: filled circle r=1.2
cx, cy = 12*SCALE, 7.6*SCALE
cr = 1.2*SCALE
for dy in range(-math.ceil(cr)-0 and -4, 5):
    for dx in range(-4, 5):
        if dx*dx + dy*dy <= cr*cr:
            put(int(round(cx))+dx, int(round(cy))+dy)

print('    ' + ''.join(str(i // 10 % 10) for i in range(GRID)))
print('    ' + ''.join(str(i % 10) for i in range(GRID)))
for row in range(GRID):
    print(f'{row:3d} ' + ''.join(grid[row]))

# connectivity checks
def sample_ys(pl):
    return [(round(y*SCALE), round(x*SCALE)) for (x, y) in pl]
print('\n== connectivity checks ==')
# wing root vs fuselage left edge at same y
for name, pl in [('left wing', polylines[6]), ('right wing', polylines[7])]:
    ys = [int(round(y*SCALE)) for (_, y) in [(x,y) for (x,y) in pl[:2]]]
    print(name, 'root segment rows', ys)
# tailplane crosses fuselage?
mid_y = int(round(19.1*SCALE))
row = grid[mid_y]
fuselage_hit = any(c == '#' for c in row[20:30])
bar_hit = any(c == '#' for c in row[14:22])
print(f'tailplane row {mid_y}: fuselage pixels in x20..29 = {fuselage_hit}, bar pixels x14..21 = {bar_hit}')
