#!/usr/bin/env python3
# 生成 DDA Voxel Grid Traversal 封面图 + 示意图
# 可视化一条射线在 2D 体素网格中的 Amanatides-Woo 遍历路径
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 720

def render_2d_dda(origin, dirn, tmax, N, cell_px):
    """2D 版 DDA，返回被穿过的 cell 列表。"""
    x = int(np.floor(origin[0])); y = int(np.floor(origin[1]))
    if not (0 <= x < N and 0 <= y < N): return []
    stepX = 1 if dirn[0] > 0 else -1
    stepY = 1 if dirn[1] > 0 else -1
    tDX = abs(1.0/dirn[0]) if dirn[0] != 0 else float('inf')
    tDY = abs(1.0/dirn[1]) if dirn[1] != 0 else float('inf')
    nbX = (x+1) if stepX > 0 else x
    nbY = (y+1) if stepY > 0 else y
    tMX = (nbX - origin[0])/dirn[0] if dirn[0] != 0 else float('inf')
    tMY = (nbY - origin[1])/dirn[1] if dirn[1] != 0 else float('inf')
    cells = []
    while True:
        if not (0 <= x < N and 0 <= y < N): break
        cells.append((x, y))
        if tMX > tmax and tMY > tmax: break
        if tMX < tMY:
            x += stepX; tMX += tDX
        else:
            y += stepY; tMY += tDY
    return cells

N = 16
origin = (1.2, 1.3)
dirn = (3.0, 1.0)
tmax = 12.0
cells = render_2d_dda(origin, dirn, tmax, N, 1)

# 布局
margin = 80
grid_size = 560
cell = grid_size // N
origin_px = (margin + 40, margin + 20)

img = Image.new("RGB", (W, H), (13, 17, 28))
d = ImageDraw.Draw(img)

# 标题
try:
    font_title = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 46)
    font_sub   = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
    font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
except:
    font_title = font_sub = font_small = ImageFont.load_default()

d.text((margin, 24), "Amanatides-Woo 3D-DDA", font=font_title, fill=(240, 244, 255))
d.text((margin, 82), "Voxel Grid Traversal — 体素网格光线遍历", font=font_sub, fill=(150, 165, 190))

# 网格
for i in range(N+1):
    d.line([(origin_px[0], origin_px[1]+i*cell), (origin_px[0]+grid_size, origin_px[1]+i*cell)], fill=(45, 52, 68), width=1)
    d.line([(origin_px[0]+i*cell, origin_px[1]), (origin_px[0]+i*cell, origin_px[1]+grid_size)], fill=(45, 52, 68), width=1)

# 高亮被穿过的 cell
palette = [(60, 90, 200), (80, 130, 230), (120, 170, 245)]
for k, (cx, cy) in enumerate(cells):
    t = k / max(1, len(cells)-1)
    r = int(60 + 100*t); g = int(90 + 120*t); b = int(200 + 40*t)
    px = origin_px[0] + cx*cell
    py = origin_px[1] + cy*cell
    d.rectangle([px, py, px+cell-1, py+cell-1], fill=(r, g, b))

# 射线
sx = origin_px[0] + origin[0]*cell
sy = origin_px[1] + origin[1]*cell
ex = sx + dirn[0]*tmax*cell
ey = sy + dirn[1]*tmax*cell
d.line([(sx, sy), (ex, ey)], fill=(255, 210, 80), width=4)
d.ellipse([sx-7, sy-7, sx+7, sy+7], fill=(255, 90, 90), outline=(255,255,255), width=2)

# 图例
lx = margin + grid_size + 60
d.text((lx, origin_px[1]), "射线起点", font=font_small, fill=(255,255,255))
d.ellipse([lx-2, origin_px[1]+32, lx+14, origin_px[1]+48], fill=(255,90,90))
d.text((lx+24, origin_px[1]+32), "", font=font_small)

d.text((lx, origin_px[1]+80), "步进访问的体素", font=font_small, fill=(255,255,255))
d.rectangle([lx-2, origin_px[1]+110, lx+14, origin_px[1]+126], fill=(80,130,230))
d.text((lx, origin_px[1]+160), f"访问 {len(cells)} 个体素", font=font_small, fill=(200,210,225))

d.text((lx, origin_px[1]+210), "tMax / tDelta 步进", font=font_small, fill=(200,210,225))
d.text((lx, origin_px[1]+240), "逐轴推进, 不逐个扫描", font=font_small, fill=(150,165,190))

img.save("/root/.openclaw/workspace/daily-coding-practice/11-24-dda-voxel-grid-traversal/dda_cover.png")
print("cover saved, cells visited:", len(cells))
