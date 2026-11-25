#!/usr/bin/env python3
# 生成 Voronoi Diagram Construction 封面图
# 将 Voronoi 渲染结果放大，加标题与标注，突出
# "Voronoi 单元分区 + Delaunay 三角剖分对偶" 这一核心效果。
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 720
BG = (13, 17, 28)

def load_scaled(path, size=(560, 560)):
    img = Image.open(path).convert("RGB")
    return img.resize(size, Image.LANCZOS)

img = load_scaled("/root/.openclaw/workspace/daily-coding-practice/11-26-voronoi-fortune/voronoi_output.png")

canvas = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(canvas)

try:
    font_title = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 46)
    font_sub   = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
    font_label = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
    font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
except Exception:
    font_title = font_sub = font_label = font_small = ImageFont.load_default()

# 标题
d.text((60, 40), "Voronoi Diagram Construction", font=font_title, fill=(240, 244, 255))
d.text((60, 100), "Fortune Sweep-Line · Delaunay Dual · 最近邻分区", font=font_sub, fill=(150, 165, 190))

# 主图（居中偏左）
img_x, img_y = 60, 140
canvas.paste(img, (img_x, img_y))
d.rectangle([img_x-2, img_y-2, img_x+560+2, img_y+560+2], outline=(90, 130, 230), width=3)

# 右侧标注区
rx = img_x + 560 + 50
d.text((rx, img_y + 20), "Voronoi 单元", font=font_label, fill=(120, 170, 245))
d.text((rx, img_y + 56), "· 40 个站点最近邻分区", font=font_small, fill=(200, 210, 225))
d.text((rx, img_y + 84), "· 单元满铺 面积和 = 1.0", font=font_small, fill=(200, 210, 225))
d.text((rx, img_y + 130), "Delaunay 对偶", font=font_label, fill=(120, 170, 245))
d.text((rx, img_y + 166), "· F = 2N-2-h = 68 三角", font=font_small, fill=(200, 210, 225))
d.text((rx, img_y + 194), "· 空圆误差 ~1e-14", font=font_small, fill=(200, 210, 225))
d.text((rx, img_y + 240), "量化验证", font=font_label, fill=(120, 170, 245))
d.text((rx, img_y + 276), "· 欧拉公式 MATCH", font=font_small, fill=(200, 210, 225))
d.text((rx, img_y + 304), "· 蒙特卡洛面积 0.0002 误差", font=font_small, fill=(200, 210, 225))

# 底部要点
d.text((60, H-48), "Fortune 扫描线  ·  半平面交裁剪单元  ·  垂直平分线共享边  ·  空圆性质校验", font=font_small, fill=(150, 165, 190))

canvas.save("/root/.openclaw/workspace/daily-coding-practice/11-26-voronoi-fortune/voronoi_cover.png")
print("cover saved")
