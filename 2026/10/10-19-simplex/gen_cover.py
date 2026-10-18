#!/usr/bin/env python3
"""Generate cover image for Simplex LP solver: 2D feasible region + optimal vertex.

Visualizes the textbook case:
  maximize 3x + 2y
  subject to: x + y <= 4, 2x + y <= 5, x >= 0, y >= 0

Feasible region is the polygon with vertices (0,0), (0,4), (1,3), (2.5,0).
Optimal vertex is (1,3) with objective value 9.
"""
from PIL import Image, ImageDraw, ImageFont
import os

W, H = 1200, 900
img = Image.new("RGB", (W, H), (250, 250, 250))
d = ImageDraw.Draw(img)

# Coordinate mapping: plot x in [0,5], y in [0,5]
X0, Y0 = 180, 700          # origin in pixels
X1, Y1 = 980, 120          # max corner in pixels
SX = (X1 - X0) / 5.0       # pixels per unit x
SY = (Y0 - Y1) / 5.0       # pixels per unit y

def px(x, y):
    return (X0 + x * SX, Y0 - y * SY)

def try_font(sizes):
    for s in sizes:
        for path in [
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
        ]:
            try:
                return ImageFont.truetype(path, s)
            except Exception:
                continue
    return ImageFont.load_default()

title_font = try_font([40])
label_font = try_font([26])
small_font = try_font([20])

# Feasible region polygon (shaded)
poly = [px(0,0), px(0,4), px(1,3), px(2.5,0)]
d.polygon(poly, fill=(120, 180, 255), outline=(30, 60, 140))

# Constraint lines
# x + y = 4
d.line([px(0,4), px(4,0)], fill=(200, 60, 60), width=4)
# 2x + y = 5
d.line([px(0,5), px(2.5,0)], fill=(200, 120, 0), width=4)

# Axes
d.line([px(0,0), px(5,0)], fill=(40,40,40), width=3)
d.line([px(0,0), px(0,5)], fill=(40,40,40), width=3)
# Arrowheads
d.polygon([px(5,0), (px(5,0)[0]-14, px(5,0)[1]-7), (px(5,0)[0]-14, px(5,0)[1]+7)], fill=(40,40,40))
d.polygon([px(0,5), (px(0,5)[0]-7, px(0,5)[1]+14), (px(0,5)[0]+7, px(0,5)[1]+14)], fill=(40,40,40))

# Axis ticks and labels
for i in range(6):
    xp, yp = px(i, 0)
    d.line([(xp, yp), (xp, yp+6)], fill=(40,40,40), width=2)
    d.text((xp-6, yp+10), str(i), fill=(40,40,40), font=small_font)
    xp, yp = px(0, i)
    d.line([(xp, yp), (xp-6, yp)], fill=(40,40,40), width=2)
    d.text((xp-28, yp-10), str(i), fill=(40,40,40), font=small_font)

# Constraint labels
d.text((px(1.7,2.3)[0], px(1.7,2.3)[1]), "x + y ≤ 4", fill=(200,60,60), font=label_font)
d.text((px(2.0,2.9)[0], px(2.0,2.9)[1]), "2x + y ≤ 5", fill=(180,100,0), font=label_font)

# Objective direction (gradient of 3x+2y -> direction (3,2), arrow from (1,3))
opt = px(1,3)
dx, dy = 3*SX, -2*SY
d.line([opt, (opt[0]+dx*0.6, opt[1]+dy*0.6)], fill=(0,140,0), width=4)
d.polygon([(opt[0]+dx*0.6, opt[1]+dy*0.6),
           (opt[0]+dx*0.6-12, opt[1]+dy*0.6+8),
           (opt[0]+dx*0.6-12, opt[1]+dy*0.6-8)], fill=(0,140,0))

# Optimal vertex marker
d.ellipse([opt[0]-12, opt[1]-12, opt[0]+12, opt[1]+12], fill=(0,140,0), outline=(0,100,0), width=3)
d.text((opt[0]+18, opt[1]-38), "optimal (1,3)", fill=(0,120,0), font=label_font)

# Title
d.text((180, 40), "Simplex LP Solver", fill=(20,20,20), font=title_font)
d.text((180, 95), "maximize 3x+2y  s.t.  x+y≤4, 2x+y≤5, x,y≥0  →  value = 9", fill=(60,60,60), font=label_font)
d.text((180, 130), "shaded = feasible region   |   green = optimal vertex", fill=(90,90,90), font=small_font)

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cover.png")
img.save(out, "PNG")
print("cover written:", out, os.path.getsize(out), "bytes")
