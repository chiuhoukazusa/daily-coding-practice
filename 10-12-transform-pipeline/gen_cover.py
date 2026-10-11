from PIL import Image, ImageDraw, ImageFont
import math

W, H = 1200, 675
img = Image.new('RGB', (W, H), (16, 20, 30))
d = ImageDraw.Draw(img)

def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except Exception:
        return ImageFont.load_default()

f_title = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 50)
f_sub   = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 24)
f_small = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 18)
f_mono  = font("/usr/share/fonts/dejavu/DejaVuSansMono.ttf", 20)
f_cell  = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 22)

# ---- Title ----
d.text((40, 24), "3D Transform Pipeline Renderer", font=f_title, fill=(235, 238, 246))
d.text((40, 90), "MVP 变换  ·  齐次坐标  ·  LookAt 相机  ·  透视投影", font=f_sub, fill=(120, 180, 220))
d.text((40, 128), "MVP = P · V · M     NDC z ∈ [-1, 1]     w-divide", font=f_mono, fill=(150, 170, 190))

# ---- Pipeline diagram: Model -> View -> Projection -> NDC ----
stages = [
    ("Model",      "M(scale·rot·trans)", (90, 165, 255)),
    ("View",       "LookAt(eye,target,up)", (130, 220, 170)),
    ("Projection", "perspective(fov,znear,zfar)", (255, 170, 90)),
    ("NDC",        "w-divide, clip [-1,1]", (255, 120, 160)),
]
sx = 60
box_w, box_h = 220, 90
gap = 60
for i, (name, desc, col) in enumerate(stages):
    x0 = sx + i * (box_w + gap)
    y0 = 190
    d.rounded_rectangle([x0, y0, x0+box_w, y0+box_h], radius=12, fill=(28, 36, 52), outline=col, width=3)
    d.text((x0+18, y0+14), name, font=f_cell, fill=col)
    d.text((x0+18, y0+48), desc, font=f_small, fill=(210, 220, 235))
    if i < 3:
        ax0 = x0 + box_w
        ax1 = x0 + box_w + gap
        ay = y0 + box_h // 2
        d.line([ax0, ay, ax1, ay], fill=(120, 150, 180), width=3)
        d.polygon([(ax1, ay), (ax1-10, ay-6), (ax1-10, ay+6)], fill=(120, 150, 180))

# ---- Wireframe cube (right side overview) ----
def project(p, scale=1.0, cx=920, cy=330):
    # simple ortho/isometric projection for prettiness
    x, y, z = p
    sx = cx + scale * (x - y) * 0.866
    sy = cy + scale * (x + y) * 0.5 - scale * z
    return (sx, sy)

verts = []
for i in range(8):
    verts.append(((-1 if not (i&1) else 1),
                  (-1 if not (i&2) else 1),
                  (-1 if not (i&4) else 1)))
edges = [(0,1),(0,2),(0,4),(1,3),(1,5),(2,3),(2,6),(3,7),(4,5),(4,6),(5,7),(6,7)]
pts = [project(v, scale=90, cx=940, cy=290) for v in verts]
for e in edges:
    d.line([pts[e[0]], pts[e[1]]], fill=(110, 200, 255), width=3)
    # small dots at vertices
for p in pts:
    d.ellipse([p[0]-4, p[1]-4, p[0]+4, p[1]+4], fill=(255, 220, 130))

# depth annotation to the right
d.text((1040, 250), "NDC", font=f_cell, fill=(255, 160, 180))
d.text((1040, 280), "x,y ∈ [-1,1]", font=f_small, fill=(210, 220, 235))
d.text((1040, 304), "z ∈ [-1,1]", font=f_small, fill=(210, 220, 235))

# ---- Verification summary strip at bottom ----
d.text((40, 480), "Quantitative verification (no eyeballing):", font=f_sub, fill=(220, 230, 245))

checks = [
    ("12/12 pipeline checks", "all PASS"),
    ("Translation/Scale", "exact (ε<1e-6)"),
    ("Rotation", "isometry, right-hand rule"),
    ("LookAt", "target → view z=-distance"),
    ("Perspective", "znear→-1, zfar→+1"),
    ("Foreshortening", "farther projects smaller"),
]
cy = 520
for i, (k, v) in enumerate(checks):
    x = 40 + (i % 3) * 380
    y = cy + (i // 3) * 62
    d.rounded_rectangle([x, y, x+350, y+50], radius=8, fill=(26, 34, 50), outline=(50, 90, 70), width=2)
    d.text((x+14, y+8), k, font=f_cell, fill=(160, 220, 255))
    d.text((x+14, y+32), v, font=f_small, fill=(160, 240, 170))

img.save("cover.png")
print("cover.png generated")
