from PIL import Image, ImageDraw, ImageFont
import math

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

try:
    f_title = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 48)
    f_sub = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 24)
    f_small = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 19)
    f_node = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 26)
except Exception:
    f_title = f_sub = f_small = f_node = ImageFont.load_default()

# ---- Draw a sample B-Tree (minimum degree t=3) ----
# A B-tree of min degree 3 stores 2..5 keys per node (t-1 .. 2t-1)

def draw_node(cx, cy, keys, w=170, h=64, fill=(34, 42, 58), outline=(90, 160, 230),
              highlight=False):
    x0, x1 = cx - w // 2, cx + w // 2
    y0, y1 = cy - h // 2, cy + h // 2
    if highlight:
        fill = (70, 110, 170)
        outline = (250, 170, 60)
    d.rounded_rectangle([x0, y0, x1, y1], radius=10, fill=fill, outline=outline, width=3)
    key_str = ",".join(str(k) for k in keys)
    tw = d.textlength(key_str, font=f_node)
    d.text((cx - tw / 2, cy - 13), key_str, font=f_node, fill=(235, 238, 246))

levels = [
    {"nodes": [(400, 100, [40], True)]},
    {"nodes": [(140, 250, [10, 20], False), (660, 250, [55, 70], False)]},
    {"nodes": [
        (55, 410, [3, 5], False), (225, 410, [15], False), (400, 410, [25, 30], False),
        (540, 410, [45, 50], False), (740, 410, [60, 65], False), (915, 410, [75, 80, 90], False),
    ]},
]

def draw_edges():
    d.line([380, 132, 150, 218], fill=(120, 140, 165), width=2)
    d.line([420, 132, 650, 218], fill=(120, 140, 165), width=2)
    d.line([110, 282, 70, 378], fill=(90, 110, 130), width=2)
    d.line([150, 282, 225, 378], fill=(90, 110, 130), width=2)
    d.line([175, 282, 390, 378], fill=(90, 110, 130), width=2)
    d.line([630, 282, 545, 378], fill=(90, 110, 130), width=2)
    d.line([660, 282, 740, 378], fill=(90, 110, 130), width=2)
    d.line([695, 282, 905, 378], fill=(90, 110, 130), width=2)

draw_edges()
for lvl in levels:
    for (cx, cy, keys, hl) in lvl["nodes"]:
        draw_node(cx, cy, keys, highlight=hl)

# ---- Title block ----
d.text((40, 20), "B-Tree", font=f_title, fill=(235, 238, 246))
d.text((40, 74), "Multiway Search Tree  |  disk-friendly  |  O(log_t n)", font=f_sub, fill=(120, 180, 220))

# legend box (right side)
lx, ly = 900, 60
d.rounded_rectangle([lx, ly, lx + 270, ly + 130], radius=10,
                    fill=(26, 32, 46), outline=(70, 85, 110), width=2)
d.text((lx + 14, ly + 12), "min degree t = 3", font=f_sub, fill=(120, 180, 220))
d.text((lx + 14, ly + 42), "keys in [t-1, 2t-1] = [2,5]", font=f_small, fill=(210, 220, 235))
d.text((lx + 14, ly + 70), "child ptr = keys + 1", font=f_small, fill=(210, 220, 235))
d.text((lx + 14, ly + 98), "all leaves same depth", font=f_small, fill=(250, 170, 60))

# ---- bottom info ----
d.text((40, 540), "splitChild  .  merge  .  borrow  .  predecessor/successor delete", font=f_small, fill=(150, 170, 190))
d.text((40, 570), "search/insert/delete O(log_t n)  .  disk-friendly  .  std::set baseline", font=f_small, fill=(150, 170, 190))
d.text((40, 610), "inorder sorted PASS . height-balanced PASS . value-map/delete PASS", font=f_small, fill=(140, 160, 180))
d.text((40, 642), "daily-coding-practice . 2026-09-27", font=f_sub, fill=(90, 110, 130))

img.save("cover.png")
print("cover.png written", img.size)
