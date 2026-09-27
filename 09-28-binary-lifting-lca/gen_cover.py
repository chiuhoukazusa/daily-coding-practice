from PIL import Image, ImageDraw, ImageFont
import math

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

try:
    f_title = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 50)
    f_sub = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 24)
    f_small = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 19)
    f_node = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 22)
except Exception:
    f_title = f_sub = f_small = f_node = ImageFont.load_default()

# ---- Draw a small rooted tree to illustrate LCA / k-th ancestor ----
# nodes: 0 root, depth levels; we draw ~11 nodes
nodes = {
    0: (600, 70),
    1: (360, 170), 2: (840, 170),
    3: (220, 280), 4: (500, 280), 5: (700, 280), 6: (960, 280),
    7: (130, 400), 8: (320, 400), 9: (600, 400), 10: (780, 400),
}

def draw_edge(a, b, color=(90, 110, 140), width=2):
    (x0, y0), (x1, y1) = nodes[a], nodes[b]
    d.line([x0, y0, x1, y1], fill=color, width=width)

edges = [(0,1),(0,2),(1,3),(1,4),(2,5),(2,6),(3,7),(3,8),(5,9),(5,10)]
for a,b in edges:
    draw_edge(a, b)

# highlight a query path: LCA(7, 8) = 3
for a,b in [(7,3),(3,1),(8,3),(1,0)]:
    draw_edge(a,b,color=(250,170,60),width=4)

def draw_node(v, fill=(34,42,58), outline=(90,160,230), text_color=(235,238,246)):
    x, y = nodes[v]
    d.ellipse([x-24, y-24, x+24, y+24], fill=fill, outline=outline, width=3)
    tw = d.textlength(str(v), font=f_node)
    d.text((x - tw/2, y - 13), str(v), font=f_node, fill=text_color)

for v in nodes:
    draw_node(v)

# highlight the LCA node (3) and endpoints (7, 8)
for v in (7, 8):
    draw_node(v, fill=(90, 60, 130), outline=(250, 170, 60))
draw_node(3, fill=(70, 110, 170), outline=(250, 170, 60), text_color=(255, 220, 150))

# annotate LCA
d.text((600 - 20, 258), "LCA(7,8)=3", font=f_small, fill=(250, 200, 120))

# ---- Title block ----
d.text((40, 20), "Binary Lifting", font=f_title, fill=(235, 238, 246))
d.text((40, 78), "LCA  ·  K-th Ancestor  |  O(log n) per query  |  倍增跳跃", font=f_sub, fill=(120, 180, 220))

# legend box
lx, ly = 880, 40
d.rounded_rectangle([lx, ly, lx + 285, ly + 150], radius=10,
                    fill=(26, 32, 46), outline=(70, 85, 110), width=2)
d.text((lx + 14, ly + 12), "up[v][j] = 2^j ancestor", font=f_sub, fill=(120, 180, 220))
d.text((lx + 14, ly + 44), "depth-align -> binary lift", font=f_small, fill=(210, 220, 235))
d.text((lx + 14, ly + 72), "j from LOG-1 downto 0", font=f_small, fill=(210, 220, 235))
d.text((lx + 14, ly + 100), "k-th ancestor: bitmask jump", font=f_small, fill=(250, 170, 60))
d.text((lx + 14, ly + 128), "preprocess O(n log n)", font=f_small, fill=(150, 170, 190))

# ---- bottom info ----
d.text((40, 540), "correctness 150000/150000  ·  depth-align + binary lift  ·  naive O(depth) baseline", font=f_small, fill=(150, 170, 190))
d.text((40, 570), "n=200000 deep chain  ·  Q=100000  ·  speedup 470.3x  ·  results identical", font=f_small, fill=(150, 170, 190))
d.text((40, 600), "LOG = floor(log2 n)+1  ·  up[v][0]=parent  ·  up[v][j]=up[up[v][j-1]][j-1]", font=f_small, fill=(140, 160, 180))
d.text((40, 642), "daily-coding-practice . 2026-09-28", font=f_sub, fill=(90, 110, 130))

img.save("cover.png")
print("cover.png written", img.size)
