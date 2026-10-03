from PIL import Image, ImageDraw, ImageFont
import math

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except Exception:
        return ImageFont.load_default()

f_title = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 52)
f_sub   = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 26)
f_small = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 19)
f_mono  = font("/usr/share/fonts/dejavu/DejaVuSansMono.ttf", 20)
f_cell  = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 22)
f_dnode = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 20)

# ---- Title block ----
d.text((40, 24), "Zero-One BFS", font=f_title, fill=(235, 238, 246))
d.text((40, 88), "Deque-based Shortest Path  |  O(V+E)  ·  0/1 edge weights", font=f_sub, fill=(120, 180, 220))

# ---- Left panel: a small 0/1 weighted graph (picture) ----
# nodes laid out in a ring-ish shape
nodes = {
    "s": (110, 420),
    "a": (250, 300),
    "b": (250, 540),
    "c": (400, 240),
    "d": (400, 420),
    "e": (400, 600),
    "t": (560, 420),
}
node_col = (235, 238, 246)
src_col  = (90, 200, 120)
tgt_col  = (250, 170, 60)

# edges: (u, v, w)
edges = [
    ("s", "a", 0),
    ("s", "b", 1),
    ("a", "b", 0),
    ("a", "c", 1),
    ("b", "d", 0),
    ("c", "d", 0),
    ("c", "t", 0),
    ("d", "t", 1),
    ("d", "e", 0),
    ("e", "t", 1),
]
w0_col = (90, 200, 120)   # weight 0
w1_col = (220, 140, 90)   # weight 1

for u, v, w in edges:
    x1, y1 = nodes[u]
    x2, y2 = nodes[v]
    col = w0_col if w == 0 else w1_col
    d.line([x1, y1, x2, y2], fill=col, width=2)
    mx, my = (x1 + x2) / 2, (y1 + y2) / 2
    d.text((mx - 10, my - 12), str(w), font=f_small, fill=col)

for name, (x, y) in nodes.items():
    r = 20
    fill = src_col if name == "s" else (tgt_col if name == "t" else (30, 38, 52))
    outline = src_col if name == "s" else (tgt_col if name == "t" else (110, 130, 150))
    d.ellipse([x - r, y - r, x + r, y + r], fill=fill, outline=outline, width=2)
    tw = d.textlength(name, font=f_dnode)
    d.text((x - tw / 2, y - 12), name, font=f_dnode, fill=(235, 238, 246))

# legend for edges
d.rectangle([80, 600, 100, 620], fill=w0_col)
d.text((110, 598), "edge weight 0  (push_front)", font=f_small, fill=(120, 200, 140))
d.rectangle([80, 628, 100, 648], fill=w1_col)
d.text((110, 626), "edge weight 1  (push_back)", font=f_small, fill=(230, 150, 100))

# ---- Right panel: deque animation (3 states) ----
d.text((640, 150), "Deque  (distance 单调不减)", font=f_sub, fill=(150, 170, 190))

# deque state helper: draw a deque as a horizontal set of boxes
def draw_deque(x0, y0, items, cw=54, ch=44, label=""):
    n = len(items)
    # draw head/tail labels
    d.text((x0 - 30, y0 + 10), "head", font=f_small, fill=(110, 170, 200))
    for i, (lab, dist) in enumerate(items):
        x = x0 + i * cw
        d.rectangle([x, y0, x + cw, y0 + ch], outline=(40, 50, 66), fill=(26, 31, 44))
        tw = d.textlength(lab, font=f_cell)
        d.text((x + cw/2 - tw/2, y0 + 4), lab, font=f_cell, fill=(235, 238, 246))
        tw2 = d.textlength(str(dist), font=f_small)
        d.text((x + cw/2 - tw2/2, y0 + 26), str(dist), font=f_small, fill=(120, 180, 220))
    if label:
        d.text((x0 + n * cw + 16, y0 + 8), label, font=f_small, fill=(140, 160, 180))

# state 1: start
d.text((640, 196), "1)  start from s (dist 0)", font=f_small, fill=(140, 160, 180))
draw_deque(700, 220, [("s", 0)])

# state 2: after relaxing edges from s
d.text((640, 278), "2)  relax s->a (w=0), s->b (w=1)", font=f_small, fill=(140, 160, 180))
draw_deque(700, 302, [("a", 0), ("b", 1)])
d.text((970, 310), "w=0 到队头", font=f_small, fill=(120, 200, 140))

# state 3: after relaxing from a
d.text((640, 358), "3)  relax a->b (w=0) 更新, a->c (w=1)", font=f_small, fill=(140, 160, 180))
draw_deque(700, 382, [("b", 0), ("c", 1)])
d.text((970, 390), "w=0 更新到队头", font=f_small, fill=(120, 200, 140))

# ---- bottom bar ----
d.text((640, 470), "关键不变量", font=f_sub, fill=(250, 170, 60))
d.text((640, 510), "队列中节点距离最多相差 1，", font=f_mono, fill=(200, 210, 225))
d.text((640, 540), "故无需优先队列，每节点至多入队一次。", font=f_mono, fill=(200, 210, 225))

# ---- results strip ----
d.text((640, 590), "0-1 BFS 0.057s  vs  Dijkstra 0.085s  ·  1.50x faster", font=f_mono, fill=(120, 200, 140))
d.text((640, 620), "n=200k  m=1M edges  ·  mismatches=0", font=f_mono, fill=(140, 160, 180))

img.save("cover.png")
print("cover.png generated")
