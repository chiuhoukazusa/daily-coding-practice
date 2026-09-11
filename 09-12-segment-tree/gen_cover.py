from PIL import Image, ImageDraw, ImageFont

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

try:
    f_title = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 56)
    f_sub = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 26)
    f_small = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 20)
    f_mono = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSansMono.ttf", 24)
except Exception:
    f_title = f_sub = f_small = f_mono = ImageFont.load_default()

d.text((60, 50), "Segment Tree", font=f_title, fill=(230, 235, 245))
d.text((60, 128), "Range Sum Query  ·  Lazy Propagation  ·  O(log n)", font=f_sub, fill=(120, 180, 220))

# 绘制线段树结构图（8 个叶子元素，满二叉树）
n = 8
# 节点布局：完全二叉树，根节点在顶层，叶子在最底层
# 用数组下标 1-indexed 表示：node 1 是根，覆盖 [0, n-1]
node_color = (90, 130, 180)
leaf_color = (120, 170, 120)
edge_color = (80, 100, 130)

# 递归画边和节点
from math import log2

levels = int(log2(n)) + 1  # 4 层（根 + 3 层内部 + 叶子）
total_nodes = 2 * n - 1

# 每个节点覆盖区间 [l, r]，按完全二叉树位置布局
def layout(node, l, r, depth):
    mid = (l + r) >> 1
    # 水平位置按 l,r 均匀分布
    cx = 70 + (l + r) / (2 * n) * (W - 140)
    cy = 180 + depth * 90
    return cx, cy

# 用 BFS 收集边
edges = []
nodes_pos = {}
def build(node, l, r, depth):
    cx, cy = layout(node, l, r, depth)
    nodes_pos[node] = (cx, cy, l == r)
    if l == r:
        return
    mid = (l + r) >> 1
    lcx, lcy = layout(node * 2, l, mid, depth + 1)
    rcx, rcy = layout(node * 2 + 1, mid + 1, r, depth + 1)
    edges.append((cx, cy, lcx, lcy))
    edges.append((cx, cy, rcx, rcy))
    build(node * 2, l, mid, depth + 1)
    build(node * 2 + 1, mid + 1, r, depth + 1)

build(1, 0, n - 1, 0)

# 画边
for (x1, y1, x2, y2) in edges:
    d.line([x1, y1, x2, y2], fill=edge_color, width=2)

# 画节点
r = 22
for node, (cx, cy, is_leaf) in nodes_pos.items():
    color = leaf_color if is_leaf else node_color
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=color, outline=(20, 20, 20), width=2)
    label = str(node)
    tw = d.textlength(label, font=f_small) if hasattr(d, 'textlength') else len(label) * 11
    d.text((cx - tw / 2, cy - 11), label, font=f_small, fill=(20, 20, 20))

d.text((60, H - 140), "tree[node] 存区间和  ·  lazy[node] 存懒标记", font=f_small, fill=(140, 160, 180))
d.text((60, H - 110), "range_add / range_query / kth  →  O(log n)", font=f_small, fill=(140, 160, 180))
d.text((60, H - 80), "push_down 懒标记下传保证正确性", font=f_small, fill=(140, 160, 180))
d.text((60, H - 50), "daily-coding-practice · 2026-09-12", font=f_sub, fill=(90, 110, 130))

img.save("cover.png")
print("cover.png written", img.size)
