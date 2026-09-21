from PIL import Image, ImageDraw, ImageFont

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

try:
    f_title = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 52)
    f_sub = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 25)
    f_small = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 20)
    f_mono = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSansMono.ttf", 20)
except Exception:
    f_title = f_sub = f_small = f_mono = ImageFont.load_default()

# 画一个跳表：四层链表，节点值为 2,5,9,12,17,25,31,42,50
# 高层只包含部分节点作为"快速索引"
nodes = [
    (2,  4),
    (5,  1),
    (9,  3),
    (12, 2),
    (17, 4),
    (25, 1),
    (31, 2),
    (42, 3),
    (50, 1),
]

# 布局：x 均匀分布，y 按 level 从顶部(level 3)到底部(level 0)
x0, x1 = 110, 1090
n = len(nodes)
xs = [x0 + (x1 - x0) * i / (n - 1) for i in range(n)]
# 4 层 (level 0..3)，level 0 在最底部
level_y = {0: 560, 1: 470, 2: 380, 3: 290}
r = 24

edge = (80, 110, 140)
node_fill = (60, 120, 190)
hi_fill = (190, 130, 60)

# 画每一层的边（横向箭头，连接该层出现的相邻节点）
for lv in range(0, 4):
    in_level = [i for i in range(n) if nodes[i][1] >= lv + 1]
    for a, b in zip(in_level, in_level[1:]):
        y = level_y[lv]
        d.line([xs[a] + r, y, xs[b] - r, y], fill=edge, width=3)
        # 箭头
        d.polygon([(xs[b]-r, y), (xs[b]-r-10, y-5), (xs[b]-r-10, y+5)], fill=edge)

# 画节点（每个节点在其最高层到 level 0 之间画竖线，节点本身画圆）
for i, (val, lv) in enumerate(nodes):
    x = xs[i]
    top_y = level_y[lv - 1]
    bot_y = level_y[0]
    if lv > 1:
        d.line([x, top_y, x, bot_y], fill=(55, 70, 95), width=2)
    # 节点圆画在最高层
    cy = level_y[lv - 1]
    fill = hi_fill if lv >= 3 else node_fill
    d.ellipse([x-r, cy-r, x+r, cy+r], fill=fill, outline=(15, 16, 20), width=2)
    tw = d.textlength(str(val), font=f_mono)
    d.text((x - tw/2, cy - 12), str(val), font=f_mono, fill=(240, 243, 250))

d.text((60, 48), "Skip List", font=f_title, fill=(235, 238, 246))
d.text((60, 120), "Probabilistic Ordered Map  ·  Geometric Random Level  ·  Expected O(log n)", font=f_sub, fill=(120, 180, 220))
d.text((60, 160), "多层排序链表 = 概率性平衡树  |  高层链表是低层的\"快速索引\"", font=f_small, fill=(150, 170, 190))

d.text((60, 610), "lookup: 自顶向下逐层下降  →  加速比 80.3x vs 线性扫描", font=f_small, fill=(140, 160, 180))
d.text((60, 640), "daily-coding-practice · 2026-09-22", font=f_sub, fill=(90, 110, 130))

img.save("cover.png")
print("cover.png written", img.size)
