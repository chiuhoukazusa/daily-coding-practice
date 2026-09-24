from PIL import Image, ImageDraw, ImageFont
import math, random

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

try:
    f_title = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 52)
    f_sub = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 25)
    f_small = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 20)
except Exception:
    f_title = f_sub = f_small = ImageFont.load_default()

random.seed(2026)

# Render a bipartite assignment diagram: left tasks, right workers, edges = assignments
n = 6
lx = 190
rx = 1010
ly0, ly1 = 130, 520
ry0, ry1 = 130, 520

# Left nodes (tasks)
lcols = [(90, 160, 230)] * n
lpts = []
for i in range(n):
    y = ly0 + (ly1 - ly0) * i / (n - 1)
    lpts.append((lx, y))
# Right nodes (workers)
rpts = []
for j in range(n):
    y = ry0 + (ry1 - ry0) * j / (n - 1)
    rpts.append((rx, y))

# Draw all edges faintly (complete bipartite)
for i in range(n):
    for j in range(n):
        d.line([lpts[i][0], lpts[i][1], rpts[j][0], rpts[j][1]],
               fill=(38, 46, 62), width=1)

# Highlight a perfect matching (a random permutation with distinct columns)
perm = list(range(n))
random.shuffle(perm)
match_colors = [(250, 170, 60), (90, 200, 140), (230, 90, 120),
                (120, 180, 250), (200, 130, 230), (240, 220, 90)]
for i, j in enumerate(perm):
    c = match_colors[i % len(match_colors)]
    d.line([lpts[i][0], lpts[i][1], rpts[j][0], rpts[j][1]], fill=c, width=4)

# Draw nodes
for (x, y), c in zip(lpts, lcols):
    d.ellipse([x - 18, y - 18, x + 18, y + 18], fill=c, outline=(220, 230, 245), width=2)
for (x, y) in rpts:
    d.ellipse([x - 18, y - 18, x + 18, y + 18], fill=(220, 120, 90),
              outline=(245, 235, 225), width=2)

# Labels
d.text((lx - 72, ly0 - 52), "TASKS (rows)", font=f_small, fill=(120, 170, 220))
d.text((rx - 76, ry0 - 52), "WORKERS (cols)", font=f_small, fill=(230, 140, 110))
for i in range(n):
    d.text((lpts[i][0] + 28, lpts[i][1] - 10), f"T{i+1}", font=f_small, fill=(180, 210, 240))
    d.text((rpts[i][0] - 64, rpts[i][1] - 10), f"W{i+1}", font=f_small, fill=(250, 200, 180))

# Cost matrix panel on left-bottom corner
mx0, my0 = 60, 545
cell = 34
cm = [[random.randint(1, 99) for _ in range(4)] for _ in range(4)]
d.rectangle([mx0 - 4, my0 - 4, mx0 + 4 * cell + 4, my0 + 4 * cell + 4],
            outline=(60, 70, 90), width=2)
for i in range(4):
    for j in range(4):
        xx = mx0 + j * cell
        yy = my0 + i * cell
        d.rectangle([xx, yy, xx + cell, yy + cell], outline=(48, 58, 76), width=1)
        d.text((xx + cell // 2 - 10, yy + cell // 2 - 12), str(cm[i][j]),
               font=f_small, fill=(210, 220, 235))
d.text((mx0, my0 - 34), "cost matrix (n x n)", font=f_small, fill=(150, 170, 190))

# Title block
d.text((60, 50), "Hungarian Algorithm", font=f_title, fill=(235, 238, 246))
d.text((60, 122), "Kuhn-Munkres  ·  Assignment Problem  ·  Maximum-Weight Perfect Matching",
       font=f_sub, fill=(120, 180, 220))
d.text((60, 162), "顶标 (u+v=cost) + 相等子图 + 增广路  |  O(n³) 多项式最优", font=f_small, fill=(150, 170, 190))

d.text((60, 610), "最优性 1600/1600  ·  贪心非最优 86.5%  ·  max-weight 1200/1200  ·  O(n³) 验证",
       font=f_small, fill=(140, 160, 180))
d.text((60, 642), "daily-coding-practice · 2026-09-25", font=f_sub, fill=(90, 110, 130))

img.save("cover.png")
print("cover.png written", img.size)
