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

d.text((60, 50), "Fenwick Tree", font=f_title, fill=(230, 235, 245))
d.text((60, 128), "Binary Indexed Tree  ·  O(log n) 点更新 / 前缀和 / 逆序对", font=f_sub, fill=(120, 180, 220))

# 绘制 Fenwick Tree 结构图（1-indexed，8 个元素）
# 树状数组节点：index -> 覆盖区间 [index - lowbit(index) + 1, index]
n = 8
nodes = {
    1: [1, 1], 2: [1, 2], 3: [3, 3], 4: [1, 4],
    5: [5, 5], 6: [5, 6], 7: [7, 7], 8: [1, 8],
}
# 布局：按 index 顺序水平排列，但按覆盖区间画连接
# 行：第 1 层 index 1-8，用树形连接表示 parent = i + lowbit(i)
x = {i: 90 + (i - 1) * 130 for i in range(1, n + 1)}
y = {i: 300 + (bin(i).count('1') - 1) * 70 for i in range(1, n + 1)}

node_color = (90, 130, 180)
edge_color = (80, 100, 130)

# 画边
for i in range(1, n + 1):
    p = i + (i & (-i))  # parent
    if p <= n:
        d.line([x[i], y[i], x[p], y[p]], fill=edge_color, width=2)

r = 24
for i in range(1, n + 1):
    cx, cy = x[i], y[i]
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=node_color, outline=(20, 20, 20), width=2)
    d.text((cx - 8, cy - 12), str(i), font=f_small, fill=(20, 20, 20))

d.text((60, H - 110), "bit[i] 覆盖区间 [i - lowbit(i) + 1, i]", font=f_small, fill=(140, 160, 180))
d.text((60, H - 80), "lowbit(i) = i & (-i)", font=f_small, fill=(140, 160, 180))
d.text((60, H - 50), "daily-coding-practice · 2026-09-11", font=f_sub, fill=(90, 110, 130))

img.save("cover.png")
print("cover.png written", img.size)
