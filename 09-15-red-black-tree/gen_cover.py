from PIL import Image, ImageDraw, ImageFont

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

try:
    f_title = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 54)
    f_sub = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 26)
    f_small = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 20)
    f_mono = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSansMono.ttf", 22)
except Exception:
    f_title = f_sub = f_small = f_mono = ImageFont.load_default()

def draw_dot(cx, cy, r, fill):
    d.ellipse([cx-r, cy-r, cx+r, cy+r], fill=fill, outline=(15, 15, 18), width=2)

BLACK = (30, 34, 44)
RED = (190, 70, 70)
edge = (80, 100, 130)

d.text((60, 48), "Red-Black Tree", font=f_title, fill=(235, 238, 246))
d.text((60, 126), "Self-Balancing BST  ·  Rotation + Recolor  ·  O(log n) Worst-Case", font=f_sub, fill=(120, 180, 220))

# 手工构造一棵红黑树展示结构性（黑色节点 + 红色节点）
#           13[B]
#          /      \
#       8[R]      17[R]
#       /  \      /   \
#    1[B] 11[B] 15[B] 25[B]
#          /           \
#        nil..         27[R]

# 布局
pos = {
    13: (600, 170),
    8:  (370, 310),
    17: (830, 310),
    1:  (190, 450),
    11: (520, 450),
    15: (680, 450),
    25: (980, 450),
    27: (1080, 585),
}
colormap = {13: BLACK, 8: RED, 17: RED, 1: BLACK, 11: BLACK, 15: BLACK, 25: BLACK, 27: RED}
edges = [(13,8),(13,17),(8,1),(8,11),(17,15),(17,25),(25,27)]

for (a,b) in edges:
    x1,y1 = pos[a]; x2,y2 = pos[b]
    d.line([x1,y1,x2,y2], fill=edge, width=3)

r = 34
for k,(cx,cy) in pos.items():
    draw_dot(cx, cy, r, colormap[k])
    d.text((cx-14, cy-16), str(k), font=f_mono, fill=(240, 243, 250))

d.text((60, 560), "1) 根节点为黑   2) 红节点不接红节点   3) 所有路径黑高相等", font=f_small, fill=(140,160,180))
d.text((60, 600), "insert / erase  →  旋转(左/右) + 染色  →  O(log n) 最坏保证", font=f_small, fill=(140,160,180))
d.text((60, 635), "daily-coding-practice · 2026-09-15", font=f_sub, fill=(90,110,130))

img.save("cover.png")
print("cover.png written", img.size)
