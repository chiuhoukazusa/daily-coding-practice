from PIL import Image, ImageDraw, ImageFont

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

try:
    f_title = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 56)
    f_sub = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 26)
    f_small = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 20)
    f_mono = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSansMono.ttf", 22)
except Exception:
    f_title = f_sub = f_small = f_mono = ImageFont.load_default()

d.text((60, 50), "Treap", font=f_title, fill=(230, 235, 245))
d.text((60, 128), "Balanced BST  ·  Random Priority  ·  O(log n) Expected", font=f_sub, fill=(120, 180, 220))

# 绘制 Treap：每个节点 (key, priority)，BST 按 key，堆按 priority
# 构建一个小 treap 便于可视化（用确定性优先级）
import random
random.seed(7)

keys = [30, 15, 50, 5, 25, 40, 70, 45]
# 显式指定 priority 让结构好看、且满足 max-heap
prio = {30: 90, 15: 60, 50: 95, 5: 30, 25: 45, 40: 55, 70: 65, 45: 35}

node_color = (90, 130, 180)
prio_color = (200, 150, 90)
edge_color = (80, 100, 130)

# 手工树结构：根 50(95)，左子树 30(90)，右子树 70(65)
#    50
#   /  \
#  30(90) 70(65)
#  / \       \
# 5(30) 25(45)  40(55)/45(35) ...  简化布局
positions = {
    50: (600, 180),
    30: (360, 320),
    70: (840, 320),
    5:  (200, 470),
    25: (540, 470),
    40: (840, 470),
}
edges = [(50,30),(50,70),(30,5),(30,25),(70,40)]

for (a,b) in edges:
    x1,y1 = positions[a]; x2,y2 = positions[b]
    d.line([x1,y1,x2,y2], fill=edge_color, width=3)

r = 34
for k,(cx,cy) in positions.items():
    d.ellipse([cx-r,cy-r,cx+r,cy+r], fill=node_color, outline=(20,20,20), width=2)
    kl = str(k)
    pl = "p"+str(prio[k])
    d.text((cx-10, cy-26), kl, font=f_mono, fill=(235,240,248))
    d.text((cx-20, cy+4), pl, font=f_small, fill=(20,20,20))

d.text((60, 560), "key 排序保持 BST      ·      priority 保持堆(大顶)      ·      随机优先级 => 期望高度 O(log n)", font=f_small, fill=(140,160,180))
d.text((60, 600), "insert / find / erase via rotations  →  均摊 O(log n)", font=f_small, fill=(140,160,180))
d.text((60, 630), "daily-coding-practice · 2026-09-13", font=f_sub, fill=(90,110,130))

img.save("cover.png")
print("cover.png written", img.size)
