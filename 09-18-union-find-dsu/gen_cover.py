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

d.text((60, 50), "Union-Find", font=f_title, fill=(230, 235, 245))
d.text((60, 128), "Disjoint Set Union  ·  Path Compression  ·  Union by Rank", font=f_sub, fill=(120, 180, 220))

# 绘制森林：若干棵树，每棵树的根由 parent 指针指向自己
# 用确定性 parent 关系构造 3 棵小树，展示路径压缩后的扁平结构

node_color = (90, 160, 120)
root_color = (200, 150, 90)
edge_color = (80, 110, 130)

# 树结构表示 (parent -> children)
# 树1: 根 0，子 1,2，孙 3,4,5
# 树2: 根 6，子 7,8
# 树3: 根 9 孤立
positions = {
    0: (220, 200),   # root1
    1: (100, 360), 2: (220, 360), 3: (340, 360),
    4: (70, 520), 5: (170, 520),
    6: (640, 200),   # root2
    7: (560, 360), 8: (720, 360),
    9: (980, 300),   # isolated root3
}
edges = [(1,0),(2,0),(3,0),(4,1),(5,2),(7,6),(8,6)]

for (a,b) in edges:
    x1,y1 = positions[a]; x2,y2 = positions[b]
    d.line([x1,y1,x2,y2], fill=edge_color, width=3)

roots = {0, 6, 9}
r = 34
for k,(cx,cy) in positions.items():
    col = root_color if k in roots else node_color
    d.ellipse([cx-r,cy-r,cx+r,cy+r], fill=col, outline=(20,20,20), width=2)
    d.text((cx-10, cy-12), str(k), font=f_mono, fill=(235,240,248))

# 右侧说明文字
d.text((760, 420), "find(x): path compression", font=f_small, fill=(140,160,180))
d.text((760, 460), "  parent[x] = parent[parent[x]]", font=f_mono, fill=(120,180,220))
d.text((760, 500), "unite(a,b): union by rank", font=f_small, fill=(140,160,180))
d.text((760, 540), "  depth <= log2(N)", font=f_mono, fill=(120,180,220))

d.text((60, 600), "amortized O(alpha(N))  ~=  O(1)", font=f_small, fill=(140,160,180))
d.text((60, 636), "Kruskal MST  ·  connectivity  ·  dynamic connectivity", font=f_small, fill=(140,160,180))

img.save("cover.png")
print("cover.png generated")
