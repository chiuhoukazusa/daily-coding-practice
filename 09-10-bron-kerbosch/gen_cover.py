from PIL import Image, ImageDraw, ImageFont

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

try:
    f_title = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 56)
    f_sub = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 28)
    f_small = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 22)
except Exception:
    f_title = f_sub = f_small = ImageFont.load_default()

d.text((60, 50), "Bron-Kerbosch", font=f_title, fill=(230, 235, 245))
d.text((60, 130), "Maximal Clique Enumeration  ·  Pivot Optimization", font=f_sub, fill=(120, 180, 220))

# 可视化极大团枚举：一个示例图 + 高亮一个极大团
# 示例图节点坐标（8 节点，含一个 K4 + 一个三角形）
nodes = {
    0: (220, 300), 1: (340, 240), 2: (340, 380), 3: (460, 300),  # K4
    4: (620, 220), 5: (700, 320), 6: (560, 360),                  # triangle
    7: (720, 480),                                               # isolated
}
edges = [
    (0,1),(0,2),(0,3),(1,2),(1,3),(2,3),  # K4
    (4,5),(5,6),(4,6),                     # triangle
    (0,4),                                # bridge
]
# 高亮一个极大团 {0,1,2,3}（K4）
highlight = {0,1,2,3}
node_color_hl = (240, 180, 60)
node_color = (90, 110, 140)
edge_color_hl = (240, 200, 100)
edge_color = (70, 85, 105)

r = 26
for a, b in edges:
    c = edge_color_hl if (a in highlight and b in highlight) else edge_color
    d.line([nodes[a], nodes[b]], fill=c, width=4)
for n, (x, y) in nodes.items():
    c = node_color_hl if n in highlight else node_color
    d.ellipse([x-r, y-r, x+r, y+r], fill=c, outline=(20,20,20), width=2)
    d.text((x-8, y-14), str(n), font=f_small, fill=(20,20,20))

d.text((60, H-70), "daily-coding-practice · 2026-09-10", font=f_sub, fill=(90,110,130))

img.save("cover.png")
print("cover.png written", img.size)
