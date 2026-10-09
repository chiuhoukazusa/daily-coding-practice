from PIL import Image, ImageDraw, ImageFont

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except Exception:
        return ImageFont.load_default()

f_title = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 56)
f_sub   = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 25)
f_small = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 19)
f_mono  = font("/usr/share/fonts/dejavu/DejaVuSansMono.ttf", 20)
f_cell  = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 22)

# ---- Title block ----
d.text((40, 26), "Dijkstra Shortest Path", font=f_title, fill=(235, 238, 246))
d.text((40, 96), "二叉堆优化  ·  Greedy 最短路  ·  Lazy Deletion", font=f_sub, fill=(120, 180, 220))
d.text((40, 136), "priority_queue< (dist, v) >   O((V+E) log V)", font=f_sub, fill=(150, 170, 190))

# ---- Small example graph with distances ----
d.text((40, 190), "Example:  single-source shortest path from node 0", font=f_sub, fill=(220, 230, 245))
# nodes layout (x, y)
nodes = {
    0: (140, 300), 1: (360, 230), 2: (360, 370), 3: (600, 230), 4: (600, 370),
}
# edges (u, v, w) with drawn offsets
edges = [(0, 1, 4), (0, 2, 1), (1, 3, 2), (2, 1, 1), (2, 4, 5), (3, 4, 1), (1, 4, 3)]
for (u, v, w) in edges:
    x1, y1 = nodes[u]; x2, y2 = nodes[v]
    d.line([x1, y1, x2, y2], fill=(90, 130, 170), width=2)
    mx, my = (x1 + x2) // 2, (y1 + y2) // 2
    d.text((mx + 4, my - 10), str(w), font=f_small, fill=(255, 200, 120))

# node colors: source highlight
dist = {0: 0, 1: 2, 2: 1, 3: 4, 4: 5}  # known shortest distances from 0
for n, (x, y) in nodes.items():
    col = (255, 160, 90) if n == 0 else (36, 48, 70)
    d.ellipse([x - 22, y - 22, x + 22, y + 22], fill=col, outline=(140, 200, 255), width=2)
    d.text((x - 8, y - 12), str(n), font=f_cell, fill=(235, 238, 246))
    if n != 0:
        d.text((x - 34, y + 26), "d=%d" % dist[n], font=f_small, fill=(130, 220, 170))

d.text((720, 250), "shortest:  0 -> 2 -> 1 -> 3 -> 4 = 5", font=f_sub, fill=(130, 220, 170))
d.text((720, 300), "path weights:  1 + 1 + 2 + 1", font=f_small, fill=(150, 170, 190))

# ---- Speedup bar chart ----
d.text((40, 470), "Speedup vs O(V^2) baseline:", font=f_sub, fill=(220, 230, 245))
bars = [("sparse V=20000", 1.00, "207.55x"), ("dense V=3000", 0.09, "5.26x")]
base_y = 500
max_w = 680
for i, (label, ratio, speedup) in enumerate(bars):
    bx = 40 + i * 360
    bw = 30 + int(max_w * ratio)
    d.rectangle([bx, base_y, bx + bw, base_y + 44], fill=(90, 170, 255), outline=(140, 200, 255))
    d.text((bx, base_y + 52), label, font=f_small, fill=(160, 190, 220))
    d.text((bx + bw + 10, base_y + 8), speedup, font=f_cell, fill=(160, 220, 255))

# ---- Verification summary ----
d.rounded_rectangle([40, 600, 1160, 650], radius=8, fill=(26, 34, 50), outline=(50, 90, 70), width=2)
d.text((60, 615), "Verified: 2326/2326 distances match Floyd-Warshall & O(V^2)  ·  sparse 207.55x  ·  dense 5.26x  ·  0 mismatch", font=f_small, fill=(160, 240, 170))

img.save("cover.png")
print("cover.png generated")
