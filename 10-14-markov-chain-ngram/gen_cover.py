from PIL import Image, ImageDraw, ImageFont
import math, random

W, H = 1200, 675
img = Image.new('RGB', (W, H), (16, 20, 30))
d = ImageDraw.Draw(img)

def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except Exception:
        return ImageFont.load_default()

f_title = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 48)
f_sub   = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 24)
f_small = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 18)
f_mono  = font("/usr/share/fonts/dejavu/DejaVuSansMono.ttf", 20)
f_cell  = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 24)

# ---- Title ----
d.text((40, 24), "Markov Chain N-Gram Text Generation", font=f_title, fill=(235, 238, 246))
d.text((40, 88), "N-Gram 马尔可夫链  ·  转移概率矩阵  ·  加权随机采样  ·  字符级生成", font=f_sub, fill=(150, 200, 255))
d.text((40, 124), "P(next | context)   Σ P = 1.0   perplexity = exp(cross-entropy)", font=f_mono, fill=(150, 170, 190))

# ---- State transition graph ----
# States as nodes; draw a directed graph representing the trigram chain.
random.seed(42)
nodes = {}  # name -> (x, y)
cx, cy = 260, 560
names = ["th", "he", "e ", " t", "qu", "ui", "br", "ow", "fo", "ov"]
# place nodes in an arc
n = len(names)
for i, nm in enumerate(names):
    ang = math.pi * (1.0 + 0.75 * (i / (n - 1))) if n > 1 else math.pi
    # spread around center
    x = 180 + (i % 5) * 220
    y = 200 + (i // 5) * 170
    nodes[nm] = (x + 60, y + 60)

# Draw edges (transition arrows) with probabilities
rng = random.Random(7)
edges = []
for a in nodes:
    for b in nodes:
        if rng.random() < 0.22:
            edges.append((a, b, round(rng.uniform(0.05, 0.85), 2)))

for (a, b, p) in edges:
    x1, y1 = nodes[a]
    x2, y2 = nodes[b]
    d.line([x1, y1, x2, y2], fill=(70, 90, 120), width=max(1, int(p * 6)))
    # arrow head
    ang = math.atan2(y2 - y1, x2 - x1)
    ax, ay = x2 - 12 * math.cos(ang), y2 - 12 * math.sin(ang)
    d.polygon([(x2, y2), (ax - 8*math.sin(ang), ay + 8*math.cos(ang)),
               (ax + 8*math.sin(ang), ay - 8*math.cos(ang))], fill=(150, 200, 255))

# Draw nodes
for nm, (x, y) in nodes.items():
    r = 34
    d.ellipse([x - r, y - r, x + r, y + r], fill=(30, 42, 62), outline=(90, 165, 255), width=3)
    # center text
    tw = d.textlength(nm, font=f_cell)
    d.text((x - tw/2, y - 14), nm, font=f_cell, fill=(235, 240, 250))

# ---- Probability distribution bar (right side) ----
d.text((980, 200), "P(·|\"th\")", font=f_cell, fill=(150, 200, 255))
dist = [('a', 0.10), (' ', 0.10), ('e', 0.80)]
by = 240
for ch, p in dist:
    d.text((980, by), f"'{ch}'", font=f_mono, fill=(210, 220, 235))
    bar_x0 = 1040
    bar_w = int(p * 120)
    d.rectangle([bar_x0, by + 4, bar_x0 + bar_w, by + 20], fill=(90, 165, 255))
    d.text((bar_x0 + bar_w + 8, by), f"{p:.2f}", font=f_small, fill=(200, 210, 225))
    by += 34

# ---- Footer metrics ----
d.text((40, 620), "quantified: normalization ΣP=1 · unseen-context handling · seed reproducibility · perplexity", font=f_small, fill=(120, 140, 160))

img.save("cover.png")
print("cover.png generated")
