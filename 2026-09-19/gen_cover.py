from PIL import Image, ImageDraw, ImageFont

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

try:
    f_title = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 58)
    f_sub = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 26)
    f_small = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 20)
    f_mono = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSansMono.ttf", 22)
except Exception:
    f_title = f_sub = f_small = f_mono = ImageFont.load_default()

d.text((60, 46), "Bloom Filter", font=f_title, fill=(230, 235, 245))
d.text((60, 128), "Probabilistic Set  ·  Bit Array  ·  Multiple Hashing", font=f_sub, fill=(120, 180, 220))

# 绘制布隆过滤器示意图：
# 顶部标签：m 位数组
# 元素 x/y 通过 k 个哈希函数映射到 k 个位，置 1

CELL = 16
GAP = 2
COLS = 60
grid_x, grid_y = 80, 260

for c in range(COLS):
    x0 = grid_x + c * (CELL + GAP)
    y0 = grid_y
    d.rectangle([x0, y0, x0 + CELL - 1, y0 + CELL - 1], outline=(60, 72, 88), width=1)

def mkhash(x, seed):
    x = (x + seed * 0x9E3779B97F4A7C15) & 0xFFFFFFFFFFFFFFFF
    x = ((x ^ (x >> 30)) * 0xBF58476D1CE4E5B9) & 0xFFFFFFFFFFFFFFFF
    x = ((x ^ (x >> 27)) * 0x94D049BB133111EB) & 0xFFFFFFFFFFFFFFFF
    return (x ^ (x >> 31)) & 0xFFFFFFFFFFFFFFFF

def positions(x, k, m=COLS):
    h1 = mkhash(x, 0x12345678)
    h2 = mkhash(x, 0x87654321) | 1
    return [(h1 + i * h2) % m for i in range(k)]

xfills = positions(0xB100, 3)
yfills = positions(0xF11E, 3)

def set_cell(col, color):
    x0 = grid_x + col * (CELL + GAP)
    y0 = grid_y
    d.rectangle([x0, y0, x0 + CELL - 1, y0 + CELL - 1], fill=color)

for col in xfills:
    set_cell(col, (90, 170, 150))
for col in yfills:
    set_cell(col, (210, 150, 90))

shared = set(xfills) & set(yfills)
for col in shared:
    set_cell(col, (240, 210, 120))

# 标注箭头
for i, col in enumerate(xfills):
    tx = grid_x + col * (CELL + GAP) + CELL // 2
    lx = 160 + i * 40
    d.line([lx, 200, tx, grid_y - 8], fill=(90, 170, 150), width=2)
    d.ellipse([tx - 3, grid_y - 11, tx + 3, grid_y - 5], fill=(90, 170, 150))

d.text((80, 150), "item x", font=f_mono, fill=(90, 170, 150))
d.text((220, 150), "item y", font=f_mono, fill=(210, 150, 90))

# 图例 + 说明（全部英文，避免 CJK 字体缺失导致乱码）
d.text((80, 470), "bit set by item x", font=f_small, fill=(90, 170, 150))
d.text((80, 505), "bit set by item y", font=f_small, fill=(210, 150, 90))
d.text((80, 540), "shared bit (collision)", font=f_small, fill=(240, 210, 120))

gr_x = 560
d.text((gr_x, 470), "insert(x): set k bits to 1", font=f_small, fill=(160, 175, 195))
d.text((gr_x, 505), "query(x): all k bits = 1 -> hit", font=f_small, fill=(160, 175, 195))
d.text((gr_x, 540), "false positive: k bits all 1 by chance", font=f_small, fill=(160, 175, 195))

d.text((60, 615), "m-bit array  ·  k hashing  ·  zero false-negative", font=f_small, fill=(90, 100, 115))

img.save("cover.png")
print("cover.png generated")
