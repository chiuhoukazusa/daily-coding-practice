from PIL import Image, ImageDraw, ImageFont

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

try:
    f_title = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 50)
    f_sub = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 25)
    f_small = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 19)
    f_cell = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 16)
except Exception:
    f_title = f_sub = f_small = f_cell = ImageFont.load_default()

# ---- Draw a sparse table st[k][i] grid (k levels x columns) ----
# levels: k=0..4, columns i=0..11
levels = 5
cols = 12
x0, y0 = 70, 140
cw, ch = 86, 52

# sample array values for n=12 (deterministic)
a = [5, 2, 8, 1, 9, 3, 6, 4, 7, 0, 2, 5]

# precompute min table values
st = []
st.append(a)
for k in range(1, levels):
    cur = []
    length = 1 << k
    for i in range(cols):
        if i + length <= cols:
            cur.append(min(st[k-1][i], st[k-1][i + (1 << (k-1))]))
        else:
            cur.append(None)
    st.append(cur)

for k in range(levels):
    y = y0 + k * ch
    # level label
    d.text((x0 - 60, y + 6), f"k={k}", font=f_small, fill=(120, 180, 220))
    d.text((x0 - 60, y + 24), f"len={1<<k}", font=f_small, fill=(90, 110, 130))
    for i in range(cols):
        x = x0 + i * cw
        v = st[k][i]
        if v is None:
            d.rectangle([x, y, x + cw - 2, y + ch - 2], outline=(40, 50, 66), fill=(22, 27, 38))
        else:
            d.rectangle([x, y, x + cw - 2, y + ch - 2], outline=(70, 85, 110), fill=(28, 34, 48))
            tv = str(v)
            tw = d.textlength(tv, font=f_cell)
            d.text((x + (cw-2)/2 - tw/2, y + (ch-2)/2 - 12), tv, font=f_cell, fill=(235, 238, 246))

# highlight example query: RMQ(3, 9) -> k=floor(log2 7)=2 -> min(st[2][3], st[2][6])
# visual: outline the two covering blocks on k=2 row
ky = y0 + 2 * ch
# st[2][3] covers indices 3..6, st[2][6] covers 6..9
for si in (3, 6):
    x = x0 + si * cw
    d.rectangle([x, ky, x + cw - 2, ky + ch - 2], outline=(250, 170, 60), width=3)

# query range annotation under the grid
d.line([x0 + 3*cw, ky + ch + 8, x0 + 9*cw + cw, ky + ch + 8], fill=(250, 170, 60), width=3)
d.text((x0 + 3*cw, ky + ch + 16), "query range [3, 9]", font=f_small, fill=(250, 200, 120))
d.text((x0 + 3*cw, ky + ch + 40), "k = floor(log2 7) = 2  ->  min(st[2][3], st[2][6])", font=f_small, fill=(210, 220, 235))

# ---- Title block ----
d.text((40, 20), "Sparse Table", font=f_title, fill=(235, 238, 246))
d.text((40, 78), "RMQ  ·  Range Minimum Query  |  O(1) per query  |  稀疏表", font=f_sub, fill=(120, 180, 220))

# legend box
lx, ly = 880, 40
d.rounded_rectangle([lx, ly, lx + 285, ly + 150], radius=10,
                    fill=(26, 32, 46), outline=(70, 85, 110), width=2)
d.text((lx + 14, ly + 12), "st[k][i] = min over 2^k", font=f_sub, fill=(120, 180, 220))
d.text((lx + 14, ly + 44), "st[0][i] = a[i]", font=f_small, fill=(210, 220, 235))
d.text((lx + 14, ly + 72), "st[k][i] = min(prev[i], prev[i+2^(k-1)])", font=f_small, fill=(210, 220, 235))
d.text((lx + 14, ly + 100), "query: k = log2[len]", font=f_small, fill=(250, 170, 60))
d.text((lx + 14, ly + 128), "min is idempotent -> overlap ok", font=f_small, fill=(150, 170, 190))

# ---- bottom info ----
d.text((40, 600), "correctness 200000/200000  ·  static array  ·  naive O(n) baseline", font=f_small, fill=(150, 170, 190))
d.text((40, 630), "n=50000  ·  Q=200000  ·  speedup 455.8x  ·  results identical", font=f_small, fill=(150, 170, 190))
d.text((40, 646), "daily-coding-practice . 2026-09-29", font=f_sub, fill=(90, 110, 130))

img.save("cover.png")
print("cover.png written", img.size)
