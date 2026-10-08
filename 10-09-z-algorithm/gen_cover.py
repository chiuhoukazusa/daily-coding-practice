from PIL import Image, ImageDraw, ImageFont

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except Exception:
        return ImageFont.load_default()

f_title = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 58)
f_sub   = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 25)
f_small = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 20)
f_mono  = font("/usr/share/fonts/dejavu/DejaVuSansMono.ttf", 20)
f_cell  = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 22)

# ---- Title block ----
d.text((40, 30), "Z-Algorithm", font=f_title, fill=(235, 238, 246))
d.text((40, 104), "Z-Function  ·  Linear-Time Pattern Matching  ·  Z-box Reuse", font=f_sub, fill=(120, 180, 220))
d.text((40, 150), "Z[i] = LCP(S[0..], S[i..])   (Z[0] = 0)", font=f_sub, fill=(150, 170, 190))

# ---- Z-value visualization for S = "abacabadabacaba" ----
d.text((40, 200), "Example:  S = abacabadabacaba", font=f_sub, fill=(220, 230, 245))
S = "abacabadabacaba"
zvals = [0, 0, 1, 0, 3, 0, 1, 0, 7, 0, 1, 0, 3, 0, 1]
cell = 72
x0 = 40
y0 = 232
for i, ch in enumerate(S):
    x = x0 + i * cell
    d.text((x + 28, y0), ch, font=f_mono, fill=(200, 215, 235))
for i, z in enumerate(zvals):
    x = x0 + i * cell
    col = (255, 160, 90) if z == max(zvals) else (120, 180, 220)
    d.text((x + 28, y0 + 48), str(z), font=f_mono, fill=col)
d.text((40, y0 + 92), "Z[8] = 7  ->  S[0..6] == S[8..]   (longest Z-box)", font=f_small, fill=(255, 160, 90))

# ---- Pattern matching construction ----
d.text((40, 396), "Linear matching:  build  P + '#' + T ,  then compute Z", font=f_sub, fill=(220, 230, 245))
d.text((40, 448), "Z[k] >= |P|   <=>   P occurs in T at position k-(|P|+1)", font=f_sub, fill=(130, 220, 170))

# ---- Speedup bar chart ----
d.text((40, 500), "Worst case (all 'a') speedup:", font=f_sub, fill=(220, 230, 245))
bars = [("36.0x", 0.13), ("102.4x", 0.37), ("273.9x", 1.0)]
base_y = 530
max_w = 680
for i, (label, ratio) in enumerate(bars):
    bx = 40 + i * 270
    bw = 40 + int(max_w * ratio)
    d.rectangle([bx, base_y, bx + bw, base_y + 44], fill=(90, 170, 255), outline=(140, 200, 255))
    d.text((bx + bw + 10, base_y + 8), label, font=f_cell, fill=(160, 220, 255))

# ---- Verification summary ----
d.rounded_rectangle([40, 600, 1160, 650], radius=8, fill=(26, 34, 50), outline=(50, 90, 70), width=2)
d.text((60, 615), "Verified: Z-function 207/207  ·  matching 300/300  ·  worst-case 273.9x speedup  ·  counts match", font=f_small, fill=(160, 240, 170))

img.save("cover.png")
print("cover.png generated")
