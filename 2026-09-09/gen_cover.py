from PIL import Image, ImageDraw, ImageFont
import random

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

# Title
try:
    f_title = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 56)
    f_sub = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 28)
except:
    f_title = f_sub = ImageFont.load_default()

d.text((60, 50), "Suffix Array & LCP", font=f_title, fill=(230, 235, 245))
d.text((60, 130), "Prefix Doubling + Kasai  ·  O(n log n) + O(n)", font=f_sub, fill=(120, 180, 220))

# Draw the SA visualization: sorted suffixes of "banana"
s = "banana"
n = len(s)
sa = [5, 3, 1, 0, 4, 2]
# colors per start position
colors = [(240,120,120),(120,200,120),(120,140,240),(240,220,120),(200,120,220),(120,220,220)]
cell = 42
x0, y0 = 80, 240
for r, start in enumerate(sa):
    y = y0 + r * (cell + 8)
    d.text((x0, y - 6), f"rank {r}:", font=f_sub, fill=(180,180,190))
    for j, ch in enumerate(s[start:]):
        x = x0 + 140 + j * cell
        c = colors[start]
        d.rounded_rectangle([x, y, x+cell-6, y+cell-6], radius=6, fill=c)
        d.text((x+14, y+8), ch, font=f_sub, fill=(20,20,20))
    d.text((x0+140+cell*len(s[start:])+20, y+8), f" (idx {start})", font=f_sub, fill=(120,120,130))

# Footer
d.text((60, H-70), "daily-coding-practice · 2026-09-09", font=f_sub, fill=(90,110,130))

img.save("cover.png")
print("cover.png written", img.size)
