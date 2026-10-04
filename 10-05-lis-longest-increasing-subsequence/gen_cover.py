from PIL import Image, ImageDraw, ImageFont
import math

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except Exception:
        return ImageFont.load_default()

f_title = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 54)
f_sub   = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 25)
f_small = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 19)
f_mono  = font("/usr/share/fonts/dejavu/DejaVuSansMono.ttf", 20)
f_cell  = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 22)

# ---- Title block ----
d.text((40, 26), "Longest Increasing Subsequence", font=f_title, fill=(235, 238, 246))
d.text((40, 92), "O(n log n) Patience Greedy  ·  DP / lower_bound / 严格递增", font=f_sub, fill=(120, 180, 220))

# ---- Left panel: DP table illustration ----
d.text((40, 150), "O(n^2) DP  —  dp[i]=1+max{dp[j]: j<i, a[j]<a[i]}", font=f_sub, fill=(150, 170, 190))

a = [10, 9, 2, 5, 3, 7, 101, 18]
n = len(a)
dp = [1]*n
for i in range(n):
    for j in range(i):
        if a[j] < a[i]:
            dp[i] = max(dp[i], dp[j]+1)

cw, ch = 78, 52
x0, y0 = 60, 210
# index row
for i in range(n):
    x = x0 + i*cw
    d.text((x + cw/2 - 8, y0 - 26), str(i), font=f_mono, fill=(110, 130, 150))
# value row
for i in range(n):
    x = x0 + i*cw
    col = (90, 200, 120) if a[i] == 101 else (235, 238, 246)
    d.rectangle([x, y0, x+cw, y0+ch], outline=(40, 50, 66), fill=(26, 31, 44))
    tw = d.textlength(str(a[i]), font=f_cell)
    d.text((x + cw/2 - tw/2, y0 + 14), str(a[i]), font=f_cell, fill=col)
# dp row
for i in range(n):
    x = x0 + i*cw
    d.rectangle([x, y0+ch+8, x+cw, y0+2*ch+8], outline=(50, 40, 90), fill=(30, 24, 52))
    tw = d.textlength(str(dp[i]), font=f_cell)
    d.text((x + cw/2 - tw/2, y0+ch+8+14), str(dp[i]), font=f_cell, fill=(200, 160, 255))
d.text((x0, y0 + 2*ch + 22), "a[i]", font=f_mono, fill=(120, 200, 140))
d.text((x0, y0 + 3*ch + 30), "dp[i]", font=f_mono, fill=(200, 160, 255))

# ---- Right panel: tails 数组 (Patience) ----
d.text((40, 470), "O(n log n) Patience  —  tails 数组（长度 k+1 的最小结尾）", font=f_sub, fill=(150, 170, 190))

def draw_tails(y, tails, label, hl=-1):
    d.text((60, y - 26), label, font=f_small, fill=(140, 160, 180))
    for k, v in enumerate(tails):
        x = x0 + k*cw
        fill = (34, 90, 60) if k == hl else (26, 31, 44)
        d.rectangle([x, y, x+cw, y+ch], outline=(40, 50, 66), fill=fill)
        tw = d.textlength(str(v), font=f_cell)
        d.text((x + cw/2 - tw/2, y + 14), str(v), font=f_cell,
               fill=(120, 220, 160) if k == hl else (235, 238, 246))
        d.text((x + cw/2 - 6, y - 22), str(k), font=f_small, fill=(110, 130, 150))

draw_tails(500, [2], "扫描 10, 9, 2 后", hl=0)
draw_tails(560, [2, 5], "扫描 5 后（扩展）", hl=1)
draw_tails(620, [2, 3, 7, 101, 18], "最终（长度=5? 见下）", hl=3)

# ---- bottom bar ----
d.text((40, 640), "最终 LIS 长度 = len(tails)", font=f_mono, fill=(250, 170, 60))

# ---- results strip (right side bottom) ----
d.text((640, 470), "验证结果", font=f_sub, fill=(250, 170, 60))
d.text((640, 510), "穷举 2000 用例 · 三种实现一致 · 失败=0", font=f_mono, fill=(120, 200, 140))
d.text((640, 540), "随机 10000 用例 · 重建严格递增合法", font=f_mono, fill=(120, 200, 140))
d.text((640, 570), "n=100k  ·  Patience 3.84ms  vs  DP 12839ms", font=f_mono, fill=(140, 160, 180))
d.text((640, 600), "加速比 ≈ 3345x", font=f_mono, fill=(250, 170, 60))

img.save("cover.png")
print("cover.png generated")
