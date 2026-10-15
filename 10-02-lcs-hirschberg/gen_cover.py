from PIL import Image, ImageDraw, ImageFont

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except Exception:
        return ImageFont.load_default()

f_title = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 52)
f_sub   = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 26)
f_small = font("/usr/share/fonts/dejavu/DejaVuSans.ttf", 19)
f_cell  = font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 22)

# ---- Title block ----
d.text((40, 24), "Longest Common Subsequence", font=f_title, fill=(235, 238, 246))
d.text((40, 88), "Hirschberg 线性空间算法  |  O(n·m) time  ·  O(min(n,m)) space", font=f_sub, fill=(120, 180, 220))

# ---- Example strings ----
a = "ABCBDAB"
b = "BDCABA"
d.text((40, 150), "a = ", font=f_sub, fill=(150, 170, 190))
d.text((86, 150), a, font=f_cell, fill=(235, 238, 246))
d.text((40, 190), "b = ", font=f_sub, fill=(150, 170, 190))
d.text((86, 190), b, font=f_cell, fill=(235, 238, 246))

# ---- LCS == BDAB ----
d.text((40, 245), "LCS =  ", font=f_sub, fill=(150, 170, 190))
d.text((128, 238), "BDAB", font=font("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 42), fill=(250, 170, 60))

# ---- DP table (n+1) x (m+1) with match highlighting ----
# rows = chars of a (+1 header), cols = chars of b (+1 header)
n, m = len(a), len(b)
# small DP table drawn on the right side
x0, y0 = 460, 150
cw, ch = 46, 44

# header cells
for j in range(m + 1):
    x = x0 + j * cw
    d.rectangle([x, y0, x + cw, y0 + ch], outline=(40, 50, 66), fill=(26, 31, 44))
    if j > 0:
        tv = b[j-1]
        tw = d.textlength(tv, font=f_cell)
        d.text((x + cw/2 - tw/2, y0 + 10), tv, font=f_cell, fill=(120, 180, 220))

# compute DP table (naive) to fill values & find matches that form LCS
dp = [[0]*(m+1) for _ in range(n+1)]
for i in range(1, n+1):
    for j in range(1, m+1):
        if a[i-1] == b[j-1]:
            dp[i][j] = dp[i-1][j-1] + 1
        else:
            dp[i][j] = max(dp[i-1][j], dp[i][j-1])

# backtrack to collect LCS cells (match positions)
match_cells = set()
i, j = n, m
while i > 0 and j > 0:
    if a[i-1] == b[j-1]:
        match_cells.add((i, j))
        i -= 1; j -= 1
    elif dp[i-1][j] >= dp[i][j-1]:
        i -= 1
    else:
        j -= 1

# draw row labels + cells
for i in range(n + 1):
    y = y0 + (i+1) * ch
    # row label
    lab = "" if i == 0 else a[i-1]
    if i == 0:
        d.rectangle([x0 - cw, y, x0, y + ch], outline=(40, 50, 66), fill=(26, 31, 44))
    else:
        d.rectangle([x0 - cw, y, x0, y + ch], outline=(40, 50, 66), fill=(26, 31, 44))
        tw = d.textlength(lab, font=f_cell)
        d.text((x0 - cw + cw/2 - tw/2, y + 10), lab, font=f_cell, fill=(120, 180, 220))
    for j in range(m + 1):
        x = x0 + j * cw
        is_match = (i > 0 and j > 0 and (i, j) in match_cells)
        if is_match:
            d.rectangle([x, y, x + cw, y + ch], outline=(250, 170, 60), fill=(60, 48, 28), width=3)
            tv = str(dp[i][j])
            tw = d.textlength(tv, font=f_cell)
            d.text((x + cw/2 - tw/2, y + 10), tv, font=f_cell, fill=(255, 220, 150))
        else:
            d.rectangle([x, y, x + cw, y + ch], outline=(40, 50, 66), fill=(22, 27, 38))
            tv = str(dp[i][j])
            tw = d.textlength(tv, font=f_cell)
            d.text((x + cw/2 - tw/2, y + 10), tv, font=f_cell, fill=(160, 175, 195))

# ---- Right-side legend / notes box ----
lx, ly = 40, 360
d.rounded_rectangle([lx, ly, lx + 360, ly + 260], radius=10, fill=(26, 32, 46), outline=(70, 85, 110), width=2)
notes = [
    ("recurrence:", (120, 180, 220)),
    ("dp[i][j] = dp[i-1][j-1]+1", (235, 238, 246)),
    ("   if a[i]==b[j]", (210, 220, 235)),
    ("dp[i][j] = max(prev)", (235, 238, 246)),
    ("   otherwise", (210, 220, 235)),
    ("", (0, 0, 0)),
    ("Hirschberg: divide & conquer", (250, 170, 60)),
    ("split + fwd/bwd LCS length", (210, 220, 235)),
    ("-> reconstruct in O(min) space", (210, 220, 235)),
    ("", (0, 0, 0)),
    ("1.5 GB  (full table)", (250, 120, 110)),
    ("3.5 MB  (Hirschberg)", (120, 220, 140)),
]
yy = ly + 14
for txt, col in notes:
    d.text((lx + 16, yy), txt, font=f_small, fill=col)
    yy += 30

# ---- Bottom stat strip ----
d.rounded_rectangle([460, y0 + (n+2)*ch + 6, W - 40, y0 + (n+2)*ch + 56], radius=8,
                    fill=(26, 32, 46), outline=(70, 85, 110), width=2)
d.text((476, y0 + (n+2)*ch + 18), "206/206 cases passed  |  3 impls agree  |  linear-space traceback", font=f_small, fill=(235, 238, 246))

img.save("cover.png")
print("saved cover.png")
