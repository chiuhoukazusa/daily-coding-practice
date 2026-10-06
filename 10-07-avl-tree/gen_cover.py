from PIL import Image, ImageDraw, ImageFont

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
d.text((40, 26), "AVL Tree", font=f_title, fill=(235, 238, 246))
d.text((40, 92), "自平衡二叉搜索树  ·  平衡因子不变式  ·  LL/RR/LR/RL 旋转", font=f_sub, fill=(120, 180, 220))

# ---- Four rotation types ----
d.text((40, 150), "四种旋转：失衡时通过局部旋转恢复平衡", font=f_sub, fill=(150, 170, 190))

def draw_tree(cx, cy, label, color):
    # simple 3-node binary tree: parent + two children
    d.ellipse([cx-22, cy-22, cx+22, cy+22], fill=color, outline=(235, 238, 246))
    tw = d.textlength(label, font=f_cell)
    d.text((cx - tw/2, cy - 12), label, font=f_cell, fill=(18, 22, 32))
    # edges and children
    d.line([cx, cy+22, cx-70, cy+70], fill=(150, 170, 190), width=2)
    d.line([cx, cy+22, cx+70, cy+70], fill=(150, 170, 190), width=2)
    d.ellipse([cx-70-22, cy+70-22, cx-70+22, cy+70+22], outline=(150, 170, 190), fill=(26, 31, 44))
    d.ellipse([cx+70-22, cy+70-22, cx+70+22, cy+70+22], outline=(150, 170, 190), fill=(26, 31, 44))

# LL: right rotation
draw_tree(120, 240, "A", (255, 160, 90))
d.text((50, 360), "LL — 右旋", font=f_small, fill=(220, 150, 90))

# RR: left rotation
draw_tree(430, 240, "A", (90, 170, 255))
d.text((360, 360), "RR — 左旋", font=f_small, fill=(90, 150, 220))

# LR: left-right
draw_tree(740, 240, "A", (160, 220, 120))
d.text((650, 360), "LR — 左旋+右旋", font=f_small, fill=(120, 200, 120))

# RL: right-left
draw_tree(1050, 240, "A", (220, 140, 180))
d.text((950, 360), "RL — 右旋+左旋", font=f_small, fill=(200, 130, 170))

# ---- Balance factor formula + height bound ----
d.text((40, 440), "平衡因子  bf = h(left) − h(right) ∈ {−1, 0, +1}", font=f_sub, fill=(220, 230, 245))
d.text((40, 500), "高度上界  h ≤ 1.44·log₂(n+2) − 0.328  →  所有操作 O(log n)", font=f_sub, fill=(130, 220, 170))

# ---- Verification summary ----
d.rounded_rectangle([40, 560, 1160, 645], radius=8, fill=(26, 34, 50), outline=(50, 90, 70), width=2)
d.text((60, 578), "量化验证：与 std::set 20 万次混合操作中序完全一致  ·  高度 20 ≪ 理论界 23.59  ·  排序输入不退化", font=f_small, fill=(160, 240, 170))

img.save("cover.png")
print("cover.png generated")
