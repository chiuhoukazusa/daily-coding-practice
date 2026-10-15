from PIL import Image, ImageDraw, ImageFont

W, H = 1200, 675
img = Image.new('RGB', (W, H), (18, 22, 32))
d = ImageDraw.Draw(img)

try:
    f_title = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 52)
    f_sub = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 25)
    f_small = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 20)
    f_mono = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSansMono.ttf", 21)
except Exception:
    f_title = f_sub = f_small = f_mono = ImageFont.load_default()

d.text((56, 48), "Particle System Physics", font=f_title, fill=(230, 235, 245))
d.text((56, 124), "Euler vs Verlet Integration", font=f_title, fill=(110, 170, 220))
d.text((56, 192), "Projectile Motion  ·  Energy Conservation  ·  Bounce Lifecycle", font=f_sub, fill=(120, 160, 200))

# 绘制弹道轨迹：左侧 Euler(发散) 右侧 Verlet(闭合)
# 抛物线投影轨迹
import math
euler_col = (220, 120, 90)
verlet_col = (90, 180, 120)
axis_col = (70, 90, 110)

# 坐标系原点（画面左半中央偏下）
ox, oy = 150, 540
scale = 3.2

# Euler 轨迹：能量发散，摆幅越来越大 (用放大的正弦包络表现能量漂移)
pts_euler = []
pts_verlet = []
for i in range(120):
    t = i / 119.0
    x = i * 4.0
    # Euler: 振幅随 t 指数增长，表现能量漂移发散
    amp_e = 30.0 * (1.0 + 6.0 * t)
    y_e = oy - amp_e * math.sin(t * 6.0 * math.pi)
    # Verlet: 振幅恒定，表现能量守恒（有界漂移）
    amp_v = 30.0
    y_v = oy - amp_v * math.sin(t * 6.0 * math.pi)
    pts_euler.append((ox + x, y_e))
    pts_verlet.append((ox + x, y_v))

# 坐标轴
d.line([ox, oy - 200, ox, oy], fill=axis_col, width=2)
d.line([ox, oy, ox + 500, oy], fill=axis_col, width=2)

# Euler 包络线（发散）
d.line([(ox, oy - 30), (ox + 476, oy - 210)], fill=(120, 60, 40), width=2)
# Verlet 包络线（恒定）
d.line([(ox, oy - 30), (ox + 476, oy - 30)], fill=(60, 120, 80), width=2)

# 轨迹线
d.line(pts_euler, fill=euler_col, width=3)
d.line(pts_verlet, fill=verlet_col, width=3)

# 图例
d.rectangle([620, 480, 660, 500], fill=euler_col)
d.text((672, 478), "Euler: energy drift (diverges)", font=f_small, fill=(220, 140, 110))
d.rectangle([620, 520, 660, 540], fill=verlet_col)
d.text((672, 518), "Verlet: symplectic (bounded)", font=f_small, fill=(120, 190, 130))

# 右侧说明
d.text((620, 560), "a  =  -g  j   (gravity)", font=f_mono, fill=(120, 180, 220))
d.text((620, 596), "E = 1/2 m v^2 + 1/2 k x^2", font=f_mono, fill=(140, 160, 180))

# 底部
d.text((56, 600), "Euler: O(dt)  explicit  ·  Verlet: symplectic  O(dt^2)", font=f_small, fill=(140, 160, 180))
d.text((56, 636), "game physics  ·  ballistic trajectory  ·  accurate long-term energy", font=f_small, fill=(140, 160, 180))

img.save("cover.png")
print("cover.png generated")
