from PIL import Image, ImageDraw, ImageFont
import math

W, H = 1200, 675
img = Image.new('RGB', (W, H), (16, 20, 30))
d = ImageDraw.Draw(img)

try:
    f_title = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf", 52)
    f_sub = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 25)
    f_small = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSans.ttf", 20)
    f_mono = ImageFont.truetype("/usr/share/fonts/dejavu/DejaVuSansMono.ttf", 19)
except Exception:
    f_title = f_sub = f_small = f_mono = ImageFont.load_default()

# --- Radar tracking scene: radar at origin, target flies across ---
# Coordinate transform to canvas space
def world_to_canvas(x, y):
    # world: target around (800..-100, 600..750). Canvas plot area: x in [90,1100], y in [80,520]
    px = 90 + (x - -200.0) / (900.0 - -200.0) * (1100 - 90)
    py = 520 - (y - 400.0) / (800.0 - 400.0) * (520 - 80)
    return px, py

# Draw grid lines
plot_x0, plot_x1 = 90, 1100
plot_y0, plot_y1 = 80, 520
for gx in range(-200, 1001, 200):
    px, _ = world_to_canvas(gx, 0)
    d.line([px, plot_y0, px, plot_y1], fill=(30, 38, 52), width=1)
for gy in range(400, 801, 100):
    _, py = world_to_canvas(0, gy)
    d.line([plot_x0, py, plot_x1, py], fill=(30, 38, 52), width=1)

# Radar origin (bottom-left area)
rx, ry = world_to_canvas(0, 0)  # will be off-canvas below; clamp visually
d.ellipse([plot_x0 - 6, plot_y1 - 6, plot_x0 + 6, plot_y1 + 6], fill=(220, 80, 60))
d.text((plot_x0 + 12, plot_y1 - 14), "RADAR", font=f_small, fill=(220, 120, 100))

# Ground-truth trajectory (constant velocity): x from 800 -> -100, y from 600 -> 750
N = 40
truth_pts = []
for i in range(N + 1):
    t = i / N
    x = 800.0 + (-3.0) * t * 3000 * 0.05   # tvx = -3.0 over 3000*0.05s = 150s => -450
    y = 600.0 + (1.5) * t * 3000 * 0.05    # tvy = 1.5 => +225
    truth_pts.append((x, y))

# Draw truth trajectory
for a, b in zip(truth_pts, truth_pts[1:]):
    ax, ay = world_to_canvas(*a)
    bx, by = world_to_canvas(*b)
    d.line([ax, ay, bx, by], fill=(80, 180, 120), width=3)

# Draw noisy range/bearing measurements (sample every 8th point)
import random
random.seed(2026)
sig_r, sig_b = 2.0, 0.008
for i in range(0, N + 1, 8):
    x, y = truth_pts[i]
    r = math.hypot(x, y)
    b = math.atan2(y, x)
    rn = r + random.gauss(0, sig_r)
    bn = b + random.gauss(0, sig_b)
    mx = 0 + rn * math.cos(bn)
    my = 0 + rn * math.sin(bn)
    mx = max(-200, min(900, mx))
    my = max(400, min(800, my))
    px, py = world_to_canvas(mx, my)
    d.ellipse([px - 3, py - 3, px + 3, py + 3], fill=(240, 180, 60))

# EKF estimate marker (a filtered point near the truth)
ex, ey = world_to_canvas(truth_pts[-1][0], truth_pts[-1][1])
d.ellipse([ex - 7, ey - 7, ex + 7, ey + 7], outline=(90, 200, 250), width=3)

# Legend
leg_y = 545
d.ellipse([100, leg_y, 112, leg_y + 12], outline=(80, 180, 120), width=2)
d.line([106, leg_y + 6, 118, leg_y + 6], fill=(80, 180, 120), width=3)
d.text((130, leg_y - 4), "ground truth", font=f_small, fill=(80, 180, 120))

d.ellipse([330, leg_y, 342, leg_y + 12], fill=(240, 180, 60))
d.text((356, leg_y - 4), "range+bearing measurement (nonlinear)", font=f_small, fill=(240, 180, 60))

d.ellipse([760, leg_y - 2, 774, leg_y + 14], outline=(90, 200, 250), width=3)
d.text((788, leg_y - 4), "EKF estimate", font=f_small, fill=(90, 200, 250))

# Title block
d.text((60, 48), "Extended Kalman Filter", font=f_title, fill=(235, 238, 246))
d.text((60, 120), "Nonlinear State Estimation  ·  Radar Tracking  ·  Jacobian Re-linearization", font=f_sub, fill=(120, 180, 220))
d.text((60, 160), "线性动力学 + 非线性量测(range/bearing)  |  每步重新线性化 H 矩阵", font=f_small, fill=(150, 170, 190))

d.text((60, 610), "EKF / naive sensor 8.72x  ·  EKF / frozen-H KF 1.04x  ·  3-sigma 100.00%", font=f_small, fill=(140, 160, 180))
d.text((60, 640), "daily-coding-practice · 2026-09-24", font=f_sub, fill=(90, 110, 130))

img.save("cover.png")
print("cover.png written", img.size)
