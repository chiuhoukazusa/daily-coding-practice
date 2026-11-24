#!/usr/bin/env python3
# 生成 Anisotropic GGX Microfacet BRDF 封面图
# 将两张渲染结果（各向异性/各向同性）并排合成一张封面，
# 加标题与标注，突出"被拉伸的高光"这一核心效果。
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 720
BG = (13, 17, 28)

def load_scaled(path, size=(360, 360)):
    img = Image.open(path).convert("RGB")
    return img.resize(size, Image.LANCZOS)

# 加载两张渲染结果
ani = load_scaled("/root/.openclaw/workspace/daily-coding-practice/2026-11-25-anisotropic-ggx/aniso_anisotropic.ppm")
iso = load_scaled("/root/.openclaw/workspace/daily-coding-practice/2026-11-25-anisotropic-ggx/aniso_isotropic.ppm")

canvas = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(canvas)

try:
    font_title = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 44)
    font_sub   = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
    font_label = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 26)
    font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
except Exception:
    font_title = font_sub = font_label = font_small = ImageFont.load_default()

# 标题
d.text((80, 40), "Anisotropic GGX Microfacet BRDF", font=font_title, fill=(240, 244, 255))
d.text((80, 100), "Microfacet Model — 各向异性微表面 · 被拉伸的高光", font=font_sub, fill=(150, 165, 190))

# 两张图并排
gap = 140
left_x = 80
right_x = left_x + 360 + gap
img_y = 180

# 左侧：各向异性
canvas.paste(ani, (left_x, img_y))
d.rectangle([left_x-2, img_y-2, left_x+360+2, img_y+360+2], outline=(90, 130, 230), width=3)
d.text((left_x, img_y+378), "Anisotropic  ax=0.30  ay=0.05", font=font_label, fill=(120, 170, 245))
d.text((left_x, img_y+412), "高光被拉伸 · 椭圆度 ≈ 6.35x", font=font_small, fill=(200, 210, 225))

# 右侧：各向同性
canvas.paste(iso, (right_x, img_y))
d.rectangle([right_x-2, img_y-2, right_x+360+2, img_y+360+2], outline=(120, 120, 135), width=3)
d.text((right_x, img_y+378), "Isotropic  a=0.15", font=font_label, fill=(210, 210, 220))
d.text((right_x, img_y+412), "高光近圆 · 椭圆度 ≈ 1.04x", font=font_small, fill=(200, 210, 225))

# 底部要点
d.text((80, H-60), "Cook-Torrance  ·  GGX NDF (T/B 双粗糙度轴)  ·  Schlick Fresnel  ·  Smith 几何遮蔽", font=font_small, fill=(150, 165, 190))

canvas.save("/root/.openclaw/workspace/daily-coding-practice/2026-11-25-anisotropic-ggx/anisotropic_ggx_cover.png")
print("cover saved")
