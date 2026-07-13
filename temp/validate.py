#!/usr/bin/env python3
from PIL import Image
import numpy as np

img = Image.open("obj_loader_output.png")
pixels = np.array(img)

# 检查图片是否全白（背景应该是白色）
white_pixels = np.sum((pixels == 255).all(axis=2))
total_pixels = pixels.shape[0] * pixels.shape[1]
white_ratio = white_pixels / total_pixels

print(f"图片尺寸: {pixels.shape[0]}x{pixels.shape[1]}")
print(f"白色像素比例: {white_ratio:.2%}")

# 检查图片中心区域是否有黑色线条（立方体边缘）
center_region = pixels[200:400, 300:500]
black_pixels = np.sum((center_region == 0).all(axis=2))
black_ratio = black_pixels / (center_region.shape[0] * center_region.shape[1])

print(f"中心区域黑色像素比例: {black_ratio:.2%}")

# 验证
if white_ratio < 0.8:
    print("❌ 白色背景比例过低")
    exit(1)

if black_ratio < 0.01:
    print("❌ 中心区域没有足够的黑色线条（立方体边缘）")
    exit(1)

print("✅ 输出验证通过：图片包含立方体线框")
