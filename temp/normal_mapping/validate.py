#!/usr/bin/env python3
"""验证法线贴图输出 - 检查两个球体的渲染差异"""
import sys
from PIL import Image
import numpy as np

def validate_sphere(img_path, center_x, center_y, size, name):
    img = Image.open(img_path)
    pixels = np.array(img)
    
    # 提取中心区域
    half_size = size // 2
    region = pixels[
        center_y-half_size:center_y+half_size,
        center_x-half_size:center_x+half_size
    ]
    
    # 统计
    mean_color = region.mean(axis=(0,1))
    min_color = region.min(axis=(0,1))
    max_color = region.max(axis=(0,1))
    std_color = region.std(axis=(0,1))
    
    print(f"\n{name} 球体 ({center_x},{center_y}) 大小 {size}x{size}:")
    print(f"  平均颜色: RGB({mean_color[0]:.1f}, {mean_color[1]:.1f}, {mean_color[2]:.1f})")
    print(f"  最小值: RGB({min_color[0]}, {min_color[1]}, {min_color[2]})")
    print(f"  最大值: RGB({max_color[0]}, {max_color[1]}, {max_color[2]})")
    print(f"  标准差: RGB({std_color[0]:.1f}, {std_color[1]:.1f}, {std_color[2]:.1f})")
    
    # 检查是否是黑色
    is_black = (mean_color < 10).all()
    if is_black:
        print(f"  ❌ {name}球体是纯黑色！")
        return False
    else:
        print(f"  ✅ {name}球体有颜色")
        return True

if __name__ == "__main__":
    img_path = sys.argv[1] if len(sys.argv) > 1 else "normal_mapping_output.png"
    
    # 图像尺寸 800x600
    # 左球中心约 (800/4, 600/2) = (200, 300)
    # 右球中心约 (800*3/4, 600/2) = (600, 300)
    
    left_ok = validate_sphere(img_path, 200, 300, 100, "左侧（平滑）")
    right_ok = validate_sphere(img_path, 600, 300, 100, "右侧（法线贴图）")
    
    print("\n" + "="*60)
    if left_ok and right_ok:
        print("✅ 验证通过：两个球体都正常渲染")
    else:
        print("❌ 验证失败：存在黑色球体")
        sys.exit(1)
