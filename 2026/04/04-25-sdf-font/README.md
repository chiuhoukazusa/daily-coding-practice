# SDF Font Rendering

Signed Distance Field (SDF) 字体渲染技术演示。

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果

![SDF Font Rendering](sdf_font_output.png)

## 技术要点

- **SDF生成**：从高分辨率位图字形生成有向距离场，逐像素搜索最近边界
- **平滑抗锯齿**：通过 threshold + smoothing 参数控制边缘梯度，实现次像素级抗锯齿
- **描边效果**：利用第二个 SDF 阈值独立提取轮廓层，无需额外几何
- **投影阴影**：对偏移 UV 坐标采样 SDF，用软化参数控制阴影虚化程度
- **发光效果**：基于 SDF 距离衰减叠加彩色光晕，二次曲线控制衰减形状
- **多尺寸渲染**：同一张 SDF 贴图在不同分辨率下均保持清晰边缘
- **等值线可视化**：距离场热力图 + 等距轮廓线，直观展示 SDF 原理
- **字重调节**：通过调整 threshold 值模拟细体/常规/粗体效果

## 核心算法

```cpp
// SDF 边缘采样渲染
float alpha = clamp(
    (dist - threshold) / smoothing + 0.5f,
    0.0f, 1.0f
);

// 描边：outline 层 = threshold - outlineWidth ~ threshold
float outline_alpha = smoothstep(threshold - outlineWidth, threshold, dist)
                    - smoothstep(threshold, threshold + smoothing, dist);

// 阴影：偏移 UV 采样 SDF
float shadow_dist = sampleSDF(sdf, u - offsetX, v - offsetY);
float shadow_alpha = smoothstep(0.5f - softness, 0.5f, shadow_dist);
```
