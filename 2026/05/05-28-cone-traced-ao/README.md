# Cone-Traced Ambient Occlusion Renderer

Day 90 of Daily Coding Practice.

A CPU-based soft rasterizer implementing cone-traced ambient occlusion with bent normals, using SDF ray marching for fast geometry queries.

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wno-missing-field-initializers
./output
```

## 输出结果
| AO Raw | Bent Normals | Final Lit |
|--------|-------------|-----------|
| ![AO](ao_raw_output.png) | ![Bent](ao_bent_output.png) | ![Final](ao_final_output.png) |

## 技术要点
- SDF场景（球体+盒子+平面）支持快速AO采样
- 余弦加权半球采样（Cosine-weighted importance sampling）
- Bent Normal计算：累积未被遮蔽方向的加权平均
- 距离衰减遮蔽（二次falloff）
- 3张输出：原始AO/弯曲法线可视化/完整光照
