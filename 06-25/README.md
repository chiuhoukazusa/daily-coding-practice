# Frustum Culling Renderer - 视锥剔除渲染器

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果
- `nocull.png` — 无剔除模式（1000球体全部渲染）
- `cull.png` — 视锥剔除模式（自动剔除视野外球体）

## 技术要点
- Gribb/Hartmann 6平面视锥体提取
- AABB-Plane P-vertex 分离轴剔除测试
- 列主序矩阵（VP = View * Proj）
- 立方体→球面投影 + 三角形光栅化 + Z-Buffer + Lambert光照
- 64.9% 剔除率（FOV=45°），47.4% 剔除率（FOV=60°）
