# BVH Accelerated Triangle Mesh Renderer

BVH层次包围盒加速的三角形网格光线追踪渲染器，实现了Cornell Box场景。

## 编译运行
```bash
cp ../../../stb_image_write.h .
g++ main.cpp -o bvh_renderer -std=c++17 -O2
./bvh_renderer
```

## 输出结果
![结果](bvh_output.png)

## 技术要点
- BVH层次包围盒加速结构（中值分割）
- Möller–Trumbore三角形快速相交算法
- 每顶点法线插值（平滑着色）
- 镜面反射与漫反射材质
- 程序化网格：icosphere、圆环体、立方体
- ACES Filmic色调映射
- Cornell Box场景（800×600，64 SPP）
