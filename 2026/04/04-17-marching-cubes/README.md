# Marching Cubes Isosurface Extraction

从 3D 标量场（SDF）提取等值面，生成三角形网格并软光栅化渲染。

## 技术要点

- **Marching Cubes 算法**：Paul Bourke 1994 经典实现，edgeTable + triTable 查找表驱动
- **SDF 场景**：多球体 + 圆环 + 胶囊体，smooth-min 有机融合
- **等值面法线**：由 SDF 梯度（数值微分）计算顶点法线，支持插值
- **软光栅化渲染**：透视投影 + 重心坐标插值 + Phong 着色
- **后处理**：gamma 矫正（γ=2.2）
- **无依赖 PNG 导出**：手写 zlib store 压缩 + PNG chunk 编写

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果

![结果](marching_cubes_output.png)

生成 25,740 个三角形，800×600 分辨率，渲染耗时 ~280ms。
