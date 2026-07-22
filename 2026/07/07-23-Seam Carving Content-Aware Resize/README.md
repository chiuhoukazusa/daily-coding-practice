# Seam Carving Content-Aware Resize

图像内容感知缩放，通过动态规划的接缝剪裁（Seam Carving）算法实现。

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出文件

- `original.ppm` — 原始输入图像
- `energy_map.ppm` — 能量图（梯度幅值）
- `seam_carved.ppm` — 接缝剪裁后的缩小图像
- `naive_scaled.ppm` — 简单缩放结果（对比用）
- `seam_visualization.ppm` — 接缝可视化（高亮被移除的像素）

## 技术要点

- **能量函数**: 使用 Sobel 梯度幅值作为像素重要性度量
- **动态规划**: DP 累计能量矩阵，逐行计算最小能量路径
- **回溯重建**: 从底部最小能量点回溯，找出完整的垂直接缝
- **内容感知**: 优先移除低能量（不重要）的像素，保持图像内容完整性
- **对比验证**: 同时生成简单缩放结果，展示内容感知缩放的优越性
