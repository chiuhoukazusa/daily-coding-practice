# Weighted Voronoi Stippling

基于加权Voronoi图的点画生成器。通过Lloyd迭代松弛算法，将点集分布调整为与输入图像密度匹配，产生经典的计算艺术点画效果。

## 核心算法

1. **密度加权**：从输入PPM图像计算每个像素的密度权重（暗区→高密度），应用gamma增强对比度
2. **拒绝采样初始化**：根据密度图用拒绝采样分布初始点集
3. **Voronoi分区**：为每个点构建Voronoi单元格（最近邻分配）
4. **加权Lloyd松弛**：将每个点移动到其单元格的密度加权质心
5. **迭代收敛**：重复步骤3-4直到平均位移低于阈值

## 量化验证

| 测试图像 | 收敛比 | 密度相关性(Pearson) | 覆盖率 |
|---------|--------|-------------------|--------|
| radial | 0.057 | 0.924 | 1.875% |
| checker | 0.059 | 0.952 | 1.875% |
| shapes | 0.071 | 0.985 | 1.875% |

- **收敛性**: 平均位移从 ~1.4px 降至 ~0.09px，收敛比 < 0.08
- **密度匹配**: Pearson相关系数均 > 0.92，点分布与图像密度强正相关
- **早期收敛**: 均在12次迭代内收敛（阈值0.2px）

## 编译运行

```bash
g++ main.cpp -o voronoi_stipple -std=c++17 -O2 -Wall -Wextra
./voronoi_stipple <input.ppm> <output_prefix> [num_points] [iterations]
```

## 输出文件

- `*_stipple.ppm`: 最终点画结果（黑点/白底）
- `*_voronoi_initial.ppm`: 初始Voronoi图
- `*_voronoi_final.ppm`: 最终Voronoi图（经Lloyd松弛后）
- `*_combined.ppm`: 密度图+红点叠加
- `*_convergence.txt`: 迭代收敛数据

## 技术要点

- Voronoi图加权质心计算
- 图像密度到点密度的映射
- Lloyd迭代算法的收敛性
- 拒绝采样的初始化策略
