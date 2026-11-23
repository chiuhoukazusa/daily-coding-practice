# DDA Voxel Grid Traversal (Amanatides-Woo)

日期：2026-11-24
方向：图形学 / 游戏开发

## 技术点

- **Amanatides-Woo 3D-DDA** (`VoxelTraversal::traverse`)：参数化体素网格光线遍历，`tMax/tDelta` 步进，逐轴推进找出射线穿过的所有体素单元格
- **世界坐标 → 体素坐标** (`floor` 映射)：设体素边长为 1，网格原点在 (0,0,0)
- **supercover 基准** (`supercover`)：解析式 26-连通超覆盖，含面/棱相切退化单元格
- **稠密采样基准** (`bruteForce`)：200000 步逐点采样的朴素遍历，用于性能对比

## 核心算法

给定射线 `P(t) = o + t·d`，体素边长 1：

```
tDeltaAxis = |1 / d.axis|            // 穿过单位体素所需参数增量
tMaxAxis   = (边界 - o.axis) / d.axis // 到达下一体素边界的参数

// 每步取最小 tMax 轴推进，直至越界或超过射程
```

## 量化验证结果（不能靠眼睛看）

| 测试 | 内容 | 结果 |
|------|------|------|
| 起点正确性 | DDA 首个 cell == floor(原点) | 2000/2000 ✅ |
| 无遗漏(误检) | DDA 输出 ⊆ supercover | 0 违例 ✅ |
| 连续性 | 相邻 cell Chebyshev 距离 == 1 | 0 违例 ✅ |
| 性能加速比 | DDA vs 稠密采样基准 | ~690x ✅ |

**结论：✅ 全部通过 (PASS)**

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```
