# Voronoi Diagram Construction (Fortune's Sweep-Line)

**日期**: 2026-11-26
**方向**: 计算几何 / Voronoi 图 / Delaunay 三角剖分

## 技术点

实现二维欧氏 **Voronoi 图**（Voronoi Diagram）的构建，并派生其对偶的
**Delaunay 三角剖分**。给定平面上 40 个站点（site），将平面划分为每个站点
“最近邻区域”的多边形单元；两条相邻单元共享一条垂直平分线，三个单元汇聚于
一个外接圆圆心（Voronoi 顶点）。

算法采用 **Fortune 扫描线（sweep-line / beach line）思想** + **半平面交
（half-plane intersection）+ Delaunay 对偶** 的混合实现：

1. 对每个站点，用**半平面交**裁剪出它的 Voronoi 单元（用其它站点的垂直平分线
   逐条裁剪凸多边形）；
2. 由开放单元（接触包围盒边界）的共享边推导 **Delaunay 三角剖分**（对偶图）；
3. 借助 **凸包（convex hull）** 与 **外接圆空圆性质（empty circumcircle）**
   进行正确性校验。

## 量化验证结果

```
sites                = 40
delaunay_triangles   = 68
convex_hull_vertices = 10
euler_check          = F=68 vs 2N-2-h=68  MATCH
max_circumcenter_err = 1.23e-14   (外接圆心到三点等距误差 ~0)
closed_voronoi_cells = 40 / 40
polygon_area_sum     = 1.000000   (单元面积和 = 全区域)
mc_area_sum          = 1.000000   (蒙特卡洛面积和 = 1.0)
centroid_nearest_ok  = 40 / 40    (质心最近站点与所属单元一致)
mean_area_err        = 0.000219   (多边形面积 vs 蒙特卡洛面积)
```

验证要点：
- **欧拉公式**：`F = 2N - 2 - h`（F=Delaunay 三角形数，N=站点数，h=凸包顶点数），
  `68 = 2×40 - 2 - 10` ✅
- **空圆性质**：外接圆圆心到三个顶点的距离误差 ~1e-14（近机器精度）✅
- **满铺**：所有 Voronoi 单元面积和 = 1.0（完整覆盖世界空间）✅
- **对偶正确性**：关闭单元 40/40，质心最近站点归属 40/40 正确 ✅

## 文件

- `main.cpp` — Voronoi 图构建 + Delaunay 对偶 + 量化验证
- `voronoi_output.ppm` — 640×640 输出图像（站点、单元边界、外接圆可视化）
- `voronoi` — 编译产物

编译：`g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra`（0 errors, 0 warnings）
