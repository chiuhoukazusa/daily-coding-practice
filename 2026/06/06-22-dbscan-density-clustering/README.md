# DBSCAN Density Clustering

## 概述
DBSCAN (Density-Based Spatial Clustering of Applications with Noise) 密度聚类算法实现。支持任意形状聚类发现和自动噪声点检测，无需预先指定聚类数量。

## 编译运行
```bash
g++ main.cpp -o dbscan -std=c++17 -O2
./dbscan
```

## 输出结果
4 个测试数据集，每个生成一张聚类可视化图片：

| 数据集 | 图案 | 特点 |
|--------|------|------|
| blobs | ![blobs](dbscan_blobs.png) | 5 个高斯团块 + 噪声 |
| moons | ![moons](dbscan_moons.png) | 两个弧形非凸簇 |
| circles | ![circles](dbscan_circles.png) | 同心圆环 (嵌套集群) |
| aniso | ![aniso](dbscan_aniso.png) | 三个各向异性条形簇 |

## 量化结果
- 全部 4 个数据集聚类数量验证 ✅ PASS
- 平均轮廓系数 (Silhouette): 0.4891
- 噪声率范围: 1.7% ~ 6.5%

## 技术要点
- **DBSCAN 核心算法**: ϵ-neighborhood 区域查询, 核心点/边界点/噪声点三分类
- **种子扩展**: 从核心点出发，通过邻域查询递归扩展聚类簇
- **轮廓系数评估**: 自动计算聚类质量 (内聚度 vs 分离度)
- **PPM 可视化**: Golden-angle 色相分布确保不同聚类颜色分明
- **4 种测试数据集**: Gaussian blobs, moons, concentric circles, anisotropic elongated clusters
- **对比 K-Means**: DBSCAN 不需要预设 k，能发现任意形状聚类，自动排除噪声
