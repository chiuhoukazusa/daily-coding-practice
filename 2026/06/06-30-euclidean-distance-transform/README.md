# Euclidean Distance Transform (欧氏距离变换)

## 概述
实现8SSEDT（八点符号顺序欧氏距离变换）算法，将二值图像转换为欧氏距离场，并为每个像素计算最近的前景点（Voronoi分割）。

## 编译运行
```bash
g++ main.cpp -o edt -std=c++17 -O2
./edt
```

## 输出结果
![距离场可视化](distance_field.png)
![Voronoi分割](voronoi_partition.png)

## 技术要点
- **8SSEDT算法**: 4-pass顺序传播，8邻域精确欧氏距离计算
- **距离场验证**: 前景点零距离检查、Lipschitz连续性验证、暴力搜索精度对比
- **Voronoi分割**: 通过最近前景点ID生成分区可视化
- **多形状测试**: 圆形、矩形、对角线、孤立点四种形状验证算法鲁棒性
