# RANSAC Robust Model Fitting

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果

生成 `ransac_output.txt`，包含四个量化验证实验的报告：

1. **直线拟合**：不同离群点比例（0%~80%）下 RANSAC vs 最小二乘参数误差对比
2. **圆拟合**：20%/50%/70% 离群点下 RANSAC vs 最小二乘对比
3. **内点率验证**：RANSAC 识别的内点数召回率
4. **无离群点一致性**：零离群点时 RANSAC 与 OLS 应一致

```text
✅ 全部验证通过
```

## 技术要点

- **RANSAC（Random Sample Consensus）**：随机采样最小子集 → 估计模型 → 统计内点 → 迭代取最优
- **直线拟合**：两点定直线，`dist = |ax - y + b| / sqrt(a² + 1)` 距离判定
- **圆拟合**：三点定圆（行列式法），几何距离 `|hypot(x-cx, y-cy) - r|` 判定
- **最小二乘基准**：直线 OLS + 圆 Kasa 线性化法，用于对比退化表现
- **内点精修**：RANSAC 粗估后，用内点集合重新做最小二乘，提升精度

## 核心结论

在存在大量离群点（outliers）时，最小二乘法会被离群点严重"拉偏"，而 RANSAC 通过
随机一致性采样能鲁棒地估计出真实模型参数。离群点越多，RANSAC 相对最小二乘的优势越显著。
