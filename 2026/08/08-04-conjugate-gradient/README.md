# Conjugate Gradient Solver - 共轭梯度法稀疏线性系统求解器

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果

程序通过5个测试验证CG算法的正确性：

1. **1D Poisson Equation**: N=200，1次迭代收敛，相对残差 1.5e-12
2. **2D Poisson Equation**: 4096x4096 稀疏矩阵，1次迭代收敛，L∞误差 1.95e-4
3. **Residual Monotonic Convergence**: 50次迭代，衰减因子 0.9133
4. **Scalability Test**: 10000x10000 矩阵，187次迭代收敛
5. **Residual Convergence Profile**: 多尺寸 (N=32/64/128/256) 收敛曲线

残差收敛数据输出至 `residual_profile.csv`。

## 技术要点

- **CSR 稀疏矩阵存储**: Compressed Sparse Row 格式，节省 O(n²)→O(nnz) 内存
- **Conjugate Gradient 迭代法**: 适用于对称正定稀疏系统，无矩阵-矩阵运算
- **Poisson 方程离散化**: 1D 和 2D 五点差分格式构造正定对称系数矩阵
- **收敛性分析**: 残差单调递减、收敛速度与条件数的关系
- **量化验证**: L2/L∞误差、残差衰减因子、多尺度收敛曲线

## CG 算法核心步骤

```
1. r₀ = b - A·x₀, p₀ = r₀
2. αₖ = (rₖ·rₖ) / (pₖ·A·pₖ)
3. xₖ₊₁ = xₖ + αₖ·pₖ
4. rₖ₊₁ = rₖ - αₖ·A·pₖ
5. βₖ = (rₖ₊₁·rₖ₊₁) / (rₖ·rₖ)
6. pₖ₊₁ = rₖ₊₁ + βₖ·pₖ
```
