# QR Decomposition (CGS / MGS / Householder)

## 概述
实现三种 QR 分解算法：**Classical Gram-Schmidt (CGS)**、**Modified Gram-Schmidt (MGS)** 和 **Householder 反射**，并对正交性、重构精度及病态矩阵鲁棒性进行量化验证。

## 编译运行
```bash
g++ main.cpp -o qr_decomposition -std=c++17 -O2 -lm
./qr_decomposition
```

## 技术要点
- CGS 算法：最直观但数值不稳定
- MGS 算法：通过对列正交化顺序的优化提高数值稳定性
- Householder 反射：引入反射变换实现最佳数值精度
- ||A - QR||_F 重构误差对比三种算法
- ||Q^T Q - I||_F 衡量正交性质量
- Hilbert 矩阵作为典型 ill-conditioned 测试案例
- QR 分解解线性方程组：Rx = Q^T b → 回代
- 7 项量化验证指标全面对比

## 输出结果
![结果](output.png)

## 验证指标
1. CGS/MGS/Householder 的重构误差 ||A - QR||
2. 正交性误差 ||Q^T Q - I||
3. R 上三角性验证
4. 病态矩阵 (Hilbert) 下各算法表现
5. 线性方程组求解精度
6. 运行时间对比
7. 随机矩阵批量验证
