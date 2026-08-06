# LU Decomposition (Doolittle Algorithm)

## 项目简介
实现 Doolittle LU 分解算法，用于求解线性方程组 Ax = b。支持多重右手边向量（multi-RHS）高效求解，包含条件数估计。

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
![结果](lu_decomposition_report.ppm)

## 技术要点
- **Doolittle 算法**：L 矩阵对角线为 1，U 为上三角矩阵
- **前代/回代**：Ly = b → Ux = y
- **LU 分解精度验证**：||A - LU||_F 测量分解误差
- **残差验证**：||Ax - b|| / ||b|| 测量求解精度
- **条件数估计**：||A||_1 · ||A^{-1}||_1 评估矩阵病态程度
- **9 组测试**：覆盖 3×3 手算验证、5×5~100×100 对角占优/SPD/一般矩阵、Hilbert 病态矩阵、多 RHS 求解

## 验证结果
- ||A-LU||_F ≤ 2.5×10⁻¹⁵（所有测试）
- ||Ax-b||/||b|| ≤ 7.5×10⁻¹⁶（所有测试）
- Hilbert 病态矩阵（κ₁≈5.8×10¹³）下仍精确分解
- 100×100 矩阵求解精度达 7.3×10⁻¹⁶
