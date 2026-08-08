# GMRES Krylov Subspace Solver

## 编译运行
```bash
g++ main.cpp -o gmres -std=c++17 -O2 -Wall -Wextra
./gmres
```

## 输出结果
- `gmres_convergence.ppm` - Restart size convergence comparison (m=5,10,20,30)
- `gmres_cg_comparison.ppm` - SPD system: GMRES vs CG convergence rate comparison

## 9项量化验证
| # | 测试 | 结果 |
|---|------|------|
| 1 | SPD系统GMRES vs CG等价性 | ✅ CG: 2.15e-09, GMRES: 5.19e-09 |
| 2 | 非对称线性系统收敛 | ✅ 残差 3.09e-09 |
| 3 | 残差单调递减 | ✅ 缩减因子 3.48e+07 |
| 4 | Givens旋转正交性 | ✅ 误差 < 1.12e-16 |
| 5 | Arnoldi Hessenberg结构 | ✅ i>j+1 ⇒ 0 |
| 6 | 后向误差分析 | ✅ ‖b-Ax‖/‖b‖ = 3.09e-09 |
| 7 | 重启尺寸收敛对比 | ✅ m=5,10,20,30 均收敛 |
| 8 | 精确解恢复 | ✅ 相对误差 9.91e-10 |
| 9 | SPD: CG vs GMRES 收敛率 | ✅ CG: 10 iters, GMRES: 11 iters |

## 技术要点
- GMRES(m): 带重启的广义最小残量法
- Arnoldi迭代构建Krylov子空间正交基
- Modified Gram-Schmidt正交化
- Givens旋转求解最小二乘问题
- Hessenberg矩阵结构验证
- 对称正定系统与CG等价性验证
- 非对称系统收敛性验证
