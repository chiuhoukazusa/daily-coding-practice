# Halton Sequence Quasi-Monte Carlo

## 项目简介
实现 Halton 低差异序列（Quasi-Monte Carlo），对比伪随机序列在 2D 均匀分布和数值积分方面的性能差异。通过量化指标（L2 Discrepancy、Star Discrepancy、Chi-squared、平均最近邻距离）和蒙特卡洛积分精度验证 Halton 序列的优越性。

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra -lm
./output
```

## 输出结果
- `halton_sequence.ppm` - Halton 序列 2D 分布图（蓝色）
- `random_sequence.ppm` - 伪随机序列 2D 分布图（红色）
- `comparison.ppm` - 左右对比图

## 技术要点
- **Halton 序列**: 基于不同素数基底的逆基数采样，生成低差异序列
- **L2 Discrepancy**: 衡量点集在 [0,1]² 上分布的均匀性
- **Star Discrepancy**: Koksma-Hlawka 不等式的核心度量
- **Quasi-Monte Carlo 积分**: 相比纯随机 MC，收敛速度从 O(1/√N) 提升到接近 O(1/N)
- **量化验证**: 8 项自动化检查确保结果正确性

## 量化结果（1024 点）
| 指标 | Halton QMC | Random MC | 改进倍数 |
|------|-----------|-----------|---------|
| L2 Discrepancy | 0.001257 | 0.012000 | 9.5x |
| Star Discrepancy | 0.004934 | 0.040328 | 8.2x |
| Chi-squared (10×10) | 21.7 | 115.0 | 5.3x |
| Mean NN Distance | 0.01959 | 0.01607 | 1.2x |
| π Estimation Error | 0.00684 | 0.01075 | 1.6x |

8/8 自动化验证全部通过。
