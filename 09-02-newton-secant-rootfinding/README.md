# Newton-Raphson & Secant Root Finding

数值方法：求根算法的收敛阶量化验证。

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果

文本输出 `newton_secant_output.txt`，包含三组测试函数 + 多重根修正对比 + 断言式量化验证。

## 技术要点

- **Newton-Raphson**：需要解析导数，简单根二次收敛（order ≈ 2）
- **Secant 割线法**：无需导数，用两点差分逼近导数，超线性收敛（order ≈ 1.618 黄金比）
- **多重根退化**：普通 Newton 在三重根退化为线性收敛（order ≈ 1），改进 Newton（m 重因子修正）恢复二次收敛
- **收敛阶估计**：通过相邻误差比的对数估计 `order ≈ log(e_{k+1}/e_k) / log(e_k/e_{k-1})`
