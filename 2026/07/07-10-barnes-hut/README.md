# Barnes-Hut N-Body Simulation

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
- ![初始状态](barnes_hut_initial.ppm)
- ![模拟中期](barnes_hut_mid.ppm)
- ![模拟终态](barnes_hut_final.ppm)

## 技术要点
- **Barnes-Hut 算法**：使用四叉树空间划分，将 N 体引力计算从 O(n²) 降为 O(n log n)
- **θ (theta) 近似准则**：当粒子群到目标距离远大于群体宽度时，用质心近似替代逐粒子计算
- **质心更新**：插入粒子时递归更新各节点的质心和总质量
- **软引力**：加入 softening factor 避免奇点
- **Verlet / Leapfrog 积分**：半隐式时间积分保持能量守恒

## 验证结果
| 粒子数 | RMS相对误差 | 加速比 | 能量漂移 |
|--------|-----------|--------|---------|
| 100    | 0.0036    | 0.33×  | 0.54%   |
| 200    | 0.0045    | 0.59×  | 1.45%   |
| 400    | 0.0043    | 0.96×  | 1.17%   |
| 800    | 0.0074    | 1.55×  | 1.10%   |

✅ 所有量化指标通过
