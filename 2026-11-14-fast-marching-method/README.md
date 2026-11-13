# Fast Marching Method (Eikonal Geodesic Distance)

## 简介
求解静态 Eikonal 方程 $|\nabla u| = F(x)$ 的快速行军法（FMM）。采用迎风有限差分 + 最小堆（类 Dijkstra 的 "trial" 前沿传播），从点源出发计算必须绕开障碍物的测地距离场。

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
- 无障碍距离场：`fmm_free.png`
- 有障碍距离场：`fmm_obstacle.png`

## 技术要点
- 迎风二次（Godunov）求解器：单侧邻居取 min，二次方程解析更新
- 最小堆 trial 前沿传播（Frozen/Trial/Far 三态）
- 障碍物设为 F=INF，实现测地绕行，验证不可达区域无泄漏
- 定量验证：无障碍时与解析欧氏距离比较（max-abs-error + RMSE，一阶离散误差）
- Dijkstra vs FMM 更新对比，展示迎风二次求解的更高精度
