# Dijkstra Shortest Path (Binary Heap) — 二叉堆优化的 Dijkstra 最短路

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
```
=== Dijkstra (Binary Heap) Verification ===
Random tests: 60 graphs (V=2..81, density 5%..95%)
Total single-source distance checks: 2326
Correct (match Floyd-Warshall & O(V^2) baseline): 2326
Failures: 0
Max abs error vs reference: 0

=== Benchmark (sparse V=20000 E=100000) ===
Heap Dijkstra: 4.71 ms
O(V^2) baseline: 978.24 ms
Speedup: 207.55x
Sparse result mismatch count: 0

=== Benchmark (dense V=3000) ===
Heap Dijkstra: 8.41 ms
O(V^2) baseline: 44.26 ms
Speedup: 5.26x
Dense result mismatch count: 0

RESULT: PASS
```

## 技术要点
- **二叉堆 + 惰性删除（lazy deletion）**：用 `priority_queue` 维护 `(dist, vertex)` 最小堆，取出堆顶后若 `d > dist[u]` 说明是陈旧条目，直接 `continue` 跳过，无需 `decrease-key`，整体复杂度 `O((V+E) log V)`
- **贪心最短路**：每次从堆中取出当前确定的最短距离节点，对其所有出边做松弛（relaxation），`nd = d + e.w` 更小时更新并压入堆
- **双重正确性基准**：与 Floyd-Warshall（全源正确参考）以及 O(V²) 朴素 Dijkstra（稠密邻接矩阵版本）交叉验证，随机图 60 组、2326 个单源距离检查全部一致
- **随机图生成**：顶点数 V=2..81、边密度 5%..95%、正权值 1..1000，允许不连通图以覆盖 `INF`（不可达）边界

## 量化验证（非视觉检查）
1. **正确性量化**：60 组随机图中，堆版 Dijkstra 与 Floyd-Warshall 参考 + O(V²) 朴素版的单源最短距离**逐点完全一致**（0 失败，最大绝对误差 0）
2. **稀疏图加速比**（V=20000, E=100000）：堆版 4.71ms vs O(V²) 978.24ms，**加速 207.55x**，结果 0 处不一致
3. **稠密图加速比**（V=3000）：堆版 8.41ms vs O(V²) 44.26ms，**加速 5.26x**，结果 0 处不一致——直观体现堆优化在稀疏图上的巨大收益，以及稠密图下 O(V²) 缓存友好的对比
