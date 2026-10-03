# Zero-One BFS Shortest Path

## 技术标签
图算法 / 最短路 / 0-1 BFS / Deque 双端队列 / O(V+E)

## 原理
0-1 BFS 解决边权仅为 0 或 1 的图的单源最短路问题。与普通 BFS 的队列不同，
它使用**双端队列（deque）**：

- 松弛到权为 0 的边 → 插入队头（`push_front`）
- 松弛到权为 1 的边 → 插入队尾（`push_back`）

这样始终保证队列中的距离单调不减，每个节点至多入队一次，时间复杂度
**O(V + E)**，优于堆优化的 Dijkstra 的 O(E log V)。

核心不变量：队列中的节点按当前距离有序（最多相差 1），因此无需优先队列即可
保证最短路最优性（对非负权，Dijkstra 是最优的，0-1 BFS 与之等价）。

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果（量化验证）

```
Test1 (random 0/1 graph, n=2000 m=12000): mismatches=0 -> PASS
Test2 (grid 300x300 with walls): mismatches=0 corner_dist=104 -> PASS
Test3 (perf, n=200000 m=1000000): 0-1 BFS=0.05s Dijkstra=0.08s speedup=1.65x
```

## 三项量化验证
1. **正确性**：随机 0/1 图上 0-1 BFS 与堆优化 Dijkstra 最短距离逐一比对，0 不匹配。
2. **网格墙穿行**：300×300 网格、60% 墙体（超过逾渗阈值），从角落到对角需击穿
   104 堵墙，0-1 BFS 与 Dijkstra 结果一致。
3. **性能**：100 万条边的大图上，0-1 BFS 约比 Dijkstra 快 1.65 倍。

## 技术要点
- 0-1 BFS 用 `deque` 替代 `priority_queue`，将 O(E log V) 降至 O(V+E)
- 权 0 进队头、权 1 进队尾，维护「距离单调不减」不变量
- 与 Dijkstra 等价性验证（非负权最短路唯一性）
- 超过逾渗阈值的网格场景，强制路径必须「穿墙」，测试有意义
