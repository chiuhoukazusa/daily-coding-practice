# Union-Find Disjoint Set Union

并查集（Disjoint Set Union / Union-Find），支持路径压缩（path compression）与按秩合并（union by rank），摊还复杂度接近 O(α(n))（阿克曼反函数，可视为常数）。

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2
./output > union_find_output.txt
```

## 输出结果

![结果](union_find_output.txt)

## 技术要点

- **路径压缩（path compression）**：`find` 时迭代地把当前节点的父指针拉向根（halving 变体），摊还后近似 O(1)。
- **按秩合并（union by rank）**：总是把秩小的树挂到秩大的树下，树高被严格控制在 `O(log n)`。
- **连通性判定**：`connected(a,b)` 即 `find(a) == find(b)`，Kruskal 最小生成树的核心原语。
- **暴力基准对比**：朴素并查集（无压缩/无按秩）在退化成链时趋近 O(n²)，用于量化加速比。
- **Kruskal 应用**：用 DSU 实现最小生成树，验证 union-find 作为图论基础设施的正确性。

## 验证结果（5 组测试全部 PASS）

- TEST 1：显式小图连通性正确性
- TEST 2：随机交叉验证（49954 次 union+query vs 朴素基准）
- TEST 3：Kruskal MST 应用（weight=29720, edges=199）
- TEST 4：随机混合 N=500000 OPS=1000000 —— DSU 0.052s，朴素基准退化为近二次（病理表现）
- TEST 5：按秩合并深度上界 —— max_depth=20 ≤ log2(N)=20
