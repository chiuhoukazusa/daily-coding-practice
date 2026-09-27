# Binary Lifting LCA & K-th Ancestor

**日期**: 2026-09-28
**方向**: 树算法 / 数据结构
**核心技术**: 最近公共祖先 LCA / Binary Lifting 倍增 / Euler Tour / K-th Ancestor / 深度对齐

## 概述

**Binary Lifting（倍增法 / 二进制提升）** 是在一棵有根树上回答**最近公共祖先（LCA）**
与**第 k 级祖先（K-th Ancestor）**查询的经典技术，将每次查询从朴素逐级上跳的 `O(depth)`
优化到 `O(log n)`。

核心思想是预处理一个 `up[v][j]` 表，表示顶点 `v` 的**第 `2^j` 级祖先**（`2^j` 代祖先），
通过递推式：

```
up[v][0]   = parent[v]
up[v][j]   = up[ up[v][j-1] ][ j-1 ]   // 先跳 2^(j-1) 步，再跳 2^(j-1) 步 = 2^j 步
```

预处理只需 `O(n log n)` 时间与空间（`LOG = floor(log2 n) + 1`），之后每次查询 `O(log n)`。

## 实现要点

- **预处理 `build(root)`**：一遍 DFS 记录每个节点的 `depth` 与 `up[v][0]`（父节点），
  再递推填满整个 `up` 表。
- **LCA 查询 `lca(u, v)`** — 三步法：
  1. **深度对齐**：把较深的节点 `u` 用二进制分解上跳到与 `v` 同深度；
  2. 若此时 `u == v`，直接返回；
  3. **从高位到低位同时上跳**：从 `LOG-1` 递减枚举 `j`，若 `up[u][j] != up[v][j]` 则同时
     上跳 `2^j` 步，最终 `u`、`v` 停在 LCA 的正下方，返回 `up[u][0]`。
- **第 k 级祖先 `kthAncestor(v, k)`**：将 `k` 做二进制分解，对每个置位位 `j` 执行
  `v = up[v][j]`；若中途 `v == -1`（越界）则返回 `-1`。

## 量化验证（全部通过）

1. **正确性对比朴素 LCA**：构造 `n=2000` 的随机树，抽样 `Q=50000` 组随机查询：
   - LCA 结果与朴素逐级上跳实现**完全一致**；
   - K-th Ancestor 结果与朴素向上走 k 步**完全一致**；
   - LCA 深度一致性（`depth[lca] <= min(depth[u], depth[v])`）成立；
   - 合计 **150000 / 150000 项全部通过（100.0000%）**。
2. **时间复杂度对比（最坏情况深链）**：构造 `n=200000` 的深链树（朴素 LCA 最坏 `O(depth)`），
   `Q=100000` 组查询：
   - 朴素 LCA 耗时 **15691.69 ms**；
   - Binary Lifting 耗时 **33.36 ms**；
   - **加速比 470.3x**，且两种实现结果 **100000 / 100000 完全一致**。

## 构建与运行

```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output > binary_lifting_output.txt
```

## 关键技术标签

Binary Lifting, 倍增法, LCA最近公共祖先, K-th Ancestor第k级祖先, Euler Tour, 深度对齐, 位运算二进制分解, O(log n), 朴素LCA基准对比, up表递推
