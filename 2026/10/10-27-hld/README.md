# Heavy-Light Decomposition (Path Queries)

## 简介
树链剖分（HLD）：将一棵树剖分成若干条「重链」，使任意树路径可拆成 O(log n) 段连续区间，
配合线段树在 O(log² n) 内回答路径和 / 路径最大值查询。

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 验证方式（量化）
- 随机生成 n = 50 / 200 / 1000 / 5000 的树，随机点权，1000 条随机查询
- HLD 结果与朴素 DFS 遍历路径（精确）逐条比对：SUM 与 MAX 100% 一致
- 统计平均跳链次数，验证 O(log n) 复杂度（实测 2.99 → 6.61，n 增长 100 倍）

## 技术要点
- 第一次 DFS 求子树大小、重儿子（子树最大的孩子）
- 第二次 DFS 优先重儿子生成 dfn 序，保证重链连续
- 线段树维护区间和 / 区间最大值
- 路径查询按 head 逐段跳链，O(log n) 段 × O(log n) 线段树 = O(log² n)
