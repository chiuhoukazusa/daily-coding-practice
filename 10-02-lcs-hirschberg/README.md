# LCS (Longest Common Subsequence) — Hirschberg 线性空间算法

求解两个字符串的**最长公共子序列（Longest Common Subsequence, LCS）**，重点对比三种实现的**时间-空间权衡**，最终用 Hirschberg 分治算法在 **O(min(n,m)) 线性空间**内完成 LCS 长度的精确求解与子序列的重建。

## 问题描述

给定两个字符串 `a` 与 `b`，长度分别为 `n` 与 `m`，求既是 `a` 的子序列、又是 `b` 的子序列的最长序列（子序列不要求连续，只要求保持相对顺序）。

经典例子：`ABCBDAB` 与 `BDCABA` 的 LCS 为 `BDAB`（长度 4）。

**核心矛盾**：朴素 DP 需要 O(n·m) 的空间保存整张表用于回溯重建子序列——当 n、m 都到 20000 时，`int` 表需要约 **1.5 GB** 内存，完全不可接受。Hirschberg 分治把空间降到 O(min(n,m))，实测 RSS 仅约 **3.5 MB**。

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra -O2
./output

# 内存占用对照实验
g++ mem_test.cpp -o mem_test -std=c++17 -O2
./mem_test
```

## 三种实现对比

| 实现 | 时间复杂度 | 空间复杂度 | 能否重建子序列 |
|------|-----------|-----------|---------------|
| Naive 全表 DP（参考基准） | O(n·m) | O(n·m) | ✅ 完整回溯 |
| 空间优化 DP（仅求长度） | O(n·m) | O(min(n,m)) | ❌ 无法回溯 |
| **Hirschberg 分治（本题核心）** | O(n·m) | O(min(n,m)) | ✅ 递归重建 |

## 验证结果

- **正确性**：206 个随机/结构化测试用例全部通过（206/206，0 失败）。
- **三实现长度一致**：naive 长度、空间优化长度、Hirschberg 重建出的 LCS 长度三者完全相等。
- **子序列有效性**：Hirschberg 重建出的字符串被验证确为两个输入串的公共子序列。
- **性能**（n=m=2000，字母表=4）：
  - naive 全表 O(nm)：~10.4 ms
  - 空间优化 O(min)：~1.9 ms
  - Hirschberg 递归：~5.4 ms（收敛到与 naive 一致）
- **内存**（n=m=20000，全相同字符）：naive 需 ~1526 MB，Hirschberg 实际 RSS ~3.5 MB。

## 技术要点

- 经典 O(n·m) 动态规划递推 `dp[i][j] = a[i]==b[j] ? dp[i-1][j-1]+1 : max(dp[i-1][j], dp[i][j-1])`
- 空间优化：每一行只依赖上一行，用滚动数组把状态降到 O(min(n,m))
- **Hirschberg 分治**：折半分割 + 前向/反向 LCS 长度数组求最优分割点，递归求解，把重建所需空间从 O(n·m) 降到 O(min(n,m))
- 量化验证：长度跨实现交叉核对 + 公共子序列性质验证 + 内存 RSS 实测对比
