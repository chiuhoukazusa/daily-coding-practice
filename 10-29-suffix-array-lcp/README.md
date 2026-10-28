# Suffix Array & LCP (Prefix-Doubling + Kasai)

**日期**: 2026-10-29

## 技术点
- **后缀数组 (Suffix Array)**: 前缀倍增 (prefix-doubling) 排序，O(n log n)
- **LCP 数组 (Kasai 算法)**: 基于逆排名 + 高度单调性，O(n)
- **RMQ 查询**: 利用 LCP 数组的区间最小值求任意两个后缀的最长公共前缀

## 实现
- `build_suffix_array`: 前缀倍增（每轮按 `(rank[i], rank[i+k])` 基数排序）
- `build_lcp`: Kasai 算法（利用 `h` 值单调递减性质）
- `naive_suffix_array` / `naive_lcp`: 暴力基准，用于正确性对比

## 量化验证结果（非视觉）
| 验证项 | 结果 |
|---|---|
| 随机字符串正确性 (500 组, 与朴素对比) | ✅ ALL MATCH |
| 边界用例 (12 组) | ✅ 全部 OK |
| 性能 n=200,000 (前缀倍增+Kasai) | ~40 ms |
| 朴素 n=4,000 耗时 | ~0.53 ms |
| 朴素外推至 n=200,000 (O(n²log n)) | ~1961 ms |
| **加速比** | **~49x** |
| LCP 值边界合法性 | ✅ YES |
| 任意两后缀 LCP (RMQ-min, 100/100 spot check) | ✅ 全部匹配 |

## 结论
RESULT: **PASS**
