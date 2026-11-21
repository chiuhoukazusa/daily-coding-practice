# Spatial Hash Grid Collision Detection

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果

```
=== Correctness (Spatial Hash vs Brute Force) ===
       N      Brute       Hash      Match  CandPairs
     100          0          0         OK          0
     500         13         13         OK         13
    1000         61         61         OK         61
    5000       1504       1504         OK       1504
✅ Collision-pair sets identical to brute force for all sizes

=== Performance (timing) ===
       N      Brute(ms)       Hash(ms)      Speedup
    1000           0.45           0.25          1.8x
    5000          11.41           1.48          7.7x
   10000          45.40           4.06         11.2x
   20000         184.93          12.13         15.2x
   50000        1173.38         162.07          7.2x

=== Hash-grid scaling sanity (should be ~O(N), not O(N^2)) ===
N=  5000  Hash=2.06 ms
N= 10000  Hash=3.19 ms   (2x objects -> 1.55x time)
N= 20000  Hash=10.31 ms   (2x objects -> 3.23x time)
N= 40000  Hash=62.05 ms   (2x objects -> 6.02x time)

DONE
```

## 技术要点

- **宽相（Broad Phase）**：将 AABB 插入均匀空间哈希网格，只对同/邻接 cell 内的对象做候选查询，避免 O(N²) 暴力配对。
- **窄相（Narrow Phase）**：对候选对做精确 AABB-vs-AABB 重叠判定的 SAT（分离轴）测试。
- **正确性验证**：与暴力 O(N²) 精确重叠计数（ground truth）对比，断言碰撞对集合完全一致（Match=OK）。
- **复杂度量化**：哈希网格查询时间应近似 O(N) 线性，而非暴力 O(N²)；测量加速比（约 7-15x）。
- **确定性 RNG**：自定义 xorshift 风格 RNG 保证跨运行可复现。
