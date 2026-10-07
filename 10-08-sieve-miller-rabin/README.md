# Sieve of Eratosthenes & Miller-Rabin Primality — 埃氏筛 + Miller-Rabin 素性测试

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
```
pi(10)=4, pi(100)=25, ..., pi(10^7)=664579  (全部匹配参考值)
[PASS] prime-counting pi(n) matches reference for n=10..10^7
[PASS] segmented sieve outputs exactly match naive sieve on 7 ranges
[PASS] Miller-Rabin agrees with sieve on [2, 10^6] (999999 numbers, 0 mismatches)
[PASS] Miller-Rabin identifies 8 known large primes
[PASS] Miller-Rabin rejects 12 Carmichael numbers (no false positive)
[PASS] Miller-Rabin rejects strong pseudoprimes (deterministic base set)
[PASS] MR vs trial-division agree on 10000 ~1e12..1e15 numbers (count=612)
    Miller-Rabin: 3.9 ms | trial division: 27472 ms | speedup ~6970x
[PASS] segmented sieve [1e9, 1e9+1e6]: 48155 primes in 5.8 ms
========================================
ALL TESTS PASSED (9 checks)
```

## 技术要点
- **埃氏筛（Sieve of Eratosthenes）**：O(n log log n) 标记合数，i² 起步优化，`vector<bool>` 位压缩减少缓存未命中；前缀和得到 π(n) 素数计数
- **分段筛（Segmented Sieve）**：只分配 `[L, R]` 区间的位数组 + `sqrt(R)` 的小素数表，使 `[1e9, 1e9+1e6]` 只需 O(1e6) 内存而非 O(1e9)，突破单数组内存上限
- **Miller-Rabin 素性测试**：基于费马小定理的强伪素数检测，将 `n-1 = d·2^s` 分解，取随机基 a 检验 `a^d≡±1 mod n`；对 64 位整数用前 12 个素数作**确定性基集**（无需随机化，结果严格正确）
- **`__int128` 模乘 + 快速幂**：避免 64 位乘法溢出，实现 O(log n) 模幂
- **关键陷阱**：`10^18 + 7` 是**合数**（常被误当质数），本实现用真实质数 `10^18 + 3` 和 `999999999999999989` 验证

## 量化验证（非视觉检查）
1. **π(n) 参考值**：n=10^1..10^7 七档与已知素数计数严格相等（4/25/168/1229/9592/78498/664579）
2. **分段筛一致性**：7 组（含重叠/边界）区间输出与朴素筛逐元素相等
3. **Miller-Rabin 正确性**：对 [2, 10^6] 全部 999999 个数与筛法结果零差异
4. **大质数识别**：8 个已知 64 位大质数（含 Mersenne 质数 2^61-1）全部正确判定
5. **Carmichael/强伪素数**：12 个 Carmichael 数 + 5 个强伪素数均被正确拒绝，无假阳性
6. **性能量化**：Miller-Rabin 相对试除法加速 **~6970x**（1e12~1e15 区间 10000 个数）
