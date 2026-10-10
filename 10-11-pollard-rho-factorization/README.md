# Pollard's Rho Integer Factorization — 随机化整数分解

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
```
=== Pollard's Rho Factorization — Quantitative Verification ===
Total test cases: 20
Failures: 0
PASS: all factors prime, reconstruct n, match trial division
Semiprime factorization time:
  Pollard Rho:  0.00211075 s
  Trial division: 4.21583 s
  Speedup: 1997.31x
Large prime factors found:
  2100000011 * 2200000009 = 4620000043100000099
  2300000041 * 2400000011 = 5520000123700000451
Miller-Rabin spot-checks:
  18446744073709551557 is_prime=1 (expect 1)
  18446744073709551615 is_composite=1 (expect 1)
```

## 技术要点
- **Pollard's Rho 伪随机序列**：用 `f(x) = (x^2 + c) mod n` 迭代生成序列，通过 Floyd 或 Brent 循环检测找到序列碰撞，进而用 `gcd(|x - y|, n)` 提取非平凡因子，期望复杂度 `O(n^1/4)`
- **Brent 循环检测**：相比 Floyd 的龟兔赛跑，Brent 通过固定跳跃步长（batch GCD）减少 `gcd` 调用次数，更高效地定位循环
- **Miller-Rabin 确定性素性测试**：对 64 位以内整数使用固定基数集 `{2,3,5,7,11,13,17,19,23,29,31,37}`，无需随机化即可得到确定结果
- **试除法小因子剥离**：先剥离 2 和所有小于阈值的素数，避免 Pollard Rho 在小因子上的性能浪费
- **`__uint128_t` 溢出安全乘法**：模乘用 128 位中间类型避免 64 位乘法溢出

## 量化验证（非视觉检查）
1. **因子素数性**：所有分解出的因子通过 Miller-Rabin + 试除法双重验证，全部为素数（20 组测试 0 失败）
2. **乘积重构验证**：`product(factors) == n`，因子乘积精确还原原整数，保证分解无遗漏/多余
3. **参考对比**：分解结果与试除法参考实现逐组匹配，0 处不一致
4. **性能加速比**：对两个 ~31 位素数的半素数乘积（约 64 位），Pollard Rho 仅 0.0021s，而试除法需 4.2158s，**加速 1997x**
5. **Miller-Rabin 边界 spot-check**：`18446744073709551557`（已知大素数）判为素数、`18446744073709551615`（=2^64-1，合数）判为合数，边界正确
