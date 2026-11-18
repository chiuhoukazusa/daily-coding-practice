# Chinese Remainder Theorem (CRT) + Modular Arithmetic

日期：2026-11-19
方向：算法 / 数论

## 技术点

- **扩展欧几里得算法** (`ext_gcd`)：求解模逆元 `a^{-1} mod m`
- **快速幂** (`mod_pow`)：`O(log e)` 二进制幂
- **经典 CRT** (`crt_coprime`)：互质模数下解同余方程组，使用 `__int128` 与安全模乘避免溢出
- **Garner 算法** (`garner_crt`)：混合进制表示，逐位解码
- **暴力基准** (`brute_crt`)：在 `[0, M)` 内线性搜索

## 核心公式

给定 `x ≡ r_i (mod m_i)`，`m_i` 两两互质，令 `M = ∏ m_i`，`M_i = M / m_i`：

```
x = Σ r_i · M_i · (M_i^{-1} mod m_i)   (mod M)
```

## 量化验证结果（不能靠眼睛看）

| 测试 | 内容 | 结果 |
|------|------|------|
| Test 1 | 模逆元正确性 `a·inv ≡ 1 (mod m)` | 175/175 ✅ |
| Test 2 | 快速幂 vs 朴素逐次乘 | 500/500 ✅ |
| Test 3 | CRT 经典 == Garner == 暴力基准 | 2000/2000 ✅ |
| Test 4 | 唯一性 `0 ≤ x < M` | 500/500 ✅ |
| Test 5 | 大模数 (>2^31)，`__int128` 无溢出 | PASS ✅ |
| Test 6a | 6 个模数 (M≈1.7e12) 重构验证 | PASS ✅ |
| Test 6b | 性能：**CRT 0.00016 ms/op vs 暴力 2.99 ms/op，加速比 ≈18287x** | ✅ |

## 踩坑记录

1. **Garner 算法混合进制 bug**：解码 `acc` 时错误地用了 `coeff[j] · m_j`，正确应为
   `coeff[j] · (m_0·m_1·...·m_{j-1})`（前缀积）。修复后 2000 例全部通过。
2. **`__int128` 溢出**：经典 CRT 的 `r_i · M_i · t_i` 中间量可达 ~1e45，超过 `__int128`
   上限 (≈1.7e38)，需用安全模乘 `mul_mod` 分段取模。
3. **编译器死代码消除**：`-O2` 会把纯函数 `brute_crt` 的重复调用常量折叠/消除，
   导致计时异常（ns 级假速度）。用 `__attribute__((noinline))` + `volatile` sink 修复。

## 编译运行

```bash
g++ main.cpp -o crt -std=c++17 -O2 -Wall -Wextra
./crt
```
