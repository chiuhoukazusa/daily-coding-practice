# Manacher's Algorithm — 最长回文子串 (O(n))

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
```
[known-cases] pass=7 fail=0
[random-correctness] trials=5000 pass=5000 fail=0
[complexity] len=1000 manacher_cmps=1999 brute_cmps=500500 ratio=250.375x
[complexity] len=2000 manacher_cmps=3999 brute_cmps=2001000 ratio=500.375x
[complexity] len=4000 manacher_cmps=7999 brute_cmps=8002000 ratio=1000.375x
[complexity] len=8000 manacher_cmps=15999 brute_cmps=32004000 ratio=2000.375x
[complexity] len=16000 manacher_cmps=31999 brute_cmps=128008000 ratio=4000.375x
[SUMMARY] known-fail=0 random-fail=0 cmp-ratio=2750.472x => PASS
```

## 技术要点
- **哨兵填充（Padding）**：在每两个字符之间、以及首尾插入 `#`，把任意长度的回文统一为「奇数长度、单一中心」形式，消除奇偶回文分支，使中心扩展逻辑统一
- **镜像复用（Mirror Reuse）**：维护当前最右回文的中心 `C` 与右边界 `R`。对每个中心 `i`，若 `i < R`，其关于 `C` 的镜像 `mirr = 2C - i` 的半径 `p[mirr]` 可被安全复用，即 `p[i] = min(R - i, p[mirr])`，从而跳过大量冗余扩展，实现均摊 O(n)
- **中心扩展 + 边界更新**：镜像初值之后，仍从 `i + p[i] + 1` 继续向外暴力扩展；一旦 `i + p[i] > R`，更新 `C = i, R = i + p[i]`，保证右边界单调右移，是线性复杂度的核心不变量
- **映射回原串**：回文在填充串中的中心 `center` 与半径 `radius` 映射回原串 `start = (center - radius) / 2`，`length = radius`

## 量化验证（非视觉检查）
1. **已知样例**：7 组手写用例（`babad`、`cbbd`、`racecar`、全 `a` 串等）长度与回文性质全部通过
2. **随机正确性**：5000 组随机串（长度 1~60、小字母表以制造大量回文），Manacher 与 O(n²) 暴力中心扩展逐一比对，长度一致且结果确为回文，0 失败
3. **复杂度加速比**：最坏情况（全同字符串）下统计字符比较次数，长度 16000 时 Manacher ~3.2e4 次 vs 暴力 ~1.28e8 次，加速比高达 4000x，且随 n 线性增长，实证 O(n) vs O(n²)
