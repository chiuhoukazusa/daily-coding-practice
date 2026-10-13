# Markov Chain N-Gram Text Generation — 马尔可夫链文本生成

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
```
=== Markov Chain N-Gram Text Generation ===
N (gram size)   = 3
Corpus length   = 262 chars
Distinct contexts (states) = 79

[Verify 1] Transition probability normalization:
  total transitions counted = 260
  all rows sum to exactly 1.0 : PASS
  transitions == |windows| = 260 : PASS

[Verify 2] Unseen context handled gracefully:
  'zzz' not in model (generation stops) : PASS

[Verify 3] Reproducibility (same seed = same output):
  3 runs with seed=12345 identical : PASS

[Verify 4] All generated N-grams are valid in training data:
  PASS

[Verify 5] Model cross-entropy / perplexity (Laplace smoothed):
  vocabulary size       = 27
  eval windows          = 260
  cross-entropy (bits)  = 2.994662
  perplexity            = 7.970456
  perplexity < V+1 (model fits better than uniform) : PASS

=== Generated sample (len=200) ===
the quick be is the is the question winter is a rose lazy dog a rembe quick brown fox jumps or not to brown fox jumps over the nor is cominters

=== OVERALL: ALL PASS ===
```

## 技术要点
- **N-Gram 马尔可夫链**：用 `(n-1)` 个字符的上下文预测下一字符，学习条件概率分布 `P(next | context)`。trigram（N=3）建模二阶依赖，兼顾语料稀疏性与语法局部性
- **转移概率矩阵**：`unordered_map<context, unordered_map<char, count>>` 稀疏存储，每个上下文构成一个离散概率分布，通过计数归一化得到转移概率
- **加权随机采样**：构建累加 CDF + `lower_bound` 二分，模拟 `P(next|ctx)` 的离散采样，等效于轮盘赌选法，O(log V) 每步
- **滑动窗口上下文更新**：生成每步后取输出末尾 `(n-1)` 字符作为新上下文，实现状态沿马尔可夫链转移
- **Laplace 平滑**：`(count+1)/(total+V)` 处理未见的字符事件，避免零概率导致 `-inf` 对数似然

## 量化验证（非视觉检查）
1. **概率归一化**：每个上下文的转移概率之和严格等于 1.0（浮点误差 < 1e-9），且总转移次数等于语料窗口数 `|corpus| - N + 1 = 260`
2. **零概率/未见上下文处理**：查询不存在的上下文 `"zzz"` 时生成优雅停止，不崩溃不产生非法输出
3. **可复现性**：固定 seed=12345 三次运行输出逐字符一致，验证随机采样引擎的确定性
4. **生成合法性**：生成文本的每个 N-gram 都必须在训练语料中出现过（上下文存在且下一字符计数 > 0），杜绝幻觉
5. **交叉熵 / 困惑度**：Laplace 平滑下每字符交叉熵 2.99 bits，困惑度 7.97，远小于词表 27，证明模型对语料的拟合显著优于均匀分布
