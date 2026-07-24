# Alpha-Beta Pruning Game AI

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果
```
============================================
   Alpha-Beta Pruning - Tic-Tac-Toe AI
   Date: 2026-07-25
============================================

Test 1 ✅ PASS (optimal result=0, ab_nodes < mm_nodes)
Test 2 ✅ PASS (ab_nodes <= mm_nodes for all depths)
Test 3 ✅ PASS (optimal vs optimal = all draws)
Test 4 ✅ PASS (optimal X never loses to random O)
Test 5 ✅ PASS (optimal O never loses to random X)
Test 6 ✅ PASS (different orderings still produce valid results)

Overall: ALL TESTS PASSED ✅
```

## 技术要点
- Minimax 搜索算法 — 零和博弈树完整搜索
- Alpha-Beta 剪枝 — 节点数减少 56.8%~96.7%，搜索深度验证
- 自对弈验证 — 最优策略对弈保证平局（Tic-Tac-Toe 已解游戏）
- 对称策略对比 — Optimal vs Random 零败率证明 AI 正确性
- 移动排序敏感性 — 验证剪枝对搜索顺序的依赖
- 性能对比 — Minimax 空盘 549,946 节点 vs Alpha-Beta 18,297 节点
