# AVL Tree Self-Balancing BST — 自平衡二叉搜索树

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果
```
[PASS] std::set correctness: inorder sequence matches reference (120940 elements)
[PASS] balance factor invariant holds after 100000 insertions
  height=20  theoretical_bound~23.5899  log2(n)=16.6096
[PASS] height within theoretical O(log n) bound
[PASS] balanced after sorted (worst-case) insertion
[PASS] height stays logarithmic for sorted input: 16
  AVL:       insert=92ms lookup=0ms erase=75ms
  std::set:  insert=64ms lookup=0ms erase=73ms
[PASS] structure empty & consistent after bulk erase
========================================
ALL TESTS PASSED
```

## 技术要点
- **平衡因子（Balance Factor）不变式**：每个节点 `bf = height(left) - height(right)` 必须落在 `[-1, 1]` 区间内，插入/删除后通过旋转恢复平衡
- **四种旋转**：LL（右旋）、RR（左旋）、LR（先左旋再右旋）、RL（先右旋再左旋），删除时根据子节点平衡因子符号选择单旋或双旋
- **AVL 高度上界**：`h ≤ 1.44·log₂(n+2) − 0.328`，保证所有操作 O(log n)；本实现实测 height=20 远低于理论界 23.59
- **最坏情况退化测试**：对严格递增（sorted）输入插入 5 万元素，高度仍保持对数级 16，不会被退化成链表
- **正确性量化验证**：与 `std::set` 做 20 万次混合插入/删除的中序序列逐元素对比（120940 元素完全一致），并逐层校验 BST 序 + 高度字段一致性
- **性能量化**：30 万次插入/查找/删除与 `std::set`（红黑树）对比，量级相当（AVL 更强调严格平衡，查询略优）
