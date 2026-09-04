# Levenshtein Edit Distance (Wagner-Fischer DP)

**日期**: 2026-09-05
**方向**: 算法 / 动态规划
**核心技术**: Wagner-Fischer 动态规划、最短编辑距离、回溯对齐、滚动数组 O(min(m,n)) 空间优化

## 实现内容

1. **完整 DP 矩阵** (`wagner_fischer_full`) — O(m·n) 空间，含回溯，输出一条最优对齐（插入/删除/替换/匹配序列 + 对齐后的两行）。
2. **滚动数组优化** (`wagner_fischer_optimized`) — O(min(m,n)) 空间。
3. **独立备忘录递归** (`edit_distance_memo`) — 自顶向下独立实现，用于交叉验证矩阵版正确性。
4. **量化验证**（不靠眼睛）：
   - 12 组功能用例（三种实现交叉一致）
   - 10 组经典参考值精确断言（kitten/sitting=3, intention/execution=5 等）
   - 数学性质：自反性、对称性、距离上下界、三角形不等式（各随机/多样本验证）
   - 性能对比：滚动数组 vs 完整矩阵（2000×2000）

## 验证结果

- 编译：0 errors, 0 warnings (`g++ -std=c++17 -O2 -Wall -Wextra`)
- 功能用例：12/12 通过
- 经典参考值：10/10 通过
- 自反性：6/6；对称性：5/5；上下界：200/200；三角形不等式：200/200
- 一致性：滚动数组 == 完整矩阵 == 备忘录
- ✅ 全部量化验证通过

## 运行

```bash
g++ edit_distance.cpp -o edit_distance -std=c++17 -O2 -Wall -Wextra
./edit_distance
```
