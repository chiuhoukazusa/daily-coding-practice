# Minimum Spanning Tree: Prim vs Kruskal

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果
![MST Comparison](mst_output.png)

## 技术要点
- **Prim算法**: 贪心算法，从单点出发逐步扩展，使用优先队列 O(E log V)
- **Kruskal算法**: 按权重排序所有边，使用并查集 O(E log E)
- **验证**: 两种算法结果一致（权重相同），MST连通性检查，最优性验证
- **可视化**: 左半部分Prim结果，右半部分Kruskal结果，背景显示完整图

## 量化验证
- 30个顶点，108条边
- MST边数=29 (V-1=29) ✓
- Prim/Kruskal权重一致性 ✓
- 连通性检查 ✓
- 最优性验证（所有边替换检查） ✓
- 图片统计：均值247.3，标准差31.0 ✓
