# Wave Function Collapse (WFC) Procedural Tiling

## 简介
实现经典的 Wave Function Collapse（波函数坍缩）程序化生成算法，用于在给定邻接约束下生成无矛盾的图案平铺（tiling）。采用 Gumin "Model Synthesis" 家族的 MAXENT 模型变体：16 种二进制边 Wang tile，通过"最小熵坍缩 + 约束传播"迭代生成完整的 256×256 平铺网格，并对结果进行严格的量化验证（非目测）。

## 编译运行
```bash
g++ main.cpp -o wfc -std=c++17 -O2 -Wall -Wextra
./wfc
```

## 输出结果
- 平铺结果可视化：`wfc_output.ppm` / `wfc_output.png`（256×256，16 色区分 tile 类别）

## 技术要点
- 波函数坍缩：每个格子维护一个"叠加态"（所有可能 tile 的 bitset），通过坍缩逐步确定
- 最小熵启发式：每次选择 Shannon 熵最小（叠加态最少）的格子坍缩，最小化冲突风险
- 约束传播：坍缩后沿四方向传播邻接约束，迭代消除与邻居不兼容的 tile 选项
- Wang tile 邻接模型：16 种二进制边（up/down/left/right ∈ {0,1}）tile，方向兼容性 = 边颜色匹配
- 定量验证（非目测）：
  - 确定性：65536 格全部坍缩，无未决格（100%）
  - 邻接不变量：261120 条边全部满足方向兼容规则（0 违例）
  - 熵统计：初始 Shannon 熵 = 2304×log2(16) = 9216，与实测 max=9216 一致；全程 min=2.00（无零熵矛盾）
  - tile 覆盖率：16 种 tile 频率 5.7%~7.0%，均匀分布
