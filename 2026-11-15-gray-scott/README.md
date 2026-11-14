# Reaction-Diffusion (Gray-Scott) Turing Pattern

## 简介
模拟 Gray-Scott 反应扩散系统，生成图灵斑图（Turing Pattern）。通过显式有限差分法（9 点离散拉普拉斯算子）在 2D 网格上求解双变量耦合偏微分方程，验证反应扩散方程自发形成空间图案（斑点/条纹/迷宫）的经典定性结论。

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果
- V 浓度场可视化：`gray_scott_output.png`（viridis 配色，蓝→青→黄）

## 技术要点
- Gray-Scott 动力学：`du/dt = Du∇²u − uv² + F(1−u)`、`dv/dt = Dv∇²v + uv² − (F+k)v`
- 图灵不稳定性条件：Du > Dv（扩散差异驱动图案自发形成）
- 5 点离散拉普拉斯算子 + 显式欧拉时间步进（dt=1 满足稳定性约束）
- 定量验证（非目测）：
  - 有界性：0 ≤ u, v ≤ 1，无负浓度、无发散
  - 图案形成：V 场空间方差 > 阈值，径向自相关提取特征波长（14px）
  - 参数扫描：(F,k) 不同组合复现斑点/条纹/均质等形态分相边界
