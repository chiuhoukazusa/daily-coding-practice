# SPH Fluid Simulation 流体模拟

## 项目描述

基于 **Smoothed Particle Hydrodynamics (SPH)** 方法的 2D 流体模拟。
实现了完整的 SPH 粒子流体系统，包括密度计算、压力梯度、粘性力和重力，
并将模拟结果渲染为彩色粒子图像（颜色映射速度大小）。

## 技术要点

- **Poly6 核函数**：计算粒子密度
- **Spiky 核函数梯度**：计算压力梯度力
- **Viscosity 核函数拉普拉斯**：计算粘性力
- **Leapfrog 积分**：速度 + 位置更新
- **边界碰撞**：阻尼反弹处理
- **颜色映射**：速度 → 蓝青绿黄红热力图

## 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 粒子数 | 600 | N_PARTICLES |
| 光滑半径 h | 0.2 | 核函数支撑域 |
| 时间步长 dt | 0.001s | 积分步长 |
| 模拟总时间 | 2s | STEPS=2000 |
| 重力 | 9.8 m/s² | 向下 |
| 气体刚度 k | 50 | GAS_CONST |
| 动力粘度 | 0.1 | VISC |

## 编译运行

```bash
g++ sph_fluid.cpp -o sph_fluid -O2 -std=c++17 -lm
./sph_fluid
```

**依赖**：stb_image_write.h（已包含，无其他依赖）

## 输出结果

- `sph_output.png` — 最终帧（2.0秒时刻）
- `sph_seq_00.png` — t=0.0s（初始方块）
- `sph_seq_01.png` — t=0.5s（开始扩散）
- `sph_seq_02.png` — t=1.0s（中间状态）
- `sph_seq_03.png` — t=1.5s（趋于稳定）
- `sph_seq_04.png` — t=2.0s（最终状态）

颜色含义：🔵蓝色=低速，🟡黄色=中速，🔴红色=高速

## 验证结果

```
Out-of-bounds: 0 / 600      ✅ 所有粒子在边界内
Density: avg=3.6 min=0.8   ✅ 密度合理
Speed: avg=0.303 max=1.312  ✅ 速度稳定
Bright pixels: 6.4%         ✅ 图像渲染正常
Bottom 1/3 colored: 41727   ✅ 重力效果正确（流体沉底）
```

## 算法参考

- Müller, M. et al. (2003). "Particle-Based Fluid Simulation for Interactive Applications"
- 核函数公式：Poly6 (密度), Spiky (压力), Viscosity (粘性)

## 代码仓库

GitHub: https://github.com/chiuhoukazusa/daily-coding-practice/tree/main/2026/03/03-09-sph-fluid-simulation
