# SPH Fluid Simulation (Dam-Break)

A 2D Smoothed Particle Hydrodynamics (SPH) fluid simulation implementing the classic dam-break scenario.

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2 -lz
./output
```

## 输出结果

![SPH Fluid Simulation](sph_output.png)

Six frames showing the dam-break fluid evolution:
- Frame 0: Initial block at rest (blue = zero velocity)
- Frame 2: Peak velocity during collapse (red/yellow = fast)
- Frame 5: Fluid settled at bottom (blue = slow again)

## 技术要点

- **Poly6 核函数**：密度估计，平滑粒子密度场
- **Spiky 核函数梯度**：压力力计算（Müller et al. 2003）
- **粘性 Laplacian 核**：粘性扩散力
- **半隐式 Euler 积分**：时序稳定性
- **速度截断**：防止数值爆炸（V_MAX = 500 sim units/s）
- **Gaussian Splat 渲染**：软光栅化，粒子可视化
- **速度着色**：蓝(0) → 青 → 绿 → 黄 → 红(快)

## 物理参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 核半径 H | 12 px | 粒子影响范围 |
| 粒子质量 | 65 | 模拟单位 |
| 静止密度 | 1000 | 目标密度 |
| 气体常数 K | 8 | 压力刚度 |
| 动力粘度 μ | 6 | 粘性系数 |
| 时间步长 dt | 0.003 | 积分步长 |
| 粒子数 | 168 | dam-break初始布局 |

## References

- Müller, M., Charypar, D., & Gross, M. (2003). *Particle-based fluid simulation for interactive applications.* SCA '03.
