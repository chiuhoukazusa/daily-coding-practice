# Subsurface Scattering (SSS) Renderer

次表面散射渲染器 — 软光栅化实现

## 技术要点

- **Dipole Diffusion Model** (Jensen et al. 2001) — 双极散射模型
- **BSSRDF** — 双向次表面散射反射分布函数
- **Sum of Gaussians** — 高斯混合散射剖面近似
- 四种材质：皮肤 (Skin)、蜡 (Wax)、大理石 (Marble)、牛奶 (Milk)
- Blinn-Phong 高光 + 球面采样积分

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2
./output
```

## 输出结果

![SSS渲染结果](sss_output.png)

## 核心原理

次表面散射通过对球面采样点的散射剖面加权积分实现：

```
R(r) = Σ w_i * G(r, σ_i)   // 高斯混合散射剖面
```

不同材质通过调整高斯权重和标准差来模拟不同的光散射距离。
