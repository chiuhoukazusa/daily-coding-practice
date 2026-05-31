# Displacement Mapping & Tessellation Renderer

基于光线步进 (Ray Marching) 的位移贴图地形渲染器，展示了程序化高度场生成、精确法线重计算和 PBR 着色的完整流程。

## 编译运行

```bash
g++ main.cpp -o displacement_renderer -std=c++17 -O2
./displacement_renderer
```

## 输出结果

![渲染结果](https://raw.githubusercontent.com/chiuhoukazusa/blog_img/main/2026/06/06-01-displacement-mapping/displacement_output.png)

## 技术要点

- **程序化高度场**：FBM 分形布朗运动（8 倍频 Perlin Noise）+ 高斯山峰叠加
- **光线步进**：自适应步长 Ray Marching（近似 Sphere Tracing）精确穿透地形
- **法线重计算**：通过相邻高度差分计算准确表面法线（替代顶点法线）
- **分层地形着色**：8 层高度分级颜色（深水 → 浅水 → 沙滩 → 草地 → 岩石 → 雪顶）
- **软阴影 (PCF)**：光线步进阴影射线，k=8 半影柔化
- **PBR 着色**：GGX 微面元 BRDF（D/G/F 项），Cook-Torrance 镜面 + Lambert 漫射
- **水面渲染**：水域低粗糙度 + 动态法线扰动（Perlin 扰动模拟水波）
- **大气雾效**：距离指数衰减 + 地平线颜色混合
- **ACES Filmic Tone Mapping**：HDR 到 LDR 的电影级色调映射
