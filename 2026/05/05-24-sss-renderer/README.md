# Subsurface Scattering Renderer

基于 Jensen SSS 偶极子模型（Dipole Approximation）的次表面散射渲染器。

## 技术要点

- **Jensen SSS 偶极子模型**：使用真实物理参数模拟光在散射介质内传播
- **漫射轮廓函数 Rd(r)**：积分光照在整个物体表面的贡献
- **预计算 LUT**：256 点 1D 查找表加速运行时采样
- **半透明阴影（Translucent Shadows）**：阴影区域仍有 SSS 散射亮度
- **材质预设**：皮肤、大理石、蜡/蜡烛、牛奶，各材质散射参数参考 Jensen 2001
- **ACES Filmic 色调映射** + Gamma 校正
- **软光栅化**：CPU 光线追踪，3×3 MSAA

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2
./output
# 生成 sss_output.ppm，使用 PIL 转 PNG：
python3 -c "from PIL import Image; Image.open('sss_output.ppm').save('sss_renderer_output.png')"
```

## 输出结果

![SSS渲染结果](sss_renderer_output.png)

## 参考文献

- H. W. Jensen et al. "A Practical Model for Subsurface Light Transport" (SIGGRAPH 2001)
- E. d'Eon & D. Luebke. "Advanced Techniques for Realistic Real-Time Skin Rendering" (GPU Gems 3)
