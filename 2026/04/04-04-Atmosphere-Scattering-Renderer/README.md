# Atmosphere Scattering Renderer

实现基于物理的大气散射模型，包含 Rayleigh 和 Mie 散射，渲染黎明、正午、日落三种天空效果。

## 编译运行
```bash
g++ main.cpp -o output -std=c++17 -O2
./output
# 生成 atmosphere_output.ppm，再用 Pillow 转 PNG
```

## 输出结果
![大气散射渲染结果](atmosphere_output.png)

三个面板从左到右：黎明（太阳仰角 5°）、正午（60°）、日落（-2°）。

## 技术要点
- Rayleigh 散射：波长相关系数 (5.8/13.5/33.1 × 10⁻⁶)，模拟天空蓝色
- Mie 散射：Henyey-Greenstein 相位函数（g=0.76），模拟日晕与光晕
- 单次散射积分：16 步主光线 + 8 步光照射线，逐采样点计算消光
- ACES Filmic 色调映射：保留高动态范围细节
- 大气层模型：地球半径 6360km，大气层顶 6420km，Rayleigh/Mie 标高 8km/1.2km
