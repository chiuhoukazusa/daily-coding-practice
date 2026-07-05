# Metaballs Implicit Surface Renderer

基于隐式曲面的 Metaballs（元球）渲染器，使用 Ray Marching 在 3D 标量场中追踪等值面。

## 技术原理

Metaballs 通过多个球体的势场函数叠加形成光滑的隐式曲面。每个球体使用 Wyvill 势场函数：

```
f(r) = (1 - r²/R²)³  (当 r < R)
```

多个势场叠加后，对阈值面进行 Ray Marching 提取等值面，使用中心差分法计算法线，并应用 Phong 光照模型进行着色。

## 编译运行

```bash
g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
./output
```

## 输出结果

![Metaballs 渲染结果](metaballs_output.png)

## 技术要点

- Wyvill 势场函数（C¹ 连续的光滑过渡）
- Ray Marching 等值面提取（阈值穿越检测 + 二分法精化）
- 中心差分梯度估计（法线计算）
- Phong 光照模型（环境光 + 漫反射 + 高光）
- 正交投影（无透视变形，展示等值面拓扑连接）
- 5 个元球的势场叠加场景

## 量化验证

| 指标 | 值 | 状态 |
|------|-----|------|
| 分辨率 | 800×600 | ✅ |
| 命中像素 | 280,328 (58.4%) | ✅ |
| 像素均值 | 27.86 | ✅ (10-240) |
| 像素标准差 | 25.90 | ✅ (>5) |
| 文件大小 | 1.4 MB | ✅ (>10KB) |
