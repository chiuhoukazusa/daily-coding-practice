# Depth of Field Renderer

基于薄透镜模型（Thin Lens Model）实现的物理正确景深渲染器，展示不同距离球体的散景（Bokeh）效果。

## 编译运行

```bash
# 需要 stb_image_write.h（从父目录获取或自行下载）
g++ main.cpp -o output -std=c++17 -O2
./output
# 输出: dof_output.png (800x600, ~1.1MB)
```

## 输出结果

![景深渲染结果](dof_output.png)

## 技术要点

- **薄透镜模型 (Thin Lens Model)**：相机光圈在透镜平面随机采样，模拟物理光圈
- **焦平面控制 (Focus Plane)**：通过 `focusDist` 参数精确控制清晰焦点位置
- **圆形散景 (Circular Bokeh)**：光圈采样使用单位圆均匀分布，产生自然圆形模糊
- **Monte Carlo 路径追踪**：128 SPP，余弦加权半球采样
- **多材质支持**：漫反射（Lambertian）、金属（镜面+粗糙度）、玻璃（Schlick菲涅尔+折射）、自发光
- **ACES Filmic 色调映射** + Gamma 2.2 校正

## 场景描述

- **前景（z=1.5，焦点处）**：红色漫反射球、玻璃球、金色金属球 → 清晰
- **中景（z=-0.5）**：蓝色漫反射球、银绿色金属球 → 轻微模糊
- **背景（z=-4.0）**：橙色球、绿色球 → 明显散景
- **天空光** + 暖色发光球（全局照明）
